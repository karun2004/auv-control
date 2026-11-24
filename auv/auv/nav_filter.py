"""DVL dead-reckoning navigation filter (EKF, numpy only). Local NED metres.

State x (10): [pn, pe, vn, ve, cn, ce, scale, yaw_bias, d, vd]
  pn,pe,vn,ve : ground-referenced position / velocity (DVL bottom track)
  cn,ce       : water current, slow random walk. Observable ONLY through
                update_water_track() (current = bottom_track - water_track).
                GPS does not touch it (no model couples it to position here).
  scale       : DVL scale factor, random walk, nominal 1.0
  yaw_bias    : compass yaw bias, random walk
  d, vd       : depth and vertical speed. vd comes from DVL vz; depth is never
                differentiated.

DVL model: v_ned = scale * R(roll, pitch, yaw - bias) @ v_body. The EKF measures
v_body with h(x) = R^T @ [vn,ve,vd] / scale, so scale and bias become observable
through the position/velocity cross-covariance when GPS fixes arrive.
"""
from __future__ import annotations
import math
import numpy as np
from .types import Sensors, NavState

CHI2_2DOF_999 = 13.82
DEPTH_SIGMA = 0.03 / math.sqrt(12.0)   # Bar100 3 cm quantisation
IDX = dict(pn=0, pe=1, vn=2, ve=3, cn=4, ce=5, sc=6, yb=7, d=8, vd=9)


def chi2_gate(nu, S, thresh: float = CHI2_2DOF_999):
    """Return (accepted, d2) with d2 = nu^T S^-1 nu. Accept iff finite and d2 <= thresh."""
    nu = np.asarray(nu, float).reshape(-1)
    S = np.atleast_2d(np.asarray(S, float))
    if not (np.all(np.isfinite(nu)) and np.all(np.isfinite(S))):
        return False, math.inf
    try:
        d2 = float(nu @ np.linalg.solve(S, nu))
    except np.linalg.LinAlgError:
        return False, math.inf
    return (d2 <= thresh), d2


def rot_body_to_ned(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr]])


class NavFilter:
    def __init__(self, n0=0.0, e0=0.0, fom_thresh=0.5, gps_hdop_max=2.0,
                 gps_min_sats=6, gps_min_epochs=5, gps_consistency_m=10.0,
                 chi2_thresh=CHI2_2DOF_999, dvl_grace_s=0.5):
        self.fom_thresh = fom_thresh
        self.gps_hdop_max = gps_hdop_max
        self.gps_min_sats = gps_min_sats
        self.gps_min_epochs = gps_min_epochs
        self.gps_consistency_m = gps_consistency_m
        self.chi2_thresh = chi2_thresh
        self.dvl_grace_s = dvl_grace_s
        self.x = np.zeros(10)
        self.x[0], self.x[1], self.x[6] = n0, e0, 1.0
        self.P = np.diag([1.0, 1.0, 0.1**2, 0.1**2, 0.2**2, 0.2**2,
                          0.02**2, 0.05**2, 0.1**2, 0.1**2])
        # process noise: variance growth per second
        self.q = np.array([0, 0, 0.02**2, 0.02**2, 0.005**2, 0.005**2,
                           1e-4**2, 1e-4**2, 0.005**2, 0.02**2], float)
        self.q_drop = 0.1**2           # extra vel variance/s while DVL dead
        self.dvl_sigma0, self.dvl_sigma_fom = 0.02, 0.1
        self.cur_sigma = 0.05
        self.t = None
        self.t_dvl = None              # time of last valid DVL
        self._t_start = None
        self.att = None                # (roll, pitch, yaw)
        self.surface = False
        self._streak: list = []
        self._outliers = 0
        self.gps_status = "none"       # last GPS decision, for logs/tests
        self.gps_d2 = math.nan
        self.gps_accepted = 0

    # ---------------- public ----------------
    def set_surface(self, flag: bool):
        """On surface: GPS updates allowed; depth pinned near 0 with shrunk
        depth/vz covariance. Entering surface restarts the GPS epoch count."""
        flag = bool(flag)
        if flag and not self.surface:
            self._streak, self._outliers = [], 0
        self.surface = flag

    def update_water_track(self, vn_water: float, ve_water: float, sigma: float = None):
        """Measure current = bottom_track(vn,ve state) - water_track(vn_water, ve_water).
        water_track is the vehicle's through-water velocity in NED (from e.g. a
        speed model x heading)."""
        if not (math.isfinite(vn_water) and math.isfinite(ve_water)):
            return
        r = (sigma or self.cur_sigma) ** 2
        for i_c, i_v, w in ((4, 2, vn_water), (5, 3, ve_water)):
            z = self.x[i_v] - w
            H = np.zeros(10); H[i_c] = 1.0
            self._kf(np.array([z]), np.array([self.x[i_c]]), H[None, :], np.array([[r]]))

    def update(self, s: Sensors) -> NavState:
        if self.t is None:
            self.t = s.t
            self._t_start = s.t
        dt = max(0.0, s.t - self.t)
        self.t = s.t
        if s.att is not None and all(map(math.isfinite, (s.att.roll, s.att.pitch, s.att.yaw))):
            self.att = (s.att.roll, s.att.pitch, s.att.yaw)
        self._predict(dt, s.t)
        if s.depth is not None and s.depth.valid and math.isfinite(s.depth.depth):
            self._depth_update(s.depth.depth)
        if self._dvl_ok(s.dvl) and self.att is not None:
            self._dvl_update(s.dvl)
            self.t_dvl = s.t
        if s.gps is not None:
            self._gps_update(s.gps)
        return self._state(s.t)

    # ---------------- internals ----------------
    def _dvl_ok(self, d) -> bool:
        if d is None or not d.valid:
            return False
        if d.fom is None or not math.isfinite(d.fom) or d.fom > self.fom_thresh:
            return False
        try:
            return len(d.v_body) == 3 and all(math.isfinite(v) for v in d.v_body)
        except TypeError:
            return False

    def _age(self, t):
        ref = self.t_dvl if self.t_dvl is not None else self._t_start
        return max(0.0, t - ref)

    def _predict(self, dt, t):
        if dt <= 0:
            return
        F = np.eye(10)
        F[0, 2] = F[1, 3] = F[8, 9] = dt
        self.x = F @ self.x
        q = self.q * dt
        if self._age(t) > self.dvl_grace_s:
            q[2] += self.q_drop * dt
            q[3] += self.q_drop * dt
            q[9] += self.q_drop * dt
        self.P = F @ self.P @ F.T + np.diag(q)
        if self.surface:                      # pinned at the surface
            self.x[8] = 0.0 if abs(self.x[8]) < 0.5 else self.x[8]
            self.P[8, :] *= 0.5; self.P[:, 8] *= 0.5
            self.P[9, :] *= 0.5; self.P[:, 9] *= 0.5

    def _kf(self, z, h, H, R):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - h)
        I_KH = np.eye(10) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def _depth_update(self, depth):
        H = np.zeros((1, 10)); H[0, 8] = 1.0
        self._kf(np.array([depth]), np.array([self.x[8]]), H, np.array([[DEPTH_SIGMA ** 2]]))

    def _h_dvl(self, x):
        roll, pitch, yaw = self.att
        R = rot_body_to_ned(roll, pitch, yaw - x[7])
        return R.T @ np.array([x[2], x[3], x[9]]) / x[6]

    def _dvl_update(self, d):
        z = np.array(d.v_body, float)
        h = self._h_dvl(self.x)
        H = np.zeros((3, 10))
        for i in (2, 3, 6, 7, 9):
            xp = self.x.copy(); eps = 1e-6
            xp[i] += eps
            H[:, i] = (self._h_dvl(xp) - h) / eps
        sig = self.dvl_sigma0 + self.dvl_sigma_fom * d.fom
        self._kf(z, h, H, np.eye(3) * sig ** 2)

    def _gps_quality(self, g) -> bool:
        return bool(g.valid and g.fix_type >= 3 and g.hdop < self.gps_hdop_max
                    and g.sats >= self.gps_min_sats
                    and math.isfinite(g.n) and math.isfinite(g.e))

    def _gps_update(self, g):
        if not self.surface:
            self.gps_status = "not_surface"; return
        if not self._gps_quality(g):
            self._streak, self._outliers = [], 0
            self.gps_status = "quality"; return
        if self._streak:
            m = np.mean(self._streak[-self.gps_min_epochs:], axis=0)
            if math.hypot(g.n - m[0], g.e - m[1]) > self.gps_consistency_m:
                self._outliers += 1
                if self._outliers >= 3:       # new consistent cluster: restart
                    self._streak, self._outliers = [(g.n, g.e)], 0
                self.gps_status = "inconsistent"; return
        self._outliers = 0
        self._streak.append((g.n, g.e))
        if len(self._streak) < self.gps_min_epochs:
            self.gps_status = "warming"; return
        H = np.zeros((2, 10)); H[0, 0] = H[1, 1] = 1.0
        r = max(1.5, 2.5 * g.hdop) ** 2
        R = np.eye(2) * r
        nu = np.array([g.n, g.e]) - self.x[:2]
        S = H @ self.P @ H.T + R
        ok, d2 = chi2_gate(nu, S, self.chi2_thresh)
        self.gps_d2 = d2
        if not ok:
            self.gps_status = "chi2"; return
        self._kf(np.array([g.n, g.e]), self.x[:2].copy(), H, R)   # Kalman, no teleport
        self.gps_status = "accepted"
        self.gps_accepted += 1

    def _state(self, t) -> NavState:
        x = self.x
        age = self._age(t)
        healthy = bool(age <= 5.0 and np.all(np.isfinite(x)) and np.all(np.isfinite(self.P)))
        return NavState(t=t, n=float(x[0]), e=float(x[1]), d=float(x[8]),
                        vn=float(x[2]), ve=float(x[3]), vd=float(x[9]),
                        cur_n=float(x[4]), cur_e=float(x[5]),
                        dvl_scale=float(x[6]), yaw_bias=float(x[7]),
                        cov_pos=(float(self.P[0, 0]), float(self.P[1, 1])),
                        healthy=healthy, dvl_age=float(age))
