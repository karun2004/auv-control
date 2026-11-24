"""Station keeping for a single-thruster finned torpedo (no hover, no sideways motion).

Modes
-----
hold_into_current  Point the nose into the (estimated) current and thrust so that water speed
                   ~ |current|: ground speed is then ~0 and the fins keep the flow they need
                   (their authority ~ u^2 of water speed). A slow P loop on the along-axis
                   position error sets a small ground-speed reference; a slow crab angle
                   (|theta| <= max_crab) removes the cross-axis error (lateral ground speed
                   = u*sin(theta)).
loiter             Current too weak for fins (mid-water, |c| < c_mid_min): swim a clockwise circle
                   of radius `loiter_radius` about the target at `loiter_speed` through water.
                   R must exceed the turn radius u/r_max (~7 m at 0.35 m/s with the sim fins).
float              At the surface with ~no current: thrust 0, positive buoyancy keeps us up.

Current estimate: nav.cur_* while the DVL is valid under water; otherwise (surface, no DVL)
c_hat = low-passed (ground velocity - modelled water velocity), because the nav filter cannot
observe current there.  The thrust model (drag_k, thrust_max, tau) is the same "perfect plant"
assumption as sim/run_sitl.py and must be re-identified on the real vehicle.

Frames/signs as in auv/types.py (NED, yaw 0=north clockwise, depth down, pitch + = nose up).
"""
from __future__ import annotations
import math

from .types import NavState, Attitude, AttitudeCmd, wrap_pi
from .controllers import DepthController, HeadingController, SPEED_FLOOR, _clamp

SURFACE_DEPTH = 0.3


class StationKeeper:
    def __init__(self, c_surface_min=0.05, c_mid_min=0.15, loiter_radius=10.0,
                 loiter_speed=0.35, k_pos=0.05, max_hold_speed=0.1, lat_len=10.0,
                 max_crab=math.radians(25.0), tau_c=10.0, loiter_delta=5.0, turn_err=math.radians(40.0), turn_speed=0.3,
                 drag_k=1.1 / 0.514 ** 2, thrust_max=20.0, tau_thrust=0.3,
                 kp_v=0.3, ki_v=0.01, depth: DepthController | None = None,
                 heading: HeadingController | None = None):
        self.c_surface_min, self.c_mid_min = c_surface_min, c_mid_min
        self.loiter_radius, self.loiter_speed = loiter_radius, loiter_speed
        self.k_pos, self.max_hold_speed = k_pos, max_hold_speed
        self.lat_len, self.max_crab = lat_len, max_crab
        self.tau_c, self.loiter_delta = tau_c, loiter_delta
        self.turn_err, self.turn_speed = turn_err, turn_speed
        self.drag_k, self.thrust_max, self.tau_thrust = drag_k, thrust_max, tau_thrust
        self.kp_v, self.ki_v = kp_v, ki_v
        self.depth = depth or DepthController(speed_floor=SPEED_FLOOR)
        self.heading = heading or HeadingController()
        self.reset()

    def reset(self):
        self.mode = None
        self._c = None          # (cn, ce) estimate
        self._T = 0.0           # modelled thrust N (first-order lag)
        self._u = 0.0           # modelled water speed
        self._vint = 0.0
        self.depth.reset()
        self.heading.reset()

    # ---- helpers ----
    def _update_model(self, cmd_thrust, dt):
        a = min(1.0, dt / self.tau_thrust)
        self._T += (cmd_thrust * self.thrust_max - self._T) * a
        self._u = max(0.0, self._u + (self._T - self.drag_k * self._u ** 2) / 30.0 * dt)

    def _current(self, nav, att, dt):
        under_dvl = nav.d > 0.5 and nav.dvl_age < 2.0
        if self._c is None or under_dvl:
            self._c = (nav.cur_n, nav.cur_e)
        else:
            meas = (nav.vn - self._u * math.cos(att.yaw), nav.ve - self._u * math.sin(att.yaw))
            a = min(1.0, dt / self.tau_c)
            self._c = (self._c[0] + a * (meas[0] - self._c[0]),
                       self._c[1] + a * (meas[1] - self._c[1]))
        return self._c

    def _pick_mode(self, cm, at_surface):
        lo = self.c_surface_min if at_surface else self.c_mid_min
        if self.mode == "hold_into_current":
            lo *= 0.8               # hysteresis
        if cm >= lo:
            return "hold_into_current"
        return "float" if at_surface else "loiter"

    # ---- main ----
    def step(self, nav: NavState, att: Attitude, target, dt: float):
        tn, te, td = target
        cn, ce = self._current(nav, att, dt)
        cm = math.hypot(cn, ce)
        at_surface = td <= SURFACE_DEPTH and nav.d < 2 * SURFACE_DEPTH
        self.mode = self._pick_mode(cm, at_surface)
        en, ee = tn - nav.n, te - nav.e
        dist = math.hypot(en, ee)
        status = {"mode": self.mode, "dist": dist, "radius_err": dist, "current": cm,
                  "speed_ok": True}
        speed_wat = max(0.0, (nav.vn - cn) * math.cos(att.yaw) + (nav.ve - ce) * math.sin(att.yaw))
        pitch = 0.0 if at_surface else self.depth.step(td, nav, speed_wat, dt)

        if self.mode == "float":
            yaw = self.heading.step(att.yaw, att.yaw, dt)
            thrust = 0.0
            self._vint = 0.0
        elif self.mode == "hold_into_current":
            psi_up = math.atan2(-ce, -cn)
            f0 = (math.cos(psi_up), math.sin(psi_up))
            r0 = (-math.sin(psi_up), math.cos(psi_up))
            err_f = en * f0[0] + ee * f0[1]
            err_r = en * r0[0] + ee * r0[1]
            theta = _clamp(math.atan2(err_r, self.lat_len), -self.max_crab, self.max_crab)
            yaw = self.heading.step(wrap_pi(psi_up + theta), att.yaw, dt)
            v_ref = _clamp(self.k_pos * err_f, -self.max_hold_speed, self.max_hold_speed)
            if abs(wrap_pi(psi_up + theta - att.yaw)) > self.turn_err:
                # not yet facing the current: swim at turn_speed so the fins can rotate us
                u_ref = self.turn_speed
                thrust = _clamp(self.drag_k * u_ref ** 2 / self.thrust_max
                                + self.kp_v * (u_ref - speed_wat), 0.0, 1.0)
            else:
                v_f = nav.vn * f0[0] + nav.ve * f0[1]
                u_ref = max(0.0, (cm + v_ref) / math.cos(theta))
                ff = self.drag_k * u_ref ** 2 / self.thrust_max
                verr = v_ref - v_f
                unsat = ff + self.kp_v * verr + self.ki_v * self._vint
                thrust = _clamp(unsat, 0.0, 1.0)
                if thrust == unsat and dt > 0:
                    self._vint = _clamp(self._vint + verr * dt, -5.0, 5.0)
        else:  # loiter
            R = self.loiter_radius
            phi = math.atan2(-ee, -en)          # bearing centre -> vehicle
            psi_d = wrap_pi(phi + math.pi / 2 + math.atan((dist - R) / self.loiter_delta))
            yaw = self.heading.step(psi_d, att.yaw, dt)
            status["radius_err"] = dist - R
            verr = self.loiter_speed - speed_wat
            unsat = (self.drag_k * self.loiter_speed ** 2 / self.thrust_max
                     + self.kp_v * verr + self.ki_v * self._vint)
            thrust = _clamp(unsat, 0.0, 1.0)
            if thrust == unsat and dt > 0:
                self._vint = _clamp(self._vint + verr * dt, -5.0, 5.0)
        status["speed_ok"] = speed_wat >= SPEED_FLOOR or self.mode == "float"
        self._update_model(thrust, dt)
        return AttitudeCmd(0.0, pitch, yaw, thrust, surface=at_surface), status
