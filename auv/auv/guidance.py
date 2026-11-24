"""Lookahead line-of-sight guidance with integral action.

Frames: x=North, y=East (local NED). Heading 0=north, +clockwise.

Cross-track sign convention
---------------------------
Standing at the leg start and looking toward the leg end, e > 0 means the
vehicle is to the RIGHT of the line (clockwise side). Example: leg runs north,
vehicle east of the line -> e > 0. The guidance law subtracts atan(e/Delta)
from the leg bearing, so a vehicle on the right steers left (smaller heading,
back toward the line), and vice versa.

The vehicle cannot move sideways, so the integral term produces a crab angle
that cancels a steady cross-current instead of a standing offset.
"""
from __future__ import annotations
import math

from .types import NavState, wrap_pi, CAPTURE_RADIUS

_ZERO_LEG = 1e-6


class LosGuidance:
    def __init__(self, delta_min: float = 4.0, delta_max: float = 10.0,
                 kappa: float = 0.01, capture_radius: float = CAPTURE_RADIUS,
                 speed_for_delta_max: float = 0.714,
                 yaw_freeze: float = math.radians(30.0),
                 max_crab: float = math.radians(40.0),
                 cur_ff: float = 0.5, ff_speed: float = 0.514):
        self.delta_min = delta_min
        self.delta_max = delta_max
        self.kappa = kappa
        self.capture_radius = capture_radius
        self.speed_for_delta_max = speed_for_delta_max
        # feed-forward crab from estimated cross-current: asin(c_cross/ff_speed); 0 disables
        self.cur_ff = cur_ff
        self.ff_speed = ff_speed
        self.yaw_freeze = yaw_freeze
        # anti-windup: kappa*I/delta_max limited to tan(max_crab)
        self.int_max = (delta_max * math.tan(max_crab) / kappa) if kappa > 0 else 0.0
        self.a = (0.0, 0.0)
        self.b = (0.0, 0.0)
        self.integral = 0.0
        self.leg_len = 0.0
        self.chi = 0.0
        self._have_leg = False

    def set_leg(self, A, B) -> None:
        self.a = (float(A[0]), float(A[1]))
        self.b = (float(B[0]), float(B[1]))
        dn, de = self.b[0] - self.a[0], self.b[1] - self.a[1]
        self.leg_len = math.hypot(dn, de)
        if self.leg_len > _ZERO_LEG:
            self.chi = math.atan2(de, dn)
        self.integral = 0.0
        self._have_leg = True

    def lookahead(self, speed: float) -> float:
        f = min(max(speed / self.speed_for_delta_max, 0.0), 1.0)
        return self.delta_min + (self.delta_max - self.delta_min) * f

    def step(self, nav: NavState, dt: float, yaw: float | None = None):
        """Returns (psi_d, cross_track, along_track, arrived).

        yaw (optional): current heading, used only to freeze the integrator
        while the vehicle is still turning toward psi_d.
        """
        if not self._have_leg:
            raise RuntimeError("set_leg() must be called before step()")
        pn, pe = nav.n - self.a[0], nav.e - self.a[1]
        dist_b = math.hypot(nav.n - self.b[0], nav.e - self.b[1])
        speed = math.hypot(nav.vn, nav.ve)
        delta = self.lookahead(speed)

        if self.leg_len <= _ZERO_LEG:
            # degenerate leg: steer straight at B, no cross-track notion
            if dist_b > 1e-9:
                psi_d = math.atan2(self.b[1] - nav.e, self.b[0] - nav.n)
            else:
                psi_d = yaw if yaw is not None else self.chi
            return psi_d, 0.0, 0.0, dist_b <= self.capture_radius

        s, c = math.sin(self.chi), math.cos(self.chi)
        along = pn * c + pe * s
        cross = -pn * s + pe * c   # + = right of leg

        arrived = dist_b <= self.capture_radius or along >= self.leg_len

        # integral with freeze + clamp
        psi_pre = self.chi - math.atan((cross + self.kappa * self.integral) / delta)
        freeze = abs(cross) > 2.0 * delta
        if yaw is not None and abs(wrap_pi(psi_pre - yaw)) > self.yaw_freeze:
            freeze = True
        if not freeze and dt > 0:
            self.integral += cross * dt
            self.integral = max(-self.int_max, min(self.int_max, self.integral))

        crab = 0.0
        if self.cur_ff > 0 and self.ff_speed > 0:
            c_cross = -nav.cur_n * s + nav.cur_e * c     # + = current pushes toward the right
            crab = self.cur_ff * math.asin(max(-0.7, min(0.7, c_cross / self.ff_speed)))
        psi_d = wrap_pi(self.chi - math.atan((cross + self.kappa * self.integral) / delta) - crab)
        return psi_d, cross, along, arrived
