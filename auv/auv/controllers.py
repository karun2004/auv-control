"""Depth, heading and combined attitude controller.

Only attitude + thrust are commanded, never position. The vehicle climbs or
dives only by pitching while moving forward, so a minimum forward speed is
needed for the fins to have authority.

Sign conventions: depth positive down. AttitudeCmd.pitch positive = nose UP
(nose up with forward speed -> climbs -> depth decreases).
"""
from __future__ import annotations
import math

from .types import (NavState, Attitude, AttitudeCmd, Waypoint, wrap_pi,
                    CRUISE_SPEED, CAPTURE_RADIUS)
from .guidance import LosGuidance

SPEED_FLOOR = 0.25          # m/s through water; below this fins lack authority


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class DepthController:
    def __init__(self, kp=0.4, ki=0.03, kd=0.4,
                 pitch_limit=math.radians(20.0), int_limit=30.0,
                 speed_floor=SPEED_FLOOR):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.pitch_limit = pitch_limit
        self.int_limit = int_limit      # clamp on integral of error, m*s
        self.speed_floor = speed_floor
        self.integral = 0.0
        self.speed_ok = True

    def reset(self):
        self.integral = 0.0

    def step(self, depth_ref: float, nav: NavState, speed: float, dt: float) -> float:
        """Pitch command (rad, + nose up). err = ref - depth (+ = too shallow).

        Too shallow -> negative pitch (nose down). Derivative uses nav.vd
        (d(err)/dt = -vd for constant ref), not differentiated depth.
        """
        self.speed_ok = speed >= self.speed_floor
        err = depth_ref - nav.d
        unsat = -(self.kp * err + self.ki * self.integral) + self.kd * nav.vd
        sat = _clamp(unsat, -self.pitch_limit, self.pitch_limit)
        # integrate only when it helps: not saturated against err, and fins effective
        pushing_out = (sat != unsat) and ((unsat > 0) == (-err > 0))
        if self.speed_ok and not pushing_out and dt > 0:
            self.integral = _clamp(self.integral + err * dt, -self.int_limit, self.int_limit)
        return _clamp(-(self.kp * err + self.ki * self.integral) + self.kd * nav.vd,
                      -self.pitch_limit, self.pitch_limit)


class HeadingController:
    """Passes psi_d through as absolute yaw command, rate limited, wrapped."""

    def __init__(self, max_rate=math.radians(30.0)):
        self.max_rate = max_rate
        self.cmd = None

    def reset(self, yaw=None):
        self.cmd = yaw

    def step(self, psi_d: float, yaw: float, dt: float) -> float:
        if self.cmd is None:
            self.cmd = yaw
        err = wrap_pi(psi_d - self.cmd)
        lim = self.max_rate * max(dt, 0.0)
        self.cmd = wrap_pi(self.cmd + _clamp(err, -lim, lim))
        return self.cmd


class Controller:
    def __init__(self, guidance: LosGuidance | None = None,
                 depth: DepthController | None = None,
                 heading: HeadingController | None = None,
                 cruise_speed=CRUISE_SPEED, speed_floor=SPEED_FLOOR,
                 thrust_ff=0.04, thrust_kp=0.3, thrust_ki=0.05,
                 slow_radius_factor=2.0):
        self.guidance = guidance or LosGuidance()
        self.depth = depth or DepthController(speed_floor=speed_floor)
        self.heading = heading or HeadingController()
        self.cruise_speed = cruise_speed
        self.speed_floor = speed_floor
        self.thrust_ff, self.thrust_kp, self.thrust_ki = thrust_ff, thrust_kp, thrust_ki
        self.slow_radius_factor = slow_radius_factor
        self._spd_int = 0.0
        self.idx = 0
        self.complete = False
        self._wps_id = None
        self._leg_start = None
        self._old = []

    def reset(self):
        self.idx = 0
        self.complete = False
        self._leg_start = None
        self._spd_int = 0.0
        self.depth.reset()
        self.heading.reset()

    def _new_list(self, wp_list):
        """Mission hands out a fresh slice as it advances. If the new head is a waypoint of the
        old list (slice advance), keep the corner as leg start instead of restarting the leg from
        the vehicle position, which would cut the corner."""
        old, key = self._old, (lambda w: (w.n, w.e))
        j = next((i for i, w in enumerate(old) if wp_list and key(w) == key(wp_list[0])), -1)
        if j >= 1 and self._leg_start is not None:
            A = key(old[j - 1])
            self.idx = 0
            self.complete = False
            self._start_leg(A, wp_list[0])
        else:
            self.reset()
        self._wps_id = id(wp_list)
        self._old = list(wp_list)

    def _start_leg(self, A, wp):
        self.guidance.set_leg(A, (wp.n, wp.e))
        self._leg_start = A

    def step(self, nav: NavState, att: Attitude, wp_list, dt: float):
        if self._wps_id != id(wp_list):
            self._new_list(wp_list)
        status = {"wp_index": self.idx, "mission_complete": False, "arrived": False,
                  "cross_track": 0.0, "along_track": 0.0, "psi_d": att.yaw,
                  "speed": 0.0, "speed_ok": True, "depth_err": 0.0, "dist_to_wp": 0.0}
        # speed through water along the heading
        u = max(0.0, (nav.vn - nav.cur_n) * math.cos(att.yaw) + (nav.ve - nav.cur_e) * math.sin(att.yaw))
        status["speed"] = u

        if self.complete or not wp_list or self.idx >= len(wp_list):
            self.complete = True
            status["mission_complete"] = True
            status["wp_index"] = len(wp_list) if wp_list else 0
            return AttitudeCmd(0.0, 0.0, att.yaw, 0.0), status

        wp = wp_list[self.idx]
        if self._leg_start is None:
            self._start_leg((nav.n, nav.e), wp)
        psi_d, xte, along, arrived = self.guidance.step(nav, dt, yaw=att.yaw)
        dist = math.hypot(wp.n - nav.n, wp.e - nav.e)

        if arrived:
            status["arrived"] = True
            self.idx += 1
            if self.idx >= len(wp_list):
                self.complete = True
                status["mission_complete"] = True
                status["wp_index"] = self.idx
                return AttitudeCmd(0.0, 0.0, att.yaw, 0.0), status
            self._start_leg((wp.n, wp.e), wp_list[self.idx])
            wp = wp_list[self.idx]
            psi_d, xte, along, _ = self.guidance.step(nav, 0.0, yaw=att.yaw)
            dist = math.hypot(wp.n - nav.n, wp.e - nav.e)

        yaw_cmd = self.heading.step(psi_d, att.yaw, dt)
        pitch = self.depth.step(wp.depth, nav, u, dt)

        # speed reference: cruise, slowing near capture, never below floor margin
        slow_r = self.slow_radius_factor * self.guidance.capture_radius
        v_ref = self.cruise_speed
        # slow only for the final waypoint: turn rate scales with u^2, so slowing at an
        # intermediate corner widens the turn and cuts cross-track accuracy on the next leg
        if dist < slow_r and self.idx == len(wp_list) - 1:
            v_ref = self.cruise_speed * dist / slow_r
        v_ref = max(v_ref, 1.2 * self.speed_floor)
        v_ref = min(v_ref, self.cruise_speed) if self.cruise_speed >= 1.2 * self.speed_floor else v_ref
        verr = v_ref - u
        unsat = self.thrust_ff + self.thrust_kp * verr + self.thrust_ki * self._spd_int
        thrust = _clamp(unsat, 0.0, 1.0)
        if thrust == unsat and dt > 0:
            self._spd_int = _clamp(self._spd_int + verr * dt, -5.0, 5.0)

        status.update(wp_index=self.idx, cross_track=xte, along_track=along, psi_d=psi_d,
                      speed_ok=self.depth.speed_ok, depth_err=wp.depth - nav.d,
                      dist_to_wp=dist)
        return AttitudeCmd(0.0, pitch, yaw_cmd, thrust), status
