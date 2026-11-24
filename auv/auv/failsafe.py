"""Failsafe ladder. Pure logic, time taken from Sensors.t (never wall clock)."""
from __future__ import annotations
import math
from .types import (Sensors, NavState, FailLevel, MAX_DEPTH, CURRENT_ABORT,
                    BATT_CRIT_V)

ACTIONS = ("none", "continue_degraded", "return_breadcrumb", "blind_surface",
           "surface_now", "outputs_off")


class FailsafeMonitor:
    def __init__(self, max_depth: float = MAX_DEPTH, max_pitch: float = math.radians(45),
                 max_runtime: float = 3600.0, current_abort: float = CURRENT_ABORT,
                 dvl_degraded_s: float = 1.0, dvl_dead_s: float = 5.0,
                 att_timeout_s: float = 0.5, depth_timeout_s: float = 1.0,
                 gcs_timeout_s: float = 5.0, current_hold_s: float = 10.0,
                 blind_surface_s: float = 20.0, batt_crit_v: float = BATT_CRIT_V,
                 dvl_required: bool = True):
        self.max_depth = max_depth
        self.max_pitch = max_pitch
        self.max_runtime = max_runtime
        self.current_abort = current_abort
        self.dvl_degraded_s = dvl_degraded_s
        self.dvl_dead_s = dvl_dead_s
        self.att_timeout_s = att_timeout_s
        self.depth_timeout_s = depth_timeout_s
        self.gcs_timeout_s = gcs_timeout_s
        self.current_hold_s = current_hold_s
        self.blind_surface_s = blind_surface_s
        self.batt_crit_v = batt_crit_v
        self.dvl_required = dvl_required   # set False while parked on the surface
        self.reset()

    def reset(self):
        self._t_dvl = None
        self._t_att = None
        self._t_depth = None
        self._cur_since = None
        self._latched = {}          # FailLevel -> action
        self._h3_t0 = None
        self.reasons = []

    def _latch(self, level: FailLevel, action: str, why: str):
        if level not in self._latched:
            self._latched[level] = action
            self.reasons.append(why)

    def update(self, s: Sensors, nav: NavState, mission_elapsed: float):
        t = s.t
        # first call starts every timer (grace period)
        for a in ("_t_dvl", "_t_att", "_t_depth"):
            if getattr(self, a) is None:
                setattr(self, a, t)
        if s.dvl is not None and s.dvl.valid:
            self._t_dvl = t
        if s.att is not None and s.att.healthy:
            self._t_att = t
        if s.depth is not None and s.depth.valid:
            self._t_depth = t

        # --- latched severe conditions ---
        if t - self._t_att > self.att_timeout_s:
            if FailLevel.H3 not in self._latched:
                self._h3_t0 = t
            self._latch(FailLevel.H3, "blind_surface", "attitude lost")
        if t - self._t_depth > self.depth_timeout_s:
            self._latch(FailLevel.H4, "surface_now", "depth timeout")
        if s.depth is not None and s.depth.valid and s.depth.depth > self.max_depth:
            self._latch(FailLevel.H7, "surface_now", "depth limit")
        if s.att is not None and s.att.healthy and abs(s.att.pitch) > self.max_pitch:
            self._latch(FailLevel.H7, "surface_now", "pitch limit")
        if s.leak:
            self._latch(FailLevel.H7, "surface_now", "leak")
        if s.battery_v <= self.batt_crit_v:
            self._latch(FailLevel.H7, "surface_now", "battery critical")
        if mission_elapsed > self.max_runtime:
            self._latch(FailLevel.H7, "surface_now", "runtime exceeded")

        # --- current: must persist current_hold_s ---
        cur = math.hypot(nav.cur_n, nav.cur_e)
        if cur > self.current_abort:
            if self._cur_since is None:
                self._cur_since = t
            if t - self._cur_since > self.current_hold_s:
                self._latch(FailLevel.H7B, "surface_now", "current too strong")
        else:
            self._cur_since = None

        cands = [(lvl, act) for lvl, act in self._latched.items()]

        # --- non-latched (clear on good data) ---
        if self.dvl_required:
            gap = t - self._t_dvl
            if gap > self.dvl_dead_s:
                cands.append((FailLevel.H2, "return_breadcrumb"))
            elif gap > self.dvl_degraded_s:
                cands.append((FailLevel.H1, "continue_degraded"))
        if s.gcs_heartbeat_age > self.gcs_timeout_s:
            cands.append((FailLevel.H6, "surface_now"))

        if not cands:
            return FailLevel.H0, "none"
        level, action = max(cands, key=lambda c: c[0].value)
        # blind surface is time limited, then outputs off
        if FailLevel.H3 in self._latched and level.value <= FailLevel.H3.value:
            if t - self._h3_t0 > self.blind_surface_s:
                return FailLevel.H3, "outputs_off"
        return level, action
