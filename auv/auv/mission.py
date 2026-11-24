"""Mission state machine. Pure logic."""
from __future__ import annotations
import math
from enum import Enum
from .types import (Waypoint, FailLevel, BATT_LOW_V, CAPTURE_RADIUS)

EARTH_R = 6378137.0


def latlon_to_ned(lat, lon, lat0, lon0):
    """Small-area equirectangular conversion -> (north m, east m)."""
    n = math.radians(lat - lat0) * EARTH_R
    e = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    return n, e


class State(Enum):
    WAIT_GPS = 0
    ARM_CHECK = 1
    DIVE = 2
    RUN = 3
    RETURN = 4
    ABORT_SURFACE = 5
    SURFACE = 6
    FIX = 7
    SYNC = 8
    DONE = 9


class MissionManager:
    def __init__(self, breadcrumb=None, surface_depth=0.3, fix_dwell_s=30.0,
                 fix_timeout_s=120.0, sync_s=5.0, dive_timeout_s=300.0,
                 return_timeout_s=300.0, heartbeat_ok_s=5.0, gps_max_hdop=2.5,
                 gps_min_sats=6, gps_stale_s=2.0, heading_return_age_s=20.0):
        self.bc = breadcrumb
        self.surface_depth = surface_depth
        self.fix_dwell_s = fix_dwell_s
        self.fix_timeout_s = fix_timeout_s
        self.sync_s = sync_s
        self.dive_timeout_s = dive_timeout_s
        self.return_timeout_s = return_timeout_s
        self.heartbeat_ok_s = heartbeat_ok_s
        self.gps_max_hdop = gps_max_hdop
        self.gps_min_sats = gps_min_sats
        self.gps_stale_s = gps_stale_s
        self.heading_return_age_s = heading_return_age_s
        self.state = State.WAIT_GPS
        self.waypoints = []
        self.surface_after_each_leg = False
        self.name = ""
        self.idx = 0
        self.aborted = False
        self.abort_reason = ""
        self._timer = 0.0       # time in state / waypoint timer
        self._dwell = 0.0
        self._gps = None
        self._depth = None
        self._yaw = 0.0
        self._ret = {"waypoints": [], "legs": []}
        self._ret_mode = "return_positional"
        self._ret_idx = 0
        self._hold = None       # (n, e, depth) station-keeping target, set on entering FIX

    # ---- loading ----
    def load(self, m: dict, origin=None):
        origin = origin or m.get("origin")
        if isinstance(origin, dict):
            origin = (origin["lat"], origin["lon"])
        wps = []
        for w in m["waypoints"]:
            if "n" in w and "e" in w:
                n, e = w["n"], w["e"]
            elif "lat" in w and "lon" in w:
                if origin is None:
                    raise ValueError("lat/lon waypoints need an origin")
                n, e = latlon_to_ned(w["lat"], w["lon"], origin[0], origin[1])
            else:
                raise ValueError("waypoint needs n/e or lat/lon")
            wps.append(Waypoint(float(n), float(e), float(w["depth"]),
                                float(w.get("timeout_s", 600.0))))
        self.waypoints = wps
        self.name = m.get("name", "")
        self.surface_after_each_leg = bool(m.get("surface_after_each_leg", False))
        self.idx = 0

    # ---- helpers ----
    def _go(self, st):
        self.state = st
        self._timer = 0.0
        self._dwell = 0.0

    def _abort(self, why):
        if not self.aborted:
            self.aborted = True
            self.abort_reason = why
        self._go(State.ABORT_SURFACE)

    def _gps_good(self, t):
        g = self._gps
        return (g is not None and g.valid and g.fix_type >= 3 and g.hdop <= self.gps_max_hdop
                and g.sats >= self.gps_min_sats and t - g.t <= self.gps_stale_s)

    def _depth_sane(self):
        d = self._depth
        return d is not None and d.valid and -0.5 <= d.depth <= 1.0

    def _enter_return(self, nav):
        if self.bc is not None:
            self._ret = {"waypoints": self.bc.positional_return(),
                         "legs": self.bc.heading_return()}
        else:
            self._ret = {"waypoints": [], "legs": []}
        self._ret_idx = 0
        self._ret_mode = ("return_heading" if nav.dvl_age > self.heading_return_age_s
                          else "return_positional")
        self._go(State.RETURN)

    def _out(self, mode, wps=None, **extra):
        d = {"state": self.state, "mode": mode,
             "waypoints": wps if wps is not None else [],
             "done": self.state == State.DONE, "wp_index": self.idx,
             "aborted": self.aborted}
        d.update(extra)
        return d

    # ---- main ----
    def step(self, s, nav, fail, dt):
        level, action = fail
        t = s.t
        if s.gps is not None:
            self._gps = s.gps
        if s.depth is not None:
            self._depth = s.depth
        if s.att is not None:
            self._yaw = s.att.yaw
        self._timer += dt
        underwater_states = (State.DIVE, State.RUN, State.RETURN)
        if self.bc is not None and self.state in (State.DIVE, State.RUN):
            self.bc.record(t, nav.n, nav.e, nav.d, self._yaw, math.hypot(nav.vn, nav.ve))

        # failsafe escalation
        if self.state in underwater_states:
            if action in ("surface_now", "blind_surface", "outputs_off"):
                self._abort("failsafe " + level.name)
            elif action == "return_breadcrumb" and self.state != State.RETURN:
                self.aborted, self.abort_reason = True, "failsafe " + level.name
                self._enter_return(nav)

        st = self.state
        if st == State.WAIT_GPS:
            if self._gps_good(t):
                self._go(State.ARM_CHECK)
            return self._out("idle")

        if st == State.ARM_CHECK:
            ok = (s.battery_v > BATT_LOW_V and self._depth_sane()
                  and s.gcs_heartbeat_age <= self.heartbeat_ok_s and self._gps_good(t)
                  and level == FailLevel.H0 and bool(self.waypoints) and not s.leak)
            if ok:
                self._go(State.DIVE)
            elif not self._gps_good(t):
                self._go(State.WAIT_GPS)
            return self._out("idle")

        if st == State.DIVE:
            target = self.waypoints[self.idx].depth
            if nav.d >= max(0.5, target - 0.5):
                self._go(State.RUN)
            elif self._timer > self.dive_timeout_s:
                self._abort("dive timeout")
                return self._out("surface")
            return self._out("track", self.waypoints[self.idx:])

        if st == State.RUN:
            wp = self.waypoints[self.idx]
            if math.hypot(wp.n - nav.n, wp.e - nav.e) < CAPTURE_RADIUS:
                self.idx += 1
                self._timer = 0.0
                if self.idx >= len(self.waypoints) or self.surface_after_each_leg:
                    self._go(State.SURFACE)
                    return self._out("surface")
            elif self._timer > wp.timeout_s:
                self._abort("waypoint timeout")
                return self._out("surface")
            return self._out("track", self.waypoints[self.idx:])

        if st == State.RETURN:
            if self._ret_mode == "return_heading":
                legs = self._ret["legs"]
                # leg progression is the controller's job; we only time the whole return
                limit = sum(d for _, d in legs) * 1.3 + 10.0 if legs else self.return_timeout_s
                if self._timer > min(limit, self.return_timeout_s):
                    self._abort("return complete/timeout")
                    return self._out("surface")
                return self._out("return_heading", [], legs=legs)
            wps = self._ret["waypoints"]
            while self._ret_idx < len(wps) and math.hypot(
                    wps[self._ret_idx].n - nav.n, wps[self._ret_idx].e - nav.e) < CAPTURE_RADIUS:
                self._ret_idx += 1
            if self._ret_idx >= len(wps) or self._timer > self.return_timeout_s:
                self._abort("return complete/timeout")
                return self._out("surface")
            return self._out("return_positional", wps[self._ret_idx:])

        if st in (State.ABORT_SURFACE, State.SURFACE):
            if nav.d < self.surface_depth:
                self._hold = (nav.n, nav.e, 0.0)
                self._go(State.FIX)
            return self._out("surface")

        if st == State.FIX:
            if self._gps_good(t):
                self._dwell += dt
            else:
                self._dwell = 0.0
            fix = None
            if self._dwell >= self.fix_dwell_s:
                fix = (self._gps.n, self._gps.e)
            if fix is not None or self._timer > self.fix_timeout_s:
                self._go(State.SYNC)
                return self._out("hold", target=self._hold, gps_fix=fix)
            return self._out("hold", target=self._hold)

        if st == State.SYNC:
            if self._timer >= self.sync_s:
                more = (not self.aborted and self.idx < len(self.waypoints)
                        and self.surface_after_each_leg)
                self._go(State.DIVE if more else State.DONE)
            if self.state == State.DIVE:
                return self._out("track")
            return self._out("hold", target=self._hold)

        return self._out("idle")   # DONE
