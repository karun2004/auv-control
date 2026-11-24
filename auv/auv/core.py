"""AuvCore: 20 Hz outer-loop wiring of nav filter, controller, failsafe, mission and logger
to a Link (auv.mavlink_io.Link, FakeLink, or a sim adapter with the same interface).

Threading: run() owns the control loop in its own thread (start()/stop()). Other threads only
touch submit_mission(), abort() (queues, lock-protected) and snapshot() (returns the last
immutable-by-convention dict built at the end of a tick). No web call can stall a tick.

Wiring follows sim/run_sitl.py. Differences from the sim harness:
  * no water-track speed model (needs the plant parameters; Pi has only the DVL),
  * modes 'surface' -> request ArduSub SURFACE mode instead of commanding nose-up thrust,
  * 'hold'/'station_keep' mode -> auv.station_keep.StationKeeper if that module exists.
"""
from __future__ import annotations
import math
import os
import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Optional

from .types import (AttitudeCmd, Attitude, FailLevel, MAX_DEPTH)
from .nav_filter import NavFilter
from .controllers import Controller, DepthController
from .failsafe import FailsafeMonitor
from .mission import MissionManager, State
from .breadcrumb import BreadcrumbTrail

NAN = math.nan
MAX_WAYPOINTS = 50
MAX_RANGE_M = 5000.0
_SURFACE_STATES = (State.WAIT_GPS, State.ARM_CHECK, State.FIX, State.SYNC, State.DONE)
_IDLE_STATES = (State.WAIT_GPS, State.ARM_CHECK, State.DONE)

LOG_SCHEMA_VERSION = 1
LOG_COLUMNS = (
    "t_ns", "source",
    # validity flags (0/1). A sample that did not arrive this tick has flag 0 AND nan values.
    "f_dvl_valid", "f_att_healthy", "f_depth_valid", "f_gps_valid", "f_nav_healthy", "f_leak",
    # raw body-frame DVL
    "dvl_vx", "dvl_vy", "dvl_vz", "dvl_fom", "dvl_alt",
    "roll", "pitch", "yaw",
    "depth_raw", "depth",
    "nav_n", "nav_e", "nav_d", "nav_vn", "nav_ve", "nav_vd", "cur_n", "cur_e",
    "dvl_scale", "yaw_bias", "cov_n", "cov_e", "dvl_age",
    "gps_fix", "gps_hdop", "gps_sats", "gps_n", "gps_e",
    "battery_v", "gcs_hb_age",
    "xte", "along", "wp_index", "dist_to_wp",
    "mission_state", "fail_level",
    "cmd_roll", "cmd_pitch", "cmd_yaw", "cmd_thrust",
)
_STR_COLS = ("source", "mission_state")


def validate_mission(m, max_depth: float = MAX_DEPTH) -> dict:
    """Return a cleaned mission dict or raise ValueError. Accepts n/e or lat/lon (+origin)."""
    if not isinstance(m, dict):
        raise ValueError("mission must be an object")
    wps = m.get("waypoints")
    if not isinstance(wps, list) or not wps:
        raise ValueError("waypoints must be a non-empty list")
    if len(wps) > MAX_WAYPOINTS:
        raise ValueError(f"at most {MAX_WAYPOINTS} waypoints")
    origin = m.get("origin")
    if origin is not None:
        if not isinstance(origin, dict) or not all(_num(origin.get(k)) for k in ("lat", "lon")):
            raise ValueError("origin needs numeric lat and lon")
    out = []
    for i, w in enumerate(wps):
        if not isinstance(w, dict):
            raise ValueError(f"waypoint {i}: not an object")
        if "n" in w and "e" in w:
            if not (_num(w["n"]) and _num(w["e"])):
                raise ValueError(f"waypoint {i}: n/e must be finite numbers")
            if abs(w["n"]) > MAX_RANGE_M or abs(w["e"]) > MAX_RANGE_M:
                raise ValueError(f"waypoint {i}: beyond {MAX_RANGE_M:.0f} m of origin")
            c = {"n": float(w["n"]), "e": float(w["e"])}
        elif "lat" in w and "lon" in w:
            if origin is None:
                raise ValueError(f"waypoint {i}: lat/lon needs mission origin")
            if not (_num(w["lat"]) and _num(w["lon"]) and abs(w["lat"]) <= 90
                    and abs(w["lon"]) <= 180):
                raise ValueError(f"waypoint {i}: bad lat/lon")
            c = {"lat": float(w["lat"]), "lon": float(w["lon"])}
        else:
            raise ValueError(f"waypoint {i}: needs n/e or lat/lon")
        d = w.get("depth")
        if not _num(d) or not (0.0 < d <= max_depth):
            raise ValueError(f"waypoint {i}: depth must be in (0, {max_depth}] m")
        c["depth"] = float(d)
        to = w.get("timeout_s", 600.0)
        if not _num(to) or to <= 0:
            raise ValueError(f"waypoint {i}: timeout_s must be > 0")
        c["timeout_s"] = float(to)
        out.append(c)
    res = {"name": str(m.get("name", ""))[:64], "waypoints": out,
           "surface_after_each_leg": bool(m.get("surface_after_each_leg", False))}
    if origin is not None:
        res["origin"] = {"lat": float(origin["lat"]), "lon": float(origin["lon"])}
    return res


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


class CsvLogger:
    """One row per tick. Missing -> 'nan' (never 0). Header carries the wall-clock anchor:
        wall_utc (ISO) taken at the same instant as mono_ns, so
        wall = wall_anchor + (t_ns - mono_anchor_ns) / 1e9."""

    def __init__(self, path: str, source: str, flush_every: int = 20):
        self.path = path
        self.source = source
        self.flush_every = flush_every
        self._n = 0
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self._f = open(path, "w", buffering=1 << 16)
        mono_ns = time.monotonic_ns()
        wall = datetime.now(timezone.utc).isoformat()
        self._f.write(f"# auv_log schema_version={LOG_SCHEMA_VERSION}\n")
        self._f.write(f"# wall_anchor_utc={wall} mono_anchor_ns={mono_ns}\n")
        self._f.write("# missing values are written as nan, never 0; f_* are 0/1 validity flags\n")
        self._f.write(",".join(LOG_COLUMNS) + "\n")
        self._f.flush()

    @staticmethod
    def fmt(v) -> str:
        if v is None:
            return "nan"
        if isinstance(v, bool):
            return "1" if v else "0"
        if isinstance(v, float):
            return "nan" if not math.isfinite(v) else repr(v)
        return str(v)

    def write(self, row: dict):
        self._f.write(",".join(self.fmt(row.get(c)) if c not in _STR_COLS
                               else str(row.get(c, "nan")) for c in LOG_COLUMNS) + "\n")
        self._n += 1
        if self._n % self.flush_every == 0:
            self._f.flush()

    def close(self):
        try:
            self._f.flush()
            self._f.close()
        except Exception:
            pass


def _jsonable(o):
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    return o


class AuvCore:
    def __init__(self, link, *, rate_hz: float = 20.0, log_dir: Optional[str] = None,
                 source: str = "real", clock=time.monotonic, auto_arm: bool = True,
                 surface_resend_s: float = 2.0):
        self.link = link
        self.dt = 1.0 / rate_hz
        self.rate_hz = rate_hz
        self.clock = clock
        self.source = source
        self.auto_arm = auto_arm
        self.surface_resend_s = surface_resend_s
        self.log_dir = log_dir
        self._lock = threading.Lock()
        self._cmds = deque()                    # ("mission", dict) | ("abort", None)
        self._snap = {"running": False}
        self._thread = None
        self._stop = threading.Event()
        self.tick_count = 0
        self.overruns = 0
        self.last_error = ""
        self._logger = None
        self._sk = None
        self._sk_tried = False
        self._build()

    # ---------- construction ----------
    def _build(self):
        self.nf = NavFilter(0.0, 0.0)
        self.ctrl = Controller()
        self.depth_hold = DepthController()
        self.fs = FailsafeMonitor()
        self.bc = BreadcrumbTrail()
        self.mm = MissionManager(breadcrumb=self.bc)
        self.att = Attitude(0.0, 0.0, 0.0, 0.0)
        self._cache_key, self._cache_list = None, []
        self._leg_i, self._leg_t = 0, 0.0
        self._hold_depth = 2.0
        self._dive_t0 = None
        self._prev_state = None
        self._last_surface_tx = -1e9
        self._cmd = AttitudeCmd()
        self._status = {}
        self._mission_name = ""
        self._pending_reject = ""
        self.fs.dvl_required = False

    # ---------- thread-safe API ----------
    def submit_mission(self, m: dict) -> dict:
        """Validate now (raises ValueError), queue for the control thread. Returns cleaned mission."""
        clean = validate_mission(m)
        with self._lock:
            self._cmds.append(("mission", clean))
        return clean

    def abort(self):
        with self._lock:
            self._cmds.append(("abort", None))

    def snapshot(self) -> dict:
        with self._lock:
            return self._snap

    def log_files(self) -> list:
        if not self.log_dir or not os.path.isdir(self.log_dir):
            return []
        return sorted(f for f in os.listdir(self.log_dir) if f.endswith(".csv"))

    # ---------- loop ----------
    def start(self):
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self.run, name="auv-core", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None

    def run(self):
        self.link.connect()
        self._open_log()
        next_t = self.clock()
        try:
            while not self._stop.is_set():
                now = self.clock()
                try:
                    self.step(now)
                except Exception as ex:   # never die: keep asking the autopilot to surface
                    self.last_error = repr(ex)
                    try:
                        self.link.request_surface()
                    except Exception:
                        pass
                next_t += self.dt
                slack = next_t - self.clock()
                if slack > 0:
                    self._stop.wait(slack)
                else:
                    self.overruns += 1
                    next_t = self.clock()
        finally:
            self.close()

    def close(self):
        if self._logger is not None:
            self._logger.close()
            self._logger = None
        try:
            self.link.close()
        except Exception:
            pass

    def _open_log(self):
        if self.log_dir and self._logger is None:
            name = datetime.now(timezone.utc).strftime("auv_%Y%m%dT%H%M%SZ.csv")
            self._logger = CsvLogger(os.path.join(self.log_dir, name), self.source)

    def _station_keeper(self):
        if not self._sk_tried:
            self._sk_tried = True
            try:
                from .station_keep import StationKeeper   # optional module, lazy
                self._sk = StationKeeper()
            except Exception:
                self._sk = None
        return self._sk

    # ---------- one tick ----------
    def step(self, now: float) -> dict:
        link = self.link
        link.service(now)
        s = link.poll(now)
        self._drain_cmds(now)

        if s.att is not None and s.att.healthy:
            self.att = s.att
        nav = self.nf.update(s)
        mm = self.mm
        st = mm.state
        self.nf.set_surface(st in _SURFACE_STATES or (
            st in (State.ABORT_SURFACE, State.SURFACE) and nav.d < 0.3))
        self.fs.dvl_required = st not in _SURFACE_STATES and st != State.FIX
        if st in (State.DIVE, State.RUN, State.RETURN) and self._dive_t0 is None:
            self._dive_t0 = now
        fail = self.fs.update(s, nav, now - (self._dive_t0 if self._dive_t0 is not None else now))
        out = mm.step(s, nav, fail, self.dt)
        level = fail[0]
        self._mode_transitions(mm.state, now)

        mode = out["mode"]
        att = self.att
        cmd = AttitudeCmd(0.0, 0.0, att.yaw, 0.0)
        status = {}
        if mode in ("track", "return_positional"):
            wl = out["waypoints"]
            key = tuple(id(w) for w in wl)
            if key != self._cache_key:
                self._cache_key, self._cache_list = key, list(wl)
            if self._cache_list:
                cmd, status = self.ctrl.step(nav, att, self._cache_list, self.dt)
                self._hold_depth = self._cache_list[min(self.ctrl.idx,
                                                        len(self._cache_list) - 1)].depth
        elif mode == "return_heading":
            legs = out.get("legs", [])
            if self._leg_i < len(legs):
                hdg, dur = legs[self._leg_i]
                self._leg_t += self.dt
                if self._leg_t > dur:
                    self._leg_i, self._leg_t = self._leg_i + 1, 0.0
                u = max(0.0, (nav.vn - nav.cur_n) * math.cos(att.yaw)
                        + (nav.ve - nav.cur_e) * math.sin(att.yaw))
                pitch = self.depth_hold.step(self._hold_depth, nav, u, self.dt)
                cmd = AttitudeCmd(0.0, pitch, hdg, 0.08)
        elif mode == "surface":
            cmd = AttitudeCmd(0.0, 0.0, att.yaw, 0.0, surface=True)
            if now - self._last_surface_tx >= self.surface_resend_s:
                self._last_surface_tx = now
                link.request_surface()
        elif mode in ("hold", "station_keep"):
            sk = self._station_keeper()
            if sk is not None:
                cmd, status = sk.step(nav, att, out.get("target") or (nav.n, nav.e, 0.0), self.dt)
        if mode != "return_heading":
            self._leg_i, self._leg_t = 0, 0.0
        if mode not in ("idle",) and mm.state not in _IDLE_STATES:
            link.send_attitude(cmd)
        self._cmd = cmd
        self._status = status
        self.tick_count += 1

        row = self._row(now, s, nav, level, cmd, status)
        if self._logger is not None:
            self._logger.write(row)
        self._publish(now, s, nav, level, fail[1], mode, status)
        return row

    def _drain_cmds(self, now):
        with self._lock:
            cmds, self._cmds = list(self._cmds), deque()
        for kind, payload in cmds:
            if kind == "abort":
                self._do_abort()
            elif kind == "mission":
                if self.mm.state in (State.WAIT_GPS, State.ARM_CHECK, State.DONE):
                    if self.mm.state == State.DONE:
                        self.bc = BreadcrumbTrail()
                        self.mm = MissionManager(breadcrumb=self.bc)
                        self.fs.reset()
                        self._dive_t0 = None
                        self.ctrl.reset()
                    self.mm.load(payload)
                    self._mission_name = payload.get("name", "")
                    self._pending_reject = ""
                else:
                    self._pending_reject = f"mission refused in state {self.mm.state.name}"

    def _do_abort(self):
        self.link.request_surface()
        if self.mm.state in (State.DIVE, State.RUN, State.RETURN):
            self.mm._abort("operator abort")
        elif self.mm.state in (State.WAIT_GPS, State.ARM_CHECK):
            self.mm.waypoints = []          # nothing to run; ARM_CHECK needs waypoints

    def _mode_transitions(self, st, now):
        prev, self._prev_state = self._prev_state, st
        if st == prev:
            return
        if st == State.DIVE and prev != State.DIVE and self.auto_arm:
            self.link.set_guided()
            self.link.arm()
        elif st == State.DONE:
            self.link.disarm()
        elif st == State.DIVE and prev == State.SYNC:
            self.link.set_guided()

    # ---------- logging / snapshot ----------
    def _row(self, now, s, nav, level, cmd, status) -> dict:
        r = {c: NAN for c in LOG_COLUMNS}
        r["t_ns"] = int(now * 1e9)
        r["source"] = self.source
        r["f_dvl_valid"] = int(bool(s.dvl is not None and s.dvl.valid))
        r["f_att_healthy"] = int(bool(s.att is not None and s.att.healthy))
        r["f_depth_valid"] = int(bool(s.depth is not None and s.depth.valid))
        r["f_gps_valid"] = int(bool(s.gps is not None and s.gps.valid))
        r["f_nav_healthy"] = int(bool(nav.healthy))
        r["f_leak"] = int(bool(s.leak))
        if s.dvl is not None:
            r.update(dvl_vx=s.dvl.v_body[0], dvl_vy=s.dvl.v_body[1], dvl_vz=s.dvl.v_body[2],
                     dvl_fom=s.dvl.fom, dvl_alt=s.dvl.altitude)
        if s.att is not None:
            r.update(roll=s.att.roll, pitch=s.att.pitch, yaw=s.att.yaw)
        raw = getattr(self.link, "raw", {}) or {}
        if "depth_raw" in raw:
            r["depth_raw"] = raw["depth_raw"]
        if s.depth is not None:
            r["depth"] = s.depth.depth
        r.update(nav_n=nav.n, nav_e=nav.e, nav_d=nav.d, nav_vn=nav.vn, nav_ve=nav.ve,
                 nav_vd=nav.vd, cur_n=nav.cur_n, cur_e=nav.cur_e, dvl_scale=nav.dvl_scale,
                 yaw_bias=nav.yaw_bias, cov_n=nav.cov_pos[0], cov_e=nav.cov_pos[1],
                 dvl_age=nav.dvl_age)
        if s.gps is not None:
            r.update(gps_fix=s.gps.fix_type, gps_hdop=s.gps.hdop, gps_sats=s.gps.sats,
                     gps_n=s.gps.n, gps_e=s.gps.e)
        r["battery_v"] = s.battery_v
        r["gcs_hb_age"] = s.gcs_heartbeat_age
        if status:
            r.update(xte=status.get("cross_track", NAN), along=status.get("along_track", NAN),
                     wp_index=status.get("wp_index", NAN), dist_to_wp=status.get("dist_to_wp", NAN))
        r["mission_state"] = self.mm.state.name
        r["fail_level"] = level.value
        r.update(cmd_roll=cmd.roll, cmd_pitch=cmd.pitch, cmd_yaw=cmd.yaw, cmd_thrust=cmd.thrust)
        return r

    def _publish(self, now, s, nav, level, action, mode, status):
        snap = {
            "running": True, "t": now, "tick": self.tick_count, "overruns": self.overruns,
            "source": self.source, "error": self.last_error,
            "mission": {"name": self._mission_name, "state": self.mm.state.name,
                        "aborted": self.mm.aborted, "abort_reason": self.mm.abort_reason,
                        "wp_index": self.mm.idx, "n_waypoints": len(self.mm.waypoints),
                        "mode": mode, "note": self._pending_reject},
            "failsafe": {"level": level.name, "action": action, "reasons": list(self.fs.reasons)},
            "nav": {"n": nav.n, "e": nav.e, "d": nav.d, "vn": nav.vn, "ve": nav.ve,
                    "cur_n": nav.cur_n, "cur_e": nav.cur_e, "healthy": nav.healthy,
                    "dvl_age": nav.dvl_age},
            "att": {"roll": self.att.roll, "pitch": self.att.pitch, "yaw": self.att.yaw},
            "battery_v": s.battery_v, "leak": s.leak, "gcs_hb_age": s.gcs_heartbeat_age,
            "cmd": {"pitch": self._cmd.pitch, "yaw": self._cmd.yaw, "thrust": self._cmd.thrust},
            "cross_track": status.get("cross_track", NAN) if status else NAN,
        }
        snap = _jsonable(snap)
        with self._lock:
            self._snap = snap
