"""MAVLink I/O between the Pi outer loops and the Pixhawk (ArduSub).

Link          real implementation over pymavlink (imported lazily, so the library and tests
              import without it).
FakeLink      same interface for tests: scripted Sensors in, captured commands out.

Interface used by auv.core.AuvCore (both classes):
    connect(), close()
    service(now)             send the 1 Hz GCS heartbeat when due (never blocks)
    poll(now) -> Sensors     drain pending messages without blocking; fields are None when
                             no new sample arrived this call
    set_guided(), arm(), disarm(), request_surface()
    send_attitude(cmd: AttitudeCmd)

UNVERIFIED ASSUMPTIONS (check against the real vehicle / BlueOS before trusting):
 A1. ArduSub 4.7.2 GUIDED mode honours SET_ATTITUDE_TARGET with the quaternion (roll, pitch,
     yaw) and takes `thrust` in 0..1. How ArduSub maps `thrust` is NOT verified: in the
     ArduPilot Sub source it is, to my recollection, interpreted around 0.5 as a vertical
     (climb) command, not as forward thrust. The project runs a Lua fin mixer on the Pixhawk,
     so forward thrust handling may be custom. `thrust_mode` selects the mapping:
       "direct"   -> wire thrust = clamp(cmd.thrust, 0, 1)            (default)
       "centered" -> wire thrust = 0.5 + 0.5 * clamp(cmd.thrust, 0, 1) (0.5 = neutral)
     Bench-test with props off before use.
 A2. Mode numbers: ArduSub GUIDED=4, SURFACE=9 (custom_mode via MAV_CMD_DO_SET_MODE).
 A3. DVL arrives as VISION_SPEED_ESTIMATE (x,y,z m/s) in the BODY frame (x fwd, y stbd,
     z down) and altitude as a downward DISTANCE_SENSOR (orientation 25). Frame and message
     choice depend on the DVL driver; unverified.
 A4. VISION_SPEED_ESTIMATE has no figure of merit. fom is derived from sqrt(covariance[0])
     when finite, otherwise `dvl_fom_default` (config; NaN would make NavFilter reject every
     sample). Raw covariance is not logged separately.
 A5. Depth from SCALED_PRESSURE2.press_abs (hPa), zeroed on the first sample or by
     zero_depth(); fluid density configurable (default fresh water, 1000 kg/m3).
 A6. gcs_heartbeat_age = age of the latest HEARTBEAT with type MAV_TYPE_GCS (6). If no
     ground station is connected the age grows and MissionManager will refuse to arm.
 A7. Leak = a STATUSTEXT containing "leak" (case-insensitive); latched until clear_leak().
 A8. Battery unknown (no SYS_STATUS/BATTERY_STATUS yet) is reported as NaN, which blocks the
     mission arm check rather than pretending the pack is full.
"""
from __future__ import annotations
import math
import time
from typing import Optional

from .types import (Sensors, DvlSample, Attitude, DepthSample, GpsSample, AttitudeCmd)

# ArduSub custom modes (A2)
SUB_MODE_GUIDED = 4
SUB_MODE_SURFACE = 9

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128
FORCE_DISARM_MAGIC = 21196
DISTANCE_SENSOR_DOWN = 25          # MAV_SENSOR_ROTATION_PITCH_270
# SET_ATTITUDE_TARGET type_mask: ignore body roll/pitch/yaw rates (bits 0..2)
ATT_TARGET_IGNORE_RATES = 0b00000111
EARTH_R = 6378137.0
G = 9.80665


def euler_to_quat(roll: float, pitch: float, yaw: float) -> tuple:
    """Aerospace ZYX (yaw-pitch-roll) -> quaternion (w, x, y, z), NED to body FRD.

    +pitch = nose up and +yaw = clockwise from north, matching MAVLink and auv.types.
    """
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def thrust_to_wire(thrust: float, mode: str = "direct") -> float:
    t = min(1.0, max(0.0, thrust if math.isfinite(thrust) else 0.0))
    if mode == "centered":
        return 0.5 + 0.5 * t
    if mode == "direct":
        return t
    raise ValueError("thrust_mode must be 'direct' or 'centered'")


def _ned(lat, lon, lat0, lon0):
    n = math.radians(lat - lat0) * EARTH_R
    e = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    return n, e


class Link:
    """pymavlink-backed link. All reads are non-blocking; nothing here sleeps."""

    def __init__(self, endpoint: str = "udpin:0.0.0.0:14550", *, source_system: int = 250,
                 source_component: int = 191, target_system: int = 1,
                 target_component: int = 1, thrust_mode: str = "direct",
                 heartbeat_period: float = 1.0, water_density: float = 1000.0,
                 dvl_fom_default: float = 0.1, origin: Optional[tuple] = None,
                 max_msgs_per_poll: int = 200, conn=None):
        thrust_to_wire(0.0, thrust_mode)       # validate
        self.endpoint = endpoint
        self.source_system, self.source_component = source_system, source_component
        self.target_system, self.target_component = target_system, target_component
        self.thrust_mode = thrust_mode
        self.heartbeat_period = heartbeat_period
        self.rho = water_density
        self.dvl_fom_default = dvl_fom_default
        self.origin = origin                   # (lat, lon) of local NED origin; set on first fix
        self.max_msgs = max_msgs_per_poll
        self.conn = conn                       # injectable (tests)
        self._t_connect = None
        self._last_hb = None
        self._t_gcs = None
        self.armed = False
        self.mode = None                       # last autopilot custom_mode seen
        self.rx_count = 0
        self.err_count = 0
        self.p_surface_hpa = None
        self._alt = (math.nan, -1e9)           # (m, t)
        self._batt = math.nan
        self._last_press = None
        self._leak = False
        self.raw = {}                          # raw values of the last poll, for the logger

    # ---- connection ----
    def connect(self, now: Optional[float] = None):
        if self.conn is None:
            from pymavlink import mavutil
            self.conn = mavutil.mavlink_connection(
                self.endpoint, source_system=self.source_system,
                source_component=self.source_component, autoreconnect=True)
        self._t_connect = time.monotonic() if now is None else now

    def close(self):
        if self.conn is not None and hasattr(self.conn, "close"):
            self.conn.close()

    # ---- tx ----
    def service(self, now: float):
        if self._last_hb is None or now - self._last_hb >= self.heartbeat_period:
            self._last_hb = now
            # type GCS, autopilot INVALID, base_mode 0, custom 0, status ACTIVE(4)
            self.conn.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 4)

    def _set_mode(self, custom_mode: int):
        self.conn.mav.command_long_send(
            self.target_system, self.target_component, MAV_CMD_DO_SET_MODE, 0,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode, 0, 0, 0, 0, 0)

    def set_guided(self):
        self._set_mode(SUB_MODE_GUIDED)

    def request_surface(self):
        self._set_mode(SUB_MODE_SURFACE)

    def arm(self):
        self.conn.mav.command_long_send(self.target_system, self.target_component,
                                        MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    def disarm(self, force: bool = False):
        self.conn.mav.command_long_send(self.target_system, self.target_component,
                                        MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,
                                        FORCE_DISARM_MAGIC if force else 0, 0, 0, 0, 0, 0)

    def send_attitude(self, cmd: AttitudeCmd, now: Optional[float] = None):
        q = euler_to_quat(cmd.roll, cmd.pitch, cmd.yaw)
        boot_ms = int(((time.monotonic() if now is None else now) * 1000)) & 0xFFFFFFFF
        self.conn.mav.set_attitude_target_send(
            boot_ms, self.target_system, self.target_component, ATT_TARGET_IGNORE_RATES,
            list(q), 0.0, 0.0, 0.0, thrust_to_wire(cmd.thrust, self.thrust_mode))

    # ---- rx ----
    def zero_depth(self):
        """Re-zero depth at the surface using the latest pressure."""
        if self._last_press is not None:
            self.p_surface_hpa = self._last_press

    def clear_leak(self):
        self._leak = False

    def poll(self, now: float) -> Sensors:
        s = Sensors(t=now, battery_v=self._batt, leak=self._leak)
        self.raw = {}
        for _ in range(self.max_msgs):
            try:
                m = self.conn.recv_match(blocking=False)
            except Exception:
                self.err_count += 1
                break
            if m is None:
                break
            self.rx_count += 1
            try:
                self._handle(m, now, s)
            except Exception:
                self.err_count += 1
        s.battery_v, s.leak = self._batt, self._leak
        ref = (self._t_gcs if self._t_gcs is not None
               else (self._t_connect if self._t_connect is not None else now))
        s.gcs_heartbeat_age = max(0.0, now - ref)
        return s

    def _handle(self, m, now: float, s: Sensors):
        ty = m.get_type()
        if ty == "HEARTBEAT":
            src = m.get_srcSystem()
            if src == self.source_system:
                return
            if m.type == MAV_TYPE_GCS:
                self._t_gcs = now
            elif m.autopilot != MAV_AUTOPILOT_INVALID:
                self.target_system, self.target_component = src, m.get_srcComponent()
                self.armed = bool(m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = m.custom_mode
        elif ty == "ATTITUDE":
            s.att = Attitude(now, m.roll, m.pitch, m.yaw, healthy=True)
        elif ty == "SCALED_PRESSURE2":
            p = float(m.press_abs)
            self._last_press = p
            if self.p_surface_hpa is None:
                self.p_surface_hpa = p
            depth = (p - self.p_surface_hpa) * 100.0 / (self.rho * G)
            self.raw["press_hpa"] = p
            self.raw["depth_raw"] = (p * 100.0) / (self.rho * G)    # absolute-pressure depth
            s.depth = DepthSample(now, depth, valid=math.isfinite(depth))
        elif ty == "DISTANCE_SENSOR":
            if getattr(m, "orientation", DISTANCE_SENSOR_DOWN) == DISTANCE_SENSOR_DOWN:
                self._alt = (m.current_distance / 100.0, now)
        elif ty == "VISION_SPEED_ESTIMATE":
            v = (float(m.x), float(m.y), float(m.z))
            cov = getattr(m, "covariance", None)
            var = cov[0] if cov else math.nan
            fom = math.sqrt(var) if math.isfinite(var) and var >= 0 else self.dvl_fom_default
            alt = self._alt[0] if now - self._alt[1] < 1.0 else math.nan
            s.dvl = DvlSample(now, v, fom, alt, valid=all(math.isfinite(c) for c in v))
        elif ty == "GPS_RAW_INT":
            fix = int(m.fix_type)
            lat, lon = m.lat / 1e7, m.lon / 1e7
            if self.origin is None and fix >= 3:
                self.origin = (lat, lon)
            if self.origin is not None and fix >= 2:
                n, e = _ned(lat, lon, *self.origin)
            else:
                n = e = math.nan
            hdop = m.eph / 100.0 if m.eph != 0xFFFF else math.nan
            sats = m.satellites_visible if m.satellites_visible != 255 else 0
            s.gps = GpsSample(now, n, e, fix, hdop, sats,
                              valid=fix >= 3 and math.isfinite(n) and math.isfinite(hdop))
        elif ty == "SYS_STATUS":
            if m.voltage_battery != 0xFFFF:
                self._batt = m.voltage_battery / 1000.0
        elif ty == "BATTERY_STATUS":
            cells = [v for v in m.voltages if v != 0xFFFF]
            if cells:
                self._batt = sum(cells) / 1000.0
        elif ty == "STATUSTEXT":
            txt = m.text if isinstance(m.text, str) else bytes(m.text).decode("ascii", "ignore")
            if "leak" in txt.lower():
                self._leak = True


class FakeLink:
    """Test double. push() Sensors for the next poll(); commands land in `sent`."""

    def __init__(self):
        self.queue = []
        self.sent = []            # (kind, payload) in order
        self.connected = False
        self.heartbeats = []      # times service() actually sent one
        self._last_hb = None
        self.raw = {}
        self.origin = None

    def push(self, s: Sensors, raw: Optional[dict] = None):
        self.queue.append((s, raw or {}))

    def connect(self, now=None):
        self.connected = True

    def close(self):
        self.connected = False

    def service(self, now: float):
        if self._last_hb is None or now - self._last_hb >= 1.0:
            self._last_hb = now
            self.heartbeats.append(now)
            self.sent.append(("heartbeat", now))

    def poll(self, now: float) -> Sensors:
        if self.queue:
            s, self.raw = self.queue.pop(0)
            return s
        self.raw = {}
        return Sensors(t=now, battery_v=math.nan)

    def set_guided(self):
        self.sent.append(("guided", None))

    def request_surface(self):
        self.sent.append(("surface", None))

    def arm(self):
        self.sent.append(("arm", None))

    def disarm(self, force: bool = False):
        self.sent.append(("disarm", force))

    def send_attitude(self, cmd: AttitudeCmd, now=None):
        self.sent.append(("attitude", cmd))

    def kinds(self):
        return [k for k, _ in self.sent]
