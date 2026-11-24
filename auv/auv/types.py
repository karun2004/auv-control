"""Shared interface contract. Pure data, no hardware imports.

Frames: local NED metres from the launch GPS fix. x=North, y=East, z=Down.
Angles in radians inside the library. Body frame: x forward, y starboard, z down.
Time: float seconds from one monotonic clock.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
import math


@dataclass
class DvlSample:
    t: float
    v_body: tuple          # (vx, vy, vz) m/s body frame
    fom: float             # figure of merit; math.nan when absent
    altitude: float        # m; math.nan when absent
    valid: bool            # bottom lock + beams ok. Never a zero standing in for missing.


@dataclass
class Attitude:
    t: float
    roll: float
    pitch: float
    yaw: float             # rad, 0=north, +clockwise
    healthy: bool = True


@dataclass
class DepthSample:
    t: float
    depth: float           # m, positive down, after surface re-zero
    valid: bool = True


@dataclass
class GpsSample:
    t: float
    n: float               # local north m
    e: float               # local east m
    fix_type: int
    hdop: float
    sats: int
    valid: bool            # only meaningful when surfaced


@dataclass
class Sensors:
    """One control tick of inputs. Any field may be None if no new sample."""
    t: float
    dvl: Optional[DvlSample] = None
    att: Optional[Attitude] = None
    depth: Optional[DepthSample] = None
    gps: Optional[GpsSample] = None
    battery_v: float = 16.0
    leak: bool = False
    gcs_heartbeat_age: float = 0.0


@dataclass
class NavState:
    t: float
    n: float
    e: float
    d: float               # depth m
    vn: float
    ve: float
    vd: float
    cur_n: float           # estimated water current
    cur_e: float
    dvl_scale: float       # estimated DVL scale factor (nominal 1.0)
    yaw_bias: float        # rad
    cov_pos: tuple         # (var_n, var_e)
    healthy: bool = True
    dvl_age: float = 0.0   # s since last valid DVL


@dataclass
class Waypoint:
    n: float
    e: float
    depth: float
    timeout_s: float = 600.0


@dataclass
class AttitudeCmd:
    """Output of the Pi to the Pixhawk (SET_ATTITUDE_TARGET + thrust)."""
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0       # absolute heading rad
    thrust: float = 0.0    # 0..1 forward
    surface: bool = False  # abort flag, informational


class FailLevel(Enum):
    H0 = 0   # healthy
    H1 = 1   # DVL gap >1s: dead-reckon, inflate cov
    H2 = 2   # DVL dead >5s: abort + return
    H3 = 3   # attitude bad >0.5s: blind surface
    H4 = 4   # depth timeout >1s
    H5 = 5   # Pi dead (fin deadman, Pixhawk side)
    H6 = 6   # no GCS heartbeat 5s
    H7 = 7   # hard limits: depth, pitch, leak, battery, runtime
    H7B = 8  # current >0.3 m/s for >10s
    H8 = 9   # everything dead: buoyancy


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# Plan constants (docs section 6-9). Placeholders marked.
CRUISE_SPEED = 0.514        # m/s
CAPTURE_RADIUS = 8.0        # m, placeholder until turn radius measured
MAX_DEPTH = 12.0            # m
CURRENT_ABORT = 0.3         # m/s
BATT_LOW_V = 13.2
BATT_CRIT_V = 12.8
