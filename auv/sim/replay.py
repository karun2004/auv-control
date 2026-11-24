"""Replay field logs as Sensors streams.

Schemas handled
  tether : CORATIA_Tether_Log_*.csv  (Timestamp HH:MM:SS, Local_X_m, Local_Y_m, Pressure_Depth_m,
           [DVL_Altitude_m], Linear_Vx_ms, Linear_Vy_ms, Gyro_Heading_Yaw deg, Satellites_Count, DVL_FOM)
  rov    : ROV_Mission_*.csv with X,Y,Z,Vx,Vy,Vz,Yaw,Pitch,Roll,[Depth,Altitude,Sensor State,..,FOM]
  rov_min: ROV_Mission_Data_*.csv, only Timestamp,X,Y,Z,Vx,Vy,Vz,Yaw,Pitch,Roll (ISO timestamps)

Time: timestamps have 1 s resolution (tether/rov) so rows sharing a timestamp are spread evenly
across that second (t = s + i/k).  Time is seconds since the first row, non-decreasing, midnight
wrap handled.  ISO timestamps (with ms) get the same grouping treatment (identical strings spread).

Frames: Vx,Vy in the logs are LOCAL-frame (north,east) velocities.  They are rotated into the body
frame with the logged yaw (v_body = Rz(yaw)^T [vn,ve]) so NavFilter's body->NED rotation round-trips
exactly.  Yaw logged in degrees -> radians wrapped to (-pi,pi].

DVL validity (deliberately conservative: when in doubt, valid=False):
  * FOM >= 50 (the 99.9 sentinel) or NaN/unparseable            -> invalid
  * tether FOM == 0.0 exactly AND Vx==Vy==0 exactly              -> invalid ('missing', not 'perfect')
    (FOM is printed with 4 decimals so 0.0000 with a non-zero velocity is a real, tiny FOM -> valid)
  * rov 'Sensor State' label 'Coasting' (state 2) or altitude==0.0 with FOM>=50 -> invalid
  * rov_min (no FOM column at all)                               -> always invalid (fom NaN)
  * invalid samples carry v_body=(0,0,0), fom=NaN, altitude=NaN.
GPS is always None (logged satellite counts are unreliable underwater / not true fixes).
Depth is the logged pressure depth (tether) / Depth column (rov) / Z (rov_min), valid=True.
"""
from __future__ import annotations
import csv
import math
import os
from dataclasses import dataclass, field
from typing import Optional
import numpy as np
from auv.types import Sensors, DvlSample, Attitude, DepthSample, wrap_pi

LOG_DIR = "/Users/karun/Documents/coratia/12_AUV_Sub_PIXHAWK/logs"


def _f(x, default=math.nan):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def _sod(ts: str) -> float:
    """Seconds-of-day from 'HH:MM:SS' or ISO 'YYYY-MM-DDTHH:MM:SS.mmmZ'."""
    s = ts.strip()
    if "T" in s:
        s = s.split("T", 1)[1]
    s = s.rstrip("Z")
    parts = s.split(":")
    return int(parts[0]) * 3600 + int(parts[1]) * 60 + float(parts[2])


def reconstruct_time(stamps: list) -> np.ndarray:
    """Monotonic time (s, from 0) with rows of equal timestamp spread evenly over that second."""
    n = len(stamps)
    sod = np.empty(n)
    day = 0.0
    prev = None
    for i, ts in enumerate(stamps):
        v = _sod(ts)
        if prev is not None and v < prev - 12 * 3600:
            day += 86400.0
        prev = v
        sod[i] = v + day
    t = sod.copy()
    i = 0
    while i < n:
        j = i
        while j < n and stamps[j] == stamps[i]:
            j += 1
        k = j - i
        if k > 1:
            span = 1.0 if "T" not in stamps[i] else 0.0   # ISO ms stamps: identical string is rare
            t[i:j] = sod[i] + span * np.arange(k) / k
        i = j
    t = np.maximum.accumulate(t)
    return t - t[0]


@dataclass
class Replay:
    path: str
    schema: str
    t: np.ndarray
    sensors: list                 # list[Sensors], one per row
    local_xy: np.ndarray          # logged (X,Y) position, for comparison with NavFilter output (n,e)
    yaw: np.ndarray               # rad
    depth: np.ndarray
    vel_local: np.ndarray         # (n,2) logged local-frame velocity
    fom: np.ndarray
    dvl_valid: np.ndarray
    columns: list = field(default_factory=list)

    def __iter__(self):
        return iter(self.sensors)

    def __len__(self):
        return len(self.sensors)


def detect_schema(cols: list) -> str:
    if "Local_X_m" in cols:
        return "tether"
    if "FOM" in cols or "Depth (m)" in cols:
        return "rov"
    return "rov_min"


def _rows(path):
    with open(path, newline="", encoding="utf-8-sig") as fh:
        rd = csv.DictReader(fh)
        cols = [c.strip() for c in rd.fieldnames]
        rows = [{(k or "").strip(): v for k, v in r.items()} for r in rd if r.get("Timestamp")]
    return cols, rows


def load_replay(path: str) -> Replay:
    cols, rows = _rows(path)
    schema = detect_schema(cols)
    n = len(rows)
    if n == 0:
        raise ValueError(f"no data rows in {path}")
    t = reconstruct_time([r["Timestamp"].strip() for r in rows])
    sensors, xy, yaws, deps, vels, foms, valids = [], [], [], [], [], [], []
    for i, r in enumerate(rows):
        if schema == "tether":
            x, y = _f(r["Local_X_m"]), _f(r["Local_Y_m"])
            depth = _f(r["Pressure_Depth_m"])
            vn, ve, vz = _f(r["Linear_Vx_ms"]), _f(r["Linear_Vy_ms"]), 0.0
            yaw = math.radians(_f(r["Gyro_Heading_Yaw"], 0.0))
            pitch = roll = 0.0
            fom = _f(r.get("DVL_FOM"))
            alt = _f(r.get("DVL_Altitude_m"))
            exact_zero = (vn == 0.0 and ve == 0.0)
            valid = math.isfinite(fom) and fom < 50.0 and not (fom == 0.0 and exact_zero)
        elif schema == "rov":
            x, y = _f(r["X (m)"]), _f(r["Y (m)"])
            depth = _f(r.get("Depth (m)"), _f(r.get("Z (m)")))
            vn, ve, vz = _f(r["Vx (m/s)"]), _f(r["Vy (m/s)"]), _f(r["Vz (m/s)"])
            yaw = math.radians(_f(r["Yaw (°)"], 0.0))
            pitch = math.radians(_f(r["Pitch (°)"], 0.0))
            roll = math.radians(_f(r["Roll (°)"], 0.0))
            fom = _f(r.get("FOM"))
            alt = _f(r.get("Altitude (m)"))
            label = (r.get("State Label") or "").strip().lower()
            valid = (math.isfinite(fom) and fom < 50.0 and label != "coasting"
                     and not (fom == 0.0 and vn == 0.0 and ve == 0.0))
        else:
            x, y = _f(r["X"]), _f(r["Y"])
            depth = _f(r["Z"])
            vn, ve, vz = _f(r["Vx"]), _f(r["Vy"]), _f(r["Vz"])
            yaw = math.radians(_f(r["Yaw"], 0.0))
            pitch = math.radians(_f(r["Pitch"], 0.0))
            roll = math.radians(_f(r["Roll"], 0.0))
            fom = alt = math.nan
            valid = False
        yaw = wrap_pi(yaw)
        if valid and not all(map(math.isfinite, (vn, ve, vz))):
            valid = False
        ti = float(t[i])
        if valid:
            # logged local velocity -> body: v_b = Rz(yaw)^T [vn, ve] (planar; pitch/roll ignored
            # for tether, applied for rov via the full rotation so NavFilter round-trips).
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                          [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                          [-sp, cp * sr, cp * cr]])
            vb = tuple(float(v) for v in R.T @ np.array([vn, ve, vz]))
            dvl = DvlSample(ti, vb, fom, alt if (math.isfinite(alt) and alt > 0) else math.nan, True)
        else:
            dvl = DvlSample(ti, (0.0, 0.0, 0.0), math.nan, math.nan, False)
        s = Sensors(t=ti, dvl=dvl, att=Attitude(ti, roll, pitch, yaw, True),
                    depth=DepthSample(ti, depth, math.isfinite(depth)), gps=None)
        sensors.append(s)
        xy.append((x, y)); yaws.append(yaw); deps.append(depth)
        vels.append((vn, ve)); foms.append(fom); valids.append(valid)
    return Replay(path, schema, t, sensors, np.array(xy), np.array(yaws), np.array(deps),
                  np.array(vels), np.array(foms), np.array(valids), cols)


def list_logs(log_dir: str = LOG_DIR) -> list:
    return sorted(os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".csv"))


def replay_stats(path: str) -> dict:
    """Summary: rows, duration, depth range, depth quantisation step, distinct depth values."""
    rp = load_replay(path)
    d = rp.depth[np.isfinite(rp.depth)]
    vals = np.unique(np.round(d, 6))
    diffs = np.diff(vals)
    diffs = diffs[diffs > 1e-9]
    step = float(np.min(diffs)) if diffs.size else math.nan
    return {
        "file": os.path.basename(path), "schema": rp.schema, "rows": len(rp),
        "duration_s": float(rp.t[-1]), "depth_min": float(d.min()) if d.size else math.nan,
        "depth_max": float(d.max()) if d.size else math.nan,
        "depth_quant_step": step, "distinct_depth_values": int(vals.size),
        "dvl_valid_frac": float(np.mean(rp.dvl_valid)),
    }


if __name__ == "__main__":
    for p in list_logs():
        print(replay_stats(p))
