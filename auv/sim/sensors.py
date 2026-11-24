"""Sensor models: Truth (from sim.plant) -> auv.types.Sensors, with a fault-injection API.

Fault windows are in sim time (seconds).  Failure modes:
  * DVL: valid=False, fom=NaN, altitude=NaN, v_body=(0,0,0) (the zero is NOT to be trusted;
    valid is the flag).  Also lost automatically when altitude >40 m or <0.05 m.
  * attitude: mode "unhealthy" (default) emits Attitude(healthy=False, garbage); "none" emits nothing.
  * depth: mode "none" (default, timeout style) emits nothing; "invalid" emits valid=False.
"""
from __future__ import annotations
import math
import random
from dataclasses import dataclass
from typing import Optional
import numpy as np
from auv.types import Sensors, DvlSample, Attitude, DepthSample, GpsSample, wrap_pi
from sim.plant import Truth, Plant


@dataclass
class SensorParams:
    dvl_hz: float = 10.0
    att_hz: float = 50.0
    depth_hz: float = 10.0
    gps_hz: float = 1.0
    dvl_noise: float = 0.004          # m/s 1-sigma per axis
    dvl_scale_err: float = 0.012      # reported/true - 1.  +1.2% fresh (c=1482 vs 1500 assumed),
                                      # -1.4% salt (c=1521); set via set_water()
    depth_scale: float = 1.0          # reported/true depth: true_density/(1000*BARO_SPEC_GRAV)
    dvl_dropout_rate: float = 0.002   # per-sample probability a dropout burst starts
    dvl_dropout_len: tuple = (1, 8)   # samples
    dvl_alt_max: float = 40.0
    dvl_alt_min: float = 0.05
    bottom_depth: float = 15.0        # m, flat bottom
    compass_bias: float = math.radians(0.0)
    compass_noise: float = math.radians(0.5)
    tilt_noise: float = math.radians(0.1)
    depth_quant: float = 0.03
    depth_noise: float = 0.01
    gps_depth_max: float = 0.3
    gps_noise: float = 2.5
    gps_spike_prob: float = 0.02
    gps_spike_m: float = 100.0
    batt_v0: float = 16.4
    batt_drain_v_per_s: float = 0.0002


class SensorSuite:
    def __init__(self, params: Optional[SensorParams] = None, seed: int = 1,
                 plant: Optional[Plant] = None):
        self.p = params or SensorParams()
        self.rng = random.Random(seed)
        self.plant = plant
        self._next = {"dvl": 0.0, "att": 0.0, "depth": 0.0, "gps": 0.0}
        self._dvl_drop_left = 0
        self._dvl_kill: list = []
        self._att_kill: list = []
        self._depth_kill: list = []
        self._hb_drop: list = []
        self._leak_t: Optional[float] = None
        self._batt_override: Optional[tuple] = None   # (t0, volts)
        self._last_hb = 0.0
        self._depth_zero = 0.0

    # ---------------- fault injection ----------------
    def kill_dvl(self, t0: float, t1: float = math.inf): self._dvl_kill.append((t0, t1))
    def kill_attitude(self, t0: float, t1: float = math.inf, mode: str = "unhealthy"):
        self._att_kill.append((t0, t1, mode))
    def kill_depth(self, t0: float, t1: float = math.inf, mode: str = "none"):
        self._depth_kill.append((t0, t1, mode))
    def drop_heartbeat(self, t0: float, t1: float = math.inf): self._hb_drop.append((t0, t1))
    def inject_leak(self, t0: float): self._leak_t = t0
    def set_battery(self, volts: float, t0: float = 0.0): self._batt_override = (t0, volts)
    def set_compass_bias_deg(self, deg: float): self.p.compass_bias = math.radians(deg)
    def set_dvl_scale_error(self, frac: float): self.p.dvl_scale_err = frac
    def set_water(self, true_water, dvl_sound_speed: float = 1500.0,
                  baro_spec_grav: Optional[float] = None):
        """Set the true water and the autopilot/DVL settings.  Defaults model an un-configured
        vehicle (DVL at 1500 m/s) with BARO_SPEC_GRAV correct for the water; pass
        baro_spec_grav=1.0 in salt (or 1.024 in fresh) to model the wrong setting."""
        from auv import water as w
        tw = w.get_water(true_water)
        self.p.dvl_scale_err = w.dvl_velocity_scale(tw, dvl_sound_speed) - 1.0
        sg = tw.baro_spec_grav if baro_spec_grav is None else baro_spec_grav
        self.p.depth_scale = w.depth_reading_scale(tw, sg)
    def set_bottom_depth(self, m: float): self.p.bottom_depth = m
    def set_current(self, n: float, e: float):
        if self.plant is None:
            raise RuntimeError("SensorSuite has no plant attached")
        self.plant.set_current(n, e)

    # ---------------- sampling ----------------
    @staticmethod
    def _in(windows, t):
        for w in windows:
            if w[0] <= t < w[1]:
                return w
        return None

    def _due(self, key: str, hz: float, t: float) -> bool:
        if t + 1e-9 >= self._next[key]:
            self._next[key] = max(self._next[key] + 1.0 / hz, t - 0.5 / hz) if hz > 0 else math.inf
            if self._next[key] <= t:
                self._next[key] = t + 1.0 / hz
            return True
        return False

    def sample(self, tr: Truth, t: Optional[float] = None) -> Sensors:
        p, g = self.p, self.rng.gauss
        t = tr.t if t is None else t
        out = Sensors(t=t)

        # --- DVL ---
        if self._due("dvl", p.dvl_hz, t):
            alt = p.bottom_depth - tr.d
            lost = (self._in(self._dvl_kill, t) is not None or alt > p.dvl_alt_max
                    or alt < p.dvl_alt_min)
            if self._dvl_drop_left > 0:
                self._dvl_drop_left -= 1
                lost = True
            elif self.rng.random() < p.dvl_dropout_rate:
                self._dvl_drop_left = self.rng.randint(*p.dvl_dropout_len) - 1
                lost = True
            if lost:
                out.dvl = DvlSample(t, (0.0, 0.0, 0.0), math.nan, math.nan, False)
            else:
                cr, sr = math.cos(tr.roll), math.sin(tr.roll)
                cp, sp = math.cos(tr.pitch), math.sin(tr.pitch)
                cy, sy = math.cos(tr.yaw), math.sin(tr.yaw)
                R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                              [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                              [-sp, cp * sr, cp * cr]])
                vb = R.T @ np.array([tr.vn, tr.ve, tr.vd]) * (1.0 + p.dvl_scale_err)
                vb = tuple(float(v + g(0, p.dvl_noise)) for v in vb)
                fom = abs(0.0003 + g(0, 0.0001))
                out.dvl = DvlSample(t, vb, fom, alt + g(0, 0.02), True)

        # --- attitude ---
        if self._due("att", p.att_hz, t):
            w = self._in(self._att_kill, t)
            if w is None:
                out.att = Attitude(t, tr.roll + g(0, p.tilt_noise), tr.pitch + g(0, p.tilt_noise),
                                   wrap_pi(tr.yaw + p.compass_bias + g(0, p.compass_noise)), True)
            elif w[2] == "unhealthy":
                out.att = Attitude(t, 0.0, 0.0, 0.0, False)

        # --- depth ---
        if self._due("depth", p.depth_hz, t):
            w = self._in(self._depth_kill, t)
            if w is None:
                dd = tr.d * p.depth_scale + g(0, p.depth_noise)
                q = p.depth_quant
                out.depth = DepthSample(t, round(dd / q) * q, True)
            elif w[2] == "invalid":
                out.depth = DepthSample(t, math.nan, False)

        # --- GPS (surface only) ---
        if self._due("gps", p.gps_hz, t) and tr.d < p.gps_depth_max:
            n = tr.n + g(0, p.gps_noise)
            e = tr.e + g(0, p.gps_noise)
            if self.rng.random() < p.gps_spike_prob:
                a = self.rng.uniform(0, 2 * math.pi)
                n += p.gps_spike_m * math.cos(a)
                e += p.gps_spike_m * math.sin(a)
            out.gps = GpsSample(t, n, e, 3, 0.9, 10, True)

        # --- slow signals ---
        if self._batt_override is not None and t >= self._batt_override[0]:
            out.battery_v = self._batt_override[1]
        else:
            out.battery_v = p.batt_v0 - p.batt_drain_v_per_s * t
        out.leak = self._leak_t is not None and t >= self._leak_t
        if self._in(self._hb_drop, t) is None:
            self._last_hb = t
        out.gcs_heartbeat_age = t - self._last_hb
        return out
