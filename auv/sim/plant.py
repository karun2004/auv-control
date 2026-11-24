"""Finned torpedo AUV plant (6-DOF-lite) plus emulated Pixhawk inner attitude loop.

Conventions (match auv/types.py): NED metres, yaw 0=north +clockwise, depth positive down.
Pitch positive = nose UP (so climbing: dD/dt = -u*sin(pitch)).  Positive elevator -> pitch up,
positive rudder -> yaw clockwise.  Roll is held at 0.

Net weight: `net_weight_N` > 0 sinks, < 0 floats (= -net buoyancy, see auv/water.py).  Legacy
presets: fresh -2 N, salt +3 N.  Water type (density) scales the hydrodynamic coefficients
(drag, heave drag, fin authority) by density/1000; use `set_water(kind, ballast_kg)` for buoyancy.  Fin authority is proportional to u^2 (u = speed through water).
No sideways motion: ground velocity = u * heading + water current.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import math
import random
from typing import Optional
from auv.types import AttitudeCmd, wrap_pi
from auv import water as _w

FRESH_NET_WEIGHT_N = -2.0
SALT_NET_WEIGHT_N = 3.0


@dataclass
class PlantParams:
    mass: float = 25.0
    added_mass: float = 5.0
    drag_k: float = 1.1 / 0.514 ** 2     # N/(m/s)^2 -> 1.1 N at 0.514 m/s
    thrust_max: float = 20.0             # N at cmd 1.0
    tau_thrust: float = 0.3
    net_weight_N: float = FRESH_NET_WEIGHT_N
    heave_drag_k: float = 100.0          # N/(m/s)^2 vertical (fresh water; guess, see docs/TRIM.md)
    water: object = _w.FRESH             # auv.water.WaterType the plant is in
    hull_mass_kg: float = 19.92          # buoyancy mass (not dynamic mass) for set_water
    hull_volume_L: float = 20.0
    heave_mass: float = 40.0
    k_rudder: float = 1.2                # yaw rate (rad/s) per rad of fin per (m/s)^2
    k_elev: float = 1.2
    k_pitch_stab: float = 0.15           # 1/s restoring
    tau_r: float = 0.5
    tau_q: float = 0.5
    fin_max: float = 0.35                # rad
    fin_rate: float = 3.0                # rad/s servo rate limit
    pitch_max: float = 1.0
    deadman_s: float = 0.25
    deadman_zero_thrust: bool = False
    # inner loop gains (Pixhawk emulation), fixed (no speed scheduling)
    yaw_kp: float = 2.0
    yaw_kd: float = 1.0
    yaw_ki: float = 0.05
    pitch_kp: float = 2.0
    pitch_kd: float = 1.0
    pitch_ki: float = 0.05
    i_limit: float = 0.15
    dt_internal: float = 0.02


@dataclass
class Truth:
    t: float
    n: float
    e: float
    d: float
    roll: float
    pitch: float
    yaw: float
    u: float            # speed through water, m/s
    vn: float           # ground velocity
    ve: float
    vd: float
    cur_n: float
    cur_e: float
    fin_r: float
    fin_e: float
    thrust: float       # actual thrust N
    deadman_active: bool


class Plant:
    def __init__(self, params: Optional[PlantParams] = None, n0=0.0, e0=0.0, d0=0.0,
                 yaw0=0.0, u0=0.0, cur=(0.0, 0.0), seed=0):
        self.p = params or PlantParams()
        self.t = 0.0
        self.n, self.e, self.d = n0, e0, d0
        self.yaw, self.pitch, self.roll = yaw0, 0.0, 0.0
        self.u = u0
        self.r = 0.0       # yaw rate
        self.q = 0.0       # pitch rate
        self.w = 0.0       # vertical water-relative velocity (down +)
        self.T = 0.0       # actual thrust
        self.fin_r = 0.0
        self.fin_e = 0.0
        self.cur_n, self.cur_e = cur
        self._gust_sigma = 0.0
        self._gust_tau = 10.0
        self._gust = [0.0, 0.0]
        self._rng = random.Random(seed)
        self._cmd = AttitudeCmd()
        self._last_cmd_t = -math.inf
        self._iy = 0.0
        self._ip = 0.0
        self._vn = self._ve = self._vd = 0.0
        self._update_ground()

    # ---- public API ----
    def command(self, cmd: AttitudeCmd, t: Optional[float] = None) -> None:
        """Deliver an attitude command (as the MAVLink message arrival). Resets deadman."""
        self._cmd = cmd
        self._last_cmd_t = self.t if t is None else t

    def set_current(self, n: float, e: float) -> None:
        self.cur_n, self.cur_e = n, e

    def set_gusts(self, sigma: float, tau: float = 10.0) -> None:
        """Ornstein-Uhlenbeck gust (m/s rms sigma, correlation time tau) added to the current."""
        self._gust_sigma, self._gust_tau = sigma, tau

    def set_water(self, kind, ballast_kg: float = 0.0) -> None:
        """Put the plant in `kind` ('fresh' | 'salt' | WaterType) carrying `ballast_kg` of lead.
        Sets net weight from hull mass/volume and scales hydrodynamics by density."""
        w = _w.get_water(kind)
        self.p.water = w
        self.p.net_weight_N = _w.net_weight_N(self.p.hull_mass_kg, self.p.hull_volume_L,
                                              ballast_kg, w)

    @property
    def rho_ratio(self) -> float:
        return self.p.water.density / _w.FRESH.density

    @property
    def deadman_active(self) -> bool:
        return (self.t - self._last_cmd_t) > self.p.deadman_s

    @property
    def truth(self) -> Truth:
        return Truth(self.t, self.n, self.e, self.d, self.roll, self.pitch, self.yaw, self.u,
                     self._vn, self._ve, self._vd, self.cur_n + self._gust[0],
                     self.cur_e + self._gust[1], self.fin_r, self.fin_e, self.T,
                     self.deadman_active)

    def step(self, dt: float) -> Truth:
        h = self.p.dt_internal
        n = max(1, int(round(dt / h)))
        hh = dt / n
        for _ in range(n):
            self._substep(hh)
        return self.truth

    # ---- internals ----
    def _update_ground(self):
        cn = self.cur_n + self._gust[0]
        ce = self.cur_e + self._gust[1]
        cp = math.cos(self.pitch)
        self._vn = self.u * cp * math.cos(self.yaw) + cn
        self._ve = self.u * cp * math.sin(self.yaw) + ce
        self._vd = self.w - self.u * math.sin(self.pitch)

    def _substep(self, h: float):
        p = self.p
        dm = self.deadman_active
        if dm:
            self._iy = self._ip = 0.0
            r_tgt = e_tgt = 0.0
            thr_cmd = 0.0 if p.deadman_zero_thrust else self._cmd.thrust
            if not math.isfinite(self._last_cmd_t):
                thr_cmd = 0.0
        else:
            c = self._cmd
            ey = wrap_pi(c.yaw - self.yaw)
            self._iy = max(-p.i_limit, min(p.i_limit, self._iy + p.yaw_ki * ey * h))
            r_tgt = p.yaw_kp * ey - p.yaw_kd * self.r + self._iy
            ep = c.pitch - self.pitch
            self._ip = max(-p.i_limit, min(p.i_limit, self._ip + p.pitch_ki * ep * h))
            e_tgt = p.pitch_kp * ep - p.pitch_kd * self.q + self._ip
            thr_cmd = c.thrust
        r_tgt = max(-p.fin_max, min(p.fin_max, r_tgt))
        e_tgt = max(-p.fin_max, min(p.fin_max, e_tgt))
        mr = p.fin_rate * h
        self.fin_r += max(-mr, min(mr, r_tgt - self.fin_r))
        self.fin_e += max(-mr, min(mr, e_tgt - self.fin_e))
        thr_cmd = max(0.0, min(1.0, thr_cmd))

        # thrust and surge (speed through water)
        self.T += (thr_cmd * p.thrust_max - self.T) * min(1.0, h / p.tau_thrust)
        rr = self.rho_ratio
        du = (self.T - rr * p.drag_k * self.u * abs(self.u)) / (p.mass + p.added_mass)
        self.u = max(0.0, self.u + du * h)
        u2 = self.u * self.u

        # yaw / pitch with fin forces ~ u^2
        r_ss = rr * p.k_rudder * u2 * self.fin_r
        self.r += (r_ss - self.r) * min(1.0, h / p.tau_r)
        self.yaw = wrap_pi(self.yaw + self.r * h)
        q_ss = rr * p.k_elev * u2 * self.fin_e - p.k_pitch_stab * self.pitch
        self.q += (q_ss - self.q) * min(1.0, h / p.tau_q)
        self.pitch = max(-p.pitch_max, min(p.pitch_max, self.pitch + self.q * h))

        # heave from net weight (+ sinks)
        dw = (p.net_weight_N - rr * p.heave_drag_k * self.w * abs(self.w)) / p.heave_mass
        self.w += dw * h

        # gusts (OU)
        if self._gust_sigma > 0:
            a = math.exp(-h / self._gust_tau)
            s = self._gust_sigma * math.sqrt(1 - a * a)
            self._gust = [a * g + s * self._rng.gauss(0, 1) for g in self._gust]

        self._update_ground()
        self.n += self._vn * h
        self.e += self._ve * h
        self.d += self._vd * h
        if self.d <= 0.0:
            self.d = 0.0
            if self.w < 0:
                self.w = 0.0
            if self._vd < 0:
                self._vd = 0.0
        self.t += h
