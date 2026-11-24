"""Open-loop trim sweep on the plant.  Run: .venv/bin/python -m sim.trim_sweep

Convention: B = net buoyancy in N, positive floats.  Plant net_weight_N = -B.
For each B (and water, heave drag): nose-down pitch needed to hold depth at cruise speed,
dive time to 2.5 m / 10 m at the controller pitch limit (-20 deg), and surface recovery time
from 10 m with the harness 'surface' command (pitch +0.3 rad, thrust 0.15) and passively (thrust 0).
"""
from __future__ import annotations
import math
import sys
from auv.types import AttitudeCmd
from auv import water as W
from sim.plant import Plant, PlantParams

THR_CRUISE = 0.055          # 1.1 N -> 0.514 m/s through water
PITCH_LIM = math.radians(20.0)


def make(B, water, k):
    p = Plant(PlantParams(net_weight_N=-B, heave_drag_k=k, water=W.get_water(water)), d0=5.0, u0=0.514)
    return p


def make_deep(B, water, k):
    p = make(B, water, k); p.d = 300.0
    return p


def hold_pitch(B, water, k, u_cmd_thr=THR_CRUISE):
    """Pitch (deg, + nose up) giving zero vertical speed at cruise; nan if |pitch| > 60 deg."""
    lo, hi = -1.0, 1.0
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        p = make_deep(B, water, k)
        cmd = AttitudeCmd(0, mid, 0, u_cmd_thr)
        for _ in range(500):          # 50 s, deep enough (d0=300 m) to never hit the surface clamp
            p.command(cmd); p.step(0.1)
        vd = p.truth.vd
        if vd > 0: lo = mid           # sinking: need more nose up
        else: hi = mid
    pit = 0.5 * (lo + hi)
    return math.degrees(pit) if abs(pit) < 0.99 else math.nan


def timed(B, water, k, pitch, thr, d0, target, tmax=600.0):
    p = make(B, water, k); p.d = d0; p.u = 0.514 if thr > 0 else 0.0
    cmd = AttitudeCmd(0, pitch, 0, thr)
    for i in range(int(tmax / 0.1)):
        p.command(cmd); tr = p.step(0.1)
        if (target > d0 and tr.d >= target) or (target < d0 and tr.d <= target):
            return tr.t
    return math.nan


def row(B, water, k):
    h = hold_pitch(B, water, k)
    d25 = timed(B, water, k, -PITCH_LIM, THR_CRUISE, 0.0, 2.5)
    d10 = timed(B, water, k, -PITCH_LIM, THR_CRUISE, 0.0, 10.0)
    s_act = timed(B, water, k, 0.3, 0.15, 10.0, 0.3)
    s_pas = timed(B, water, k, 0.0, 0.0, 10.0, 0.3)
    return h, d25, d10, s_act, s_pas


def fmt(x, f="{:6.1f}"):
    return "   n/a " if x != x else f.format(x)


def main():
    Bs = [-3, -2, -1, 0, 0.5, 1, 1.5, 2, 3, 4, 5]
    for water, k in (("fresh", 100.0), ("salt", 100.0)):
        print(f"\n## {water}, heave_drag_k={k}")
        print("B_N  hold_pitch_deg  dive2.5_s  dive10_s  surf_active_s  surf_passive_s")
        for B in Bs:
            r = row(B, water, k)
            print(f"{B:4}  {fmt(r[0])}  {fmt(r[1])}  {fmt(r[2])}  {fmt(r[3])}  {fmt(r[4])}")
    print("\n## heave-drag sensitivity (fresh): hold pitch deg / passive surf s")
    ks = [50, 100, 150, 200]
    print("B_N  " + "  ".join(f"k={k:<3}        " for k in ks))
    for B in Bs:
        cells = []
        for k in ks:
            r = row(B, "fresh", float(k))
            cells.append(f"{fmt(r[0])}/{fmt(r[4], '{:5.0f}')}")
        print(f"{B:4}  " + "  ".join(cells))


if __name__ == "__main__":
    main()
