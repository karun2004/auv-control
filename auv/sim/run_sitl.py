"""Closed-loop SITL scenario harness: plant + sensors + NavFilter + Controller + Failsafe + Mission.

Run from the project root:   .venv/bin/python -m sim.run_sitl [scenario ...] [--list]
Writes sim/out/<scenario>.png and sim/out/results.json.
auv modules are imported lazily inside run_scenario().

Wiring per 0.1 s tick:
  truth -> SensorSuite.sample -> NavFilter.update -> FailsafeMonitor.update(s, nav, t)
  -> MissionManager.step(s, nav, fail, dt) -> dict(mode, waypoints, ...)
  mode 'track'/'return_positional' -> Controller.step(nav, att, waypoints, dt)
  mode 'return_heading'            -> timed legs [(heading, dur)] with depth hold
  mode 'surface'                   -> nose up, light thrust until shallow, then thrust 0
  mode 'idle'                      -> thrust 0
  -> Plant.command / Plant.step
Because MissionManager hands out a fresh slice each tick and Controller resets when id(list)
changes, the harness reuses one list object while the slice contents are unchanged.
"""
from __future__ import annotations
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Callable, Optional

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
# Plan targets used in the pass criteria.
XTRACK_TARGET_M = 2.0
DEPTH_TARGET_M = 0.3
# Columns of the single per-run SITL log file (sim/out/sitl_<timestamp>.csv), after 'scenario'.
LOG_COLS = ("t", "n", "e", "d", "yaw", "nav_n", "nav_e", "state", "level", "xte", "dref", "thr")
DT = 0.1


@dataclass
class Scenario:
    name: str
    waypoints: list = field(default_factory=lambda: [
        {"n": 60.0, "e": 0.0, "depth": 3.0}, {"n": 60.0, "e": 50.0, "depth": 3.0}])
    duration: float = 700.0
    current: tuple = (0.0, 0.0)
    net_weight_N: float = -0.5             # fresh-trim default for scenarios (see sim.plant: -2 N is marginal)
    compass_bias_deg: float = 0.0
    water: Optional[str] = None             # 'fresh' | 'salt': overrides net_weight_N via auv.water
    ballast_kg: float = 0.0                 # lead carried (kg)
    dvl_sound_speed: float = 1500.0         # DVL setting (m/s)
    baro_spec_grav: Optional[float] = None  # BARO_SPEC_GRAV setting; None = correct for the water
    setup: Optional[Callable] = None        # setup(plant, suite)
    events: list = field(default_factory=list)   # [(t, fn(plant, suite))]
    expect_return: bool = False             # expects abort + surface
    note: str = ""


def _salt_ballast() -> float:
    from auv import water as W
    return W.required_ballast_kg(19.92, 20.0, W.net_buoyancy_N(19.92, 20.0, 0.0, "fresh"), "salt")


SALT_BALLAST_KG = _salt_ballast()


def scenarios() -> dict:
    S = {}
    def add(s): S[s.name] = s
    add(Scenario("two_waypoint", note="baseline"))
    add(Scenario("two_waypoint_fresh_m2N", net_weight_N=-2.0, note="spec fresh trim"))
    add(Scenario("two_waypoint_fresh", water="fresh", ballast_kg=0.0,
                 note="recommended fresh trim: +0.78 N buoyant, no ballast (docs/TRIM.md)"))
    add(Scenario("two_waypoint_salt", water="salt", ballast_kg=SALT_BALLAST_KG,
                 note="salt: same +0.78 N via ballast, DVL +/- sound-speed error, BARO 1.024"))
    add(Scenario("wrong_ballast_salt_in_fresh", water="fresh", ballast_kg=SALT_BALLAST_KG,
                 note="safety: salt ballast left on in fresh (-4.1 N, sinks). Expect abort/surface or honest fail"))
    add(Scenario("cross_current_0p2", current=(0.0, 0.2)))
    add(Scenario("compass_bias_3deg", compass_bias_deg=3.0))
    add(Scenario("dvl_dead_6s", setup=lambda p, s: s.kill_dvl(90.0, 96.0), expect_return=True))
    add(Scenario("attitude_loss", setup=lambda p, s: s.kill_attitude(90.0), expect_return=True))
    add(Scenario("depth_loss", setup=lambda p, s: s.kill_depth(90.0), expect_return=True))
    add(Scenario("battery_critical", setup=lambda p, s: s.set_battery(12.5, 90.0),
                 expect_return=True))
    add(Scenario("leak", setup=lambda p, s: s.inject_leak(90.0), expect_return=True))
    add(Scenario("current_gt_0p3", events=[(90.0, lambda p, s: p.set_current(0.35, 0.0))],
                 expect_return=True))
    return S


def _seg_dist(p, a, b):
    ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return math.hypot(p[0] - ax, p[1] - ay)
    return abs(dx * (p[1] - ay) - dy * (p[0] - ax)) / L     # infinite-line cross-track


def run_scenario(sc: Scenario, seed: int = 1, plot: bool = True, rows: Optional[list] = None) -> dict:
    from auv.types import AttitudeCmd, Attitude, FailLevel, CRUISE_SPEED
    from auv.nav_filter import NavFilter
    from auv.controllers import Controller, DepthController
    from auv.failsafe import FailsafeMonitor
    from auv.mission import MissionManager, State
    from auv.breadcrumb import BreadcrumbTrail
    from sim.plant import Plant
    from sim.sensors import SensorSuite

    plant = Plant(seed=seed, cur=sc.current)
    suite = SensorSuite(seed=seed, plant=plant)
    if sc.water:
        plant.set_water(sc.water, sc.ballast_kg)
        suite.set_water(sc.water, sc.dvl_sound_speed, sc.baro_spec_grav)
    else:
        plant.p.net_weight_N = sc.net_weight_N
    suite.set_compass_bias_deg(sc.compass_bias_deg)
    if sc.setup:
        sc.setup(plant, suite)
    events = sorted(sc.events, key=lambda x: x[0])

    nf = NavFilter(0.0, 0.0)
    ctrl = Controller()
    depth_hold = DepthController()
    sk = None
    fs = FailsafeMonitor()
    bc = BreadcrumbTrail()
    mm = MissionManager(breadcrumb=bc)
    mm.load({"name": sc.name, "waypoints": sc.waypoints})
    wps = mm.waypoints

    att = Attitude(0.0, 0.0, 0.0, 0.0)
    cache_key, cache_list = None, []
    leg_i, leg_t = 0, 0.0
    hold_depth = 2.0
    T_m = u_m = 0.0
    surface_states = (State.WAIT_GPS, State.ARM_CHECK, State.FIX, State.SYNC, State.DONE)
    log = {k: [] for k in ("t", "n", "e", "d", "yaw", "nav_n", "nav_e", "state", "level", "xte",
                            "dref", "thr")}
    max_level = 0
    final_level = FailLevel.H0
    n_steps = int(sc.duration / DT)
    dive_t0 = None
    for k in range(n_steps):
        t = plant.t
        while events and events[0][0] <= t:
            events.pop(0)[1](plant, suite)
        tr = plant.truth
        s = suite.sample(tr)
        if s.att is not None and s.att.healthy:
            att = s.att
        nav = nf.update(s)
        st = mm.state
        nf.set_surface(st in surface_states or (st in (State.ABORT_SURFACE, State.SURFACE)
                                                 and nav.d < 0.3))
        fs.dvl_required = st not in surface_states and st not in (State.FIX,)
        if st in (State.DIVE, State.RUN, State.RETURN) and dive_t0 is None:
            dive_t0 = t
        fail = fs.update(s, nav, t - (dive_t0 if dive_t0 is not None else t))
        out = mm.step(s, nav, fail, DT)
        level = fail[0]
        final_level = level
        max_level = max(max_level, level.value)
        mode = out["mode"]
        cmd = AttitudeCmd(0.0, 0.0, att.yaw, 0.0)
        dref = math.nan
        if mode in ("track", "return_positional"):
            wl = out["waypoints"]
            key = tuple(id(w) for w in wl)
            if key != cache_key:
                cache_key, cache_list = key, list(wl)
            if cache_list:
                cmd, status = ctrl.step(nav, att, cache_list, DT)
                dref = cache_list[min(ctrl.idx, len(cache_list) - 1)].depth
                hold_depth = dref
            if mode == "track" and mm.state == State.DIVE:
                pass
        elif mode == "return_heading":
            legs = out.get("legs", [])
            if leg_i < len(legs):
                hdg, dur = legs[leg_i]
                leg_t += DT
                if leg_t > dur:
                    leg_i, leg_t = leg_i + 1, 0.0
                u = max(0.0, (nav.vn - nav.cur_n) * math.cos(att.yaw)
                        + (nav.ve - nav.cur_e) * math.sin(att.yaw))
                pitch = depth_hold.step(hold_depth, nav, u, DT)
                cmd = AttitudeCmd(0.0, pitch, hdg, 0.08)
                dref = hold_depth
        elif mode == "hold":
            from auv.station_keep import StationKeeper
            sk = sk or StationKeeper()
            cmd, _ = sk.step(nav, att, out["target"], DT)
        elif mode == "surface":
            if nav.d > 0.4:
                cmd = AttitudeCmd(0.0, 0.3, att.yaw, 0.15, surface=True)
            else:
                cmd = AttitudeCmd(0.0, 0.0, att.yaw, 0.0, surface=True)
        # fail-safe may have blinded Pi (attitude loss etc.): still command what mission asked
        # water-track model: first-order thrust/drag speed model (perfect plant params, note!)
        pp = plant.p
        T_m = T_m + (min(1.0, max(0.0, cmd.thrust)) * pp.thrust_max - T_m) * min(1.0, DT / pp.tau_thrust)
        u_m = max(0.0, u_m + (T_m - pp.drag_k * u_m * abs(u_m)) / (pp.mass + pp.added_mass) * DT)
        if nav.d > 0.5 and u_m > 0.1:
            nf.update_water_track(u_m * math.cos(att.yaw), u_m * math.sin(att.yaw))
        plant.command(cmd)
        plant.step(DT)

        idx = min(out.get("wp_index", 0), len(wps) - 1)
        a = (0.0, 0.0) if idx == 0 else (wps[idx - 1].n, wps[idx - 1].e)
        xte = _seg_dist((tr.n, tr.e), a, (wps[idx].n, wps[idx].e))
        for key_, val in (("t", t), ("n", tr.n), ("e", tr.e), ("d", tr.d), ("yaw", tr.yaw),
                          ("nav_n", nav.n),
                          ("nav_e", nav.e), ("state", mm.state.name), ("level", level.value),
                          ("xte", xte), ("dref", dref), ("thr", cmd.thrust)):
            log[key_].append(val)
        if mm.state == State.DONE and t > 5:
            break

    import numpy as np
    t_ = np.array(log["t"]); d_ = np.array(log["d"]); xte_ = np.array(log["xte"])
    run = np.array([s == "RUN" for s in log["state"]])
    dref_ = np.array(log["dref"])
    m = run & np.isfinite(dref_)
    res = {
        "scenario": sc.name,
        "sim_time_s": float(t_[-1]),
        "cross_track_rms_m": float(np.sqrt(np.mean(xte_[run] ** 2))) if run.any() else None,
        "depth_rms_m": float(np.sqrt(np.mean((d_[m] - dref_[m]) ** 2))) if m.any() else None,
        "closure_error_to_last_wp_m": float(math.hypot(log["n"][-1] - wps[-1].n,
                                                       log["e"][-1] - wps[-1].e)),
        "closure_error_to_start_m": float(math.hypot(log["n"][-1], log["e"][-1])),
        "max_depth_m": float(d_.max()),
        "final_fail_level": final_level.name,
        "max_fail_level": FailLevel(max_level).name,
        "final_state": mm.state.name,
        "aborted": bool(mm.aborted), "abort_reason": mm.abort_reason,
        "surfaced": bool(d_[-1] < 0.3 and d_.max() > 1.0),
        "expect_return": sc.expect_return,
        "reasons": list(getattr(fs, "reasons", [])),
        "nav_err_final_m": float(math.hypot(log["n"][-1] - log["nav_n"][-1],
                                            log["e"][-1] - log["nav_e"][-1])),
    }
    # Explicit pass criteria. A failsafe scenario must abort and surface. A normal scenario must
    # not abort, must reach the last waypoint (closest approach within capture radius), must
    # surface, and must meet the cross-track and depth targets. 'why_failed' names what missed.
    from auv.types import CAPTURE_RADIUS
    closest = float(min(math.hypot(a - wps[-1].n, b - wps[-1].e)
                        for a, b in zip(log["n"], log["e"])))
    res["closest_to_last_wp_m"] = closest
    checks = {"surfaced": res["surfaced"]}
    if sc.expect_return:
        checks["aborted"] = bool(mm.aborted)
    else:
        checks["not_aborted"] = not mm.aborted
        checks["reached_last_wp"] = closest <= CAPTURE_RADIUS
        checks["xtrack_rms_lt_2m"] = res["cross_track_rms_m"] is not None and res["cross_track_rms_m"] < XTRACK_TARGET_M
        checks["depth_rms_lt_0.3m"] = res["depth_rms_m"] is not None and res["depth_rms_m"] < DEPTH_TARGET_M
    res["checks"] = checks
    res["why_failed"] = [k for k, ok in checks.items() if not ok]
    res["pass"] = not res["why_failed"]
    if rows is not None:
        for i in range(len(log["t"])):
            rows.append([sc.name] + [log[c][i] for c in LOG_COLS])
    if plot:
        _plot(sc, log, wps)
    return res


def _plot(sc, log, wps):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT_DIR, exist_ok=True)
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    ax[0].plot(log["e"], log["n"], label="truth")
    ax[0].plot(log["nav_e"], log["nav_n"], "--", label="nav")
    ax[0].plot([0] + [w.e for w in wps], [0] + [w.n for w in wps], "k:o", label="plan")
    ax[0].set_xlabel("east m"); ax[0].set_ylabel("north m"); ax[0].axis("equal"); ax[0].legend()
    ax[1].plot(log["t"], log["d"], label="depth"); ax[1].plot(log["t"], log["dref"], ":", label="ref")
    ax[1].invert_yaxis(); ax[1].set_xlabel("t s"); ax[1].legend()
    ax[2].plot(log["t"], log["level"], drawstyle="steps-post"); ax[2].set_xlabel("t s")
    ax[2].set_ylabel("fail level (enum value)")
    fig.suptitle(sc.name); fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, sc.name + ".png"), dpi=80)
    plt.close(fig)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    S = scenarios()
    if "--list" in argv:
        print("\n".join(S)); return 0
    names = [a for a in argv if not a.startswith("-")] or list(S)
    os.makedirs(OUT_DIR, exist_ok=True)
    results = []
    rows = []
    for nme in names:
        try:
            r = run_scenario(S[nme], rows=rows)
        except Exception as ex:  # keep going: report harness/contract errors per scenario
            import traceback
            r = {"scenario": nme, "error": repr(ex), "trace": traceback.format_exc()}
        results.append(r)
        print(json.dumps({k: v for k, v in r.items() if k != "trace"}))
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump(results, fh, indent=2, default=str)
    import csv
    import time
    log_path = os.path.join(OUT_DIR, "sitl_" + time.strftime("%Y%m%d_%H%M%S") + ".csv")
    with open(log_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(("scenario",) + LOG_COLS)
        w.writerows(rows)
    print("log:", log_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
