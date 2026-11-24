"""Parameter sweep for guidance + depth gains, and station-keeping drift scenario.

Run from project root:
  .venv/bin/python -m sim.tune sweep        # grid over LOS + depth gains
  .venv/bin/python -m sim.tune drift        # post-mission drift: old surface mode vs StationKeeper
Sweep patches auv.controllers.Controller with a factory, so defaults in the library are untouched.
"""
from __future__ import annotations
import itertools
import math
import sys

SCEN = ("two_waypoint", "cross_current_0p2")


def _run(names, g, d):
    import auv.controllers as C
    from auv.guidance import LosGuidance
    from sim import run_sitl
    orig = C.Controller
    C.Controller = lambda: orig(guidance=LosGuidance(**g), depth=C.DepthController(**d))
    try:
        S = run_sitl.scenarios()
        return [run_sitl.run_scenario(S[n], plot=False) for n in names]
    finally:
        C.Controller = orig


def sweep():
    grid_g = [dict(delta_min=a, delta_max=b, kappa=k)
              for a, b, k in itertools.product((3.0, 4.0, 6.0), (8.0, 10.0), (0.01, 0.02, 0.05))]
    grid_d = [dict(kp=kp, ki=ki, kd=kd)
              for kp, ki, kd in itertools.product((0.15, 0.25, 0.4), (0.01, 0.03), (0.4, 0.8))]
    rows = []
    # decoupled: guidance only affects xte, depth gains only depth (to first order)
    for g in grid_g:
        r = _run(SCEN, g, {})
        rows.append(("g", g, [x["cross_track_rms_m"] for x in r]))
    for d in grid_d:
        r = _run(SCEN, {}, d)
        rows.append(("d", d, [x["depth_rms_m"] for x in r]))
    for kind, p, v in sorted(rows, key=lambda x: (x[0], max(x[2]))):
        print(kind, p, [round(x, 3) for x in v])


def drift(currents=((0.0, 0.1), (0.0, 0.2), (0.2, 0.0), (0.0, 0.0)), fix_dwell_s=30.0):
    """Post-mission drift from the point where FIX starts, with and without StationKeeper.

    run_sitl does not know mode 'hold' (falls to thrust 0 = float, i.e. the 'before' case).
    Prints (max excursion from FIX-entry point, final distance to hold target) in metres. For 'after' we wrap MissionManager.step / Plant.command to substitute the StationKeeper
    command whenever the mission asks for 'hold'. To make this permanent in sim/run_sitl.py add
        elif mode == "hold":
            if sk is None: sk = StationKeeper()
            cmd, _ = sk.step(nav, att, out["target"], DT)
    and create sk = StationKeeper() next to Controller().
    """
    import math
    from auv.mission import MissionManager
    from auv.station_keep import StationKeeper
    from sim import run_sitl
    from sim.plant import Plant
    res = []
    for cur in currents:
        row = {"current": cur}
        for label in ("before", "after"):
            ctx = {"nav": None, "att": None, "out": None, "p0": None, "max": 0.0, "end": 0.0, "sk": StationKeeper()}
            o_step, o_cmd = MissionManager.step, Plant.command

            def mstep(self, s, nav, fail, dt, _o=o_step, _c=ctx):
                out = _o(self, s, nav, fail, dt)
                _c["nav"], _c["out"] = nav, out
                if s.att is not None:
                    _c["att"] = s.att
                return out

            def pcmd(self, cmd, t=None, _o=o_cmd, _c=ctx, _l=label):
                out = _c["out"]
                if out and out["mode"] == "hold":
                    tr = self.truth
                    if _c["p0"] is None:
                        _c["p0"] = (tr.n, tr.e)
                    _c["max"] = max(_c["max"], math.hypot(tr.n - _c["p0"][0], tr.e - _c["p0"][1]))
                    _c["end"] = math.hypot(tr.n - out["target"][0], tr.e - out["target"][1])
                    if _l == "after" and _c["att"] is not None:
                        cmd, _ = _c["sk"].step(_c["nav"], _c["att"], out["target"], run_sitl.DT)
                return _o(self, cmd, t)

            import auv.mission as am
            o_cls = am.MissionManager
            am.MissionManager = lambda **kw: o_cls(fix_dwell_s=fix_dwell_s, **kw)
            MissionManager.step, Plant.command = mstep, pcmd
            try:
                run_sitl.run_scenario(run_sitl.Scenario("drift", current=cur), plot=False)
            finally:
                MissionManager.step, Plant.command = o_step, o_cmd
                am.MissionManager = o_cls
            row[label] = (round(ctx["max"], 1), round(ctx["end"], 1))
        res.append(row)
        print(row)
    return res


if __name__ == "__main__":
    {"sweep": sweep, "drift": drift}.get(sys.argv[1] if len(sys.argv) > 1 else "sweep")()
