import math
from auv.types import *
from auv.mission import MissionManager, State, latlon_to_ned
from auv.breadcrumb import BreadcrumbTrail

H0 = (FailLevel.H0, "none")
M = {"name": "t", "surface_after_each_leg": False,
     "waypoints": [{"n": 50, "e": 0, "depth": 5, "timeout_s": 100},
                   {"n": 50, "e": 50, "depth": 5, "timeout_s": 100}]}


def nav(n=0.0, e=0.0, d=0.0, age=0.0):
    return NavState(0, n, e, d, 0, 0, 0, 0, 0, 1, 0, (1, 1), True, age)


def sens(t, good_gps=True, batt=16.0):
    return Sensors(t=t, gps=GpsSample(t, 0, 0, 3 if good_gps else 0, 1.0, 10, good_gps),
                   depth=DepthSample(t, 0.0), att=Attitude(t, 0, 0, 0), battery_v=batt)


def armed(mm=None, mission=M):
    mm = mm or MissionManager()
    mm.load(mission)
    t = 0.0
    for _ in range(3):
        mm.step(sens(t), nav(), H0, 0.1); t += 0.1
    assert mm.state == State.DIVE
    return mm, t


def test_load_latlon():
    mm = MissionManager()
    mm.load({"origin": {"lat": 10.0, "lon": 20.0},
             "waypoints": [{"lat": 10.001, "lon": 20.001, "depth": 3}]})
    w = mm.waypoints[0]
    assert abs(w.n - 111.3) < 1.0 and abs(w.e - 109.6) < 1.0
    assert latlon_to_ned(10, 20, 10, 20) == (0, 0)


def test_wait_gps_and_arm_checks():
    mm = MissionManager(); mm.load(M)
    mm.step(sens(0, good_gps=False), nav(), H0, 0.1)
    assert mm.state == State.WAIT_GPS
    mm.step(sens(0.1), nav(), H0, 0.1)
    assert mm.state == State.ARM_CHECK
    mm.step(sens(0.2, batt=13.0), nav(), H0, 0.1)
    assert mm.state == State.ARM_CHECK          # low battery blocks
    s = sens(0.3); s.gcs_heartbeat_age = 9
    mm.step(s, nav(), H0, 0.1)
    assert mm.state == State.ARM_CHECK          # heartbeat blocks
    mm.step(sens(0.4), nav(), H0, 0.1)
    assert mm.state == State.DIVE


def test_full_mission_to_done():
    mm, t = armed()
    r = mm.step(sens(t), nav(d=0.2), H0, 0.1)
    assert r["mode"] == "track" and mm.state == State.DIVE
    mm.step(sens(t), nav(d=4.6), H0, 0.1)
    assert mm.state == State.RUN
    mm.step(sens(t), nav(50, 0, 5), H0, 0.1)
    assert mm.idx == 1 and mm.state == State.RUN
    r = mm.step(sens(t), nav(50, 50, 5), H0, 0.1)
    assert mm.state == State.SURFACE and r["mode"] == "surface"
    mm.step(sens(t), nav(50, 50, 0.1), H0, 0.1)
    assert mm.state == State.FIX
    for i in range(310):
        t += 0.1
        r = mm.step(sens(t), nav(50, 50, 0.1), H0, 0.1)
        if mm.state == State.SYNC:
            break
    assert mm.state == State.SYNC and r["gps_fix"] is not None
    for _ in range(60):
        r = mm.step(sens(t), nav(50, 50, 0.1), H0, 0.1)
    assert mm.state == State.DONE and r["done"] and not mm.aborted


def test_fix_dwell_resets_on_gps_loss():
    mm = MissionManager(fix_dwell_s=5)
    mm._go(State.FIX)
    t = 0.0
    for _ in range(40):
        t += 0.1; mm.step(sens(t), nav(), H0, 0.1)
    for _ in range(5):
        t += 0.1; mm.step(sens(t, good_gps=False), nav(), H0, 0.1)
    assert mm.state == State.FIX and mm._dwell == 0.0
    for _ in range(60):
        t += 0.1; mm.step(sens(t), nav(), H0, 0.1)
    assert mm.state == State.SYNC


def test_abort_from_run_on_failsafe():
    mm, t = armed()
    mm.step(sens(t), nav(d=4.8), H0, 0.1)
    assert mm.state == State.RUN
    r = mm.step(sens(t), nav(10, 0, 5), (FailLevel.H7, "surface_now"), 0.1)
    assert mm.state == State.ABORT_SURFACE and r["mode"] == "surface" and mm.aborted
    mm.step(sens(t), nav(10, 0, 0.1), H0, 0.1)
    assert mm.state == State.FIX


def test_waypoint_timeout_aborts():
    mm, t = armed()
    mm.step(sens(t), nav(d=4.8), H0, 0.1)
    for _ in range(1100):
        r = mm.step(sens(t), nav(10, 0, 5), H0, 0.1)
        if mm.state != State.RUN:
            break
    assert mm.state == State.ABORT_SURFACE and "timeout" in mm.abort_reason


def test_return_positional_and_heading():
    for age, mode in ((0.0, "return_positional"), (40.0, "return_heading")):
        bc = BreadcrumbTrail()
        mm, t = armed(MissionManager(breadcrumb=bc))
        mm.step(sens(t), nav(d=4.8), H0, 0.1)
        for i in range(0, 41):                       # run north 40 m
            t += 2.0
            mm.step(sens(t), nav(float(i), 0, 5), H0, 0.1)
        assert mm.state == State.RUN
        r = mm.step(sens(t + 2), nav(40, 0, 5, age), (FailLevel.H2, "return_breadcrumb"), 0.1)
        assert mm.state == State.RETURN and r["mode"] == mode
        if mode == "return_positional":
            assert [(w.n, w.e) for w in r["waypoints"]][-1] == (0.0, 0.0)
        else:
            assert len(r["legs"]) == 1 and abs(abs(r["legs"][0][0]) - math.pi) < 1e-6
        # return finishes by timeout / arrival -> surfaces
        r = mm.step(sens(t + 3), nav(40, 0, 5, age), (FailLevel.H7, "surface_now"), 0.1)
        assert mm.state == State.ABORT_SURFACE


def test_surface_after_each_leg_resumes():
    m = dict(M, surface_after_each_leg=True)
    mm, t = armed(MissionManager(fix_dwell_s=1, sync_s=1), m)
    mm.step(sens(t), nav(d=4.8), H0, 0.1)
    mm.step(sens(t), nav(50, 0, 5), H0, 0.1)
    assert mm.state == State.SURFACE and mm.idx == 1
    mm.step(sens(t), nav(50, 0, 0.1), H0, 0.1)
    for _ in range(200):
        t += 0.1
        mm.step(sens(t), nav(50, 0, 0.1), H0, 0.1)
        if mm.state == State.DIVE:
            break
    assert mm.state == State.DIVE and mm.idx == 1


def test_hold_mode_in_fix_and_sync():
    mm, t = armed()
    mm._go(State.FIX)
    mm._hold = None
    mm._go(State.SURFACE)
    r = mm.step(sens(t), nav(n=4.0, e=5.0, d=0.1), H0, 0.1)
    assert mm.state == State.FIX and r["mode"] == "surface"
    r = mm.step(sens(t + 0.1), nav(n=4.5, e=5.0, d=0.1), H0, 0.1)
    assert r["mode"] == "hold" and r["target"] == (4.0, 5.0, 0.0)
    mm._go(State.SYNC)
    assert mm.step(sens(t + 0.2), nav(n=4.5, e=5.0, d=0.1), H0, 0.1)["mode"] == "hold"
