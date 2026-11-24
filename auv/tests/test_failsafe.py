import math
from auv.types import *
from auv.failsafe import FailsafeMonitor


def nav(t=0.0, cur=(0, 0)):
    return NavState(t, 0, 0, 5, 0, 0, 0, cur[0], cur[1], 1.0, 0.0, (1, 1))


def sens(t, dvl=True, att=True, depth=5.0, pitch=0.0, **kw):
    return Sensors(t=t,
                   dvl=DvlSample(t, (0.5, 0, 0), 0.01, 3.0, True) if dvl else None,
                   att=Attitude(t, 0, pitch, 0, True) if att else None,
                   depth=DepthSample(t, depth) if depth is not None else None, **kw)


def run(fm, t0, t1, dt=0.1, **kw):
    out = None
    t = t0
    while t <= t1 + 1e-9:
        out = fm.update(sens(t, **kw), nav(t), t)
        t += dt
    return out


def test_healthy():
    assert run(FailsafeMonitor(), 0, 3) == (FailLevel.H0, "none")


def test_h1_then_clears_with_one_sample():
    fm = FailsafeMonitor()
    run(fm, 0, 1)
    assert run(fm, 1.1, 2.5, dvl=False)[0] == FailLevel.H1
    assert fm.update(sens(2.6), nav(), 2.6) == (FailLevel.H0, "none")


def test_h2_dvl_dead():
    fm = FailsafeMonitor()
    run(fm, 0, 1)
    assert run(fm, 1.1, 7.0, dvl=False) == (FailLevel.H2, "return_breadcrumb")


def test_h3_blind_surface_then_outputs_off_and_latched():
    fm = FailsafeMonitor(blind_surface_s=5)
    run(fm, 0, 1)
    assert run(fm, 1.1, 1.9, att=False) == (FailLevel.H3, "blind_surface")
    # attitude recovers: still latched
    assert run(fm, 2.0, 3.0)[1] == "blind_surface"
    assert run(fm, 3.1, 10.0) == (FailLevel.H3, "outputs_off")


def test_h4_depth_timeout_latched():
    fm = FailsafeMonitor()
    run(fm, 0, 1)
    assert run(fm, 1.1, 2.5, depth=None) == (FailLevel.H4, "surface_now")
    assert run(fm, 2.6, 4.0)[0] == FailLevel.H4


def test_h6_gcs():
    fm = FailsafeMonitor()
    assert run(fm, 0, 1, gcs_heartbeat_age=6.0) == (FailLevel.H6, "surface_now")
    assert run(fm, 1.1, 1.5)[0] == FailLevel.H0   # not latched


def test_h7_variants():
    for kw in (dict(depth=13.0), dict(pitch=math.radians(50)), dict(leak=True),
               dict(battery_v=12.8)):
        fm = FailsafeMonitor()
        assert run(fm, 0, 0.5, **kw) == (FailLevel.H7, "surface_now"), kw
        assert run(fm, 0.6, 1.0)[0] == FailLevel.H7   # latched


def test_runtime():
    fm = FailsafeMonitor(max_runtime=100)
    assert fm.update(sens(0), nav(), 101) == (FailLevel.H7, "surface_now")


def test_h7b_current_needs_duration():
    fm = FailsafeMonitor()
    out = None
    for i in range(0, 150):
        t = i * 0.1
        out = fm.update(sens(t), nav(t, cur=(0.3, 0.3)), t)
        if t < 10.0:
            assert out[0] == FailLevel.H0
    assert out == (FailLevel.H7B, "surface_now")
    # transient spike resets
    fm = FailsafeMonitor()
    for i in range(0, 80):
        t = i * 0.1
        fm.update(sens(t), nav(t, cur=(0.4, 0)), t)
    for i in range(80, 300):
        t = i * 0.1
        assert fm.update(sens(t), nav(t), t)[0] == FailLevel.H0


def test_highest_severity_wins():
    fm = FailsafeMonitor()
    run(fm, 0, 1)
    out = run(fm, 1.1, 7.0, dvl=False, leak=True)
    assert out == (FailLevel.H7, "surface_now")
