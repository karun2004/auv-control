import math
import numpy as np
from auv.types import Sensors, DvlSample, Attitude, DepthSample, GpsSample
from auv.nav_filter import NavFilter, chi2_gate

DT = 0.1
YAW = math.radians(45)


def dvl(t, vx=0.514, vy=0.0, vz=0.0, fom=0.1, valid=True):
    return DvlSample(t, (vx, vy, vz), fom, 5.0, valid)


def gps(t, n, e, **kw):
    d = dict(fix_type=3, hdop=1.0, sats=10, valid=True)
    d.update(kw)
    return GpsSample(t, n, e, **d)


def run_leg(f, rng, secs, noise=0.02, t0=0.0, yaw=YAW):
    t = t0
    st = None
    for _ in range(int(secs / DT)):
        t += DT
        v = 0.514 + rng.normal(0, noise)
        st = f.update(Sensors(t, dvl=dvl(t, v), att=Attitude(t, 0, 0, yaw),
                              depth=DepthSample(t, 5.0)))
    return st, t


def test_straight_leg_100m():
    rng = np.random.default_rng(1)
    f = NavFilter()
    st, _ = run_leg(f, rng, 100 / 0.514)
    assert abs(st.n - 70.7) < 3 and abs(st.e - 70.7) < 3
    assert st.healthy and st.dvl_age < 0.2


def test_dropout_inflates_cov_and_age():
    rng = np.random.default_rng(2)
    f = NavFilter()
    st, t = run_leg(f, rng, 20)
    c0 = st.cov_pos[0]
    for _ in range(80):
        t += DT
        st = f.update(Sensors(t, att=Attitude(t, 0, 0, YAW)))
    assert st.dvl_age > 7.9 and not st.healthy
    assert st.cov_pos[0] > 2 * c0
    assert st.vn > 0.2  # propagated with last velocity, not zeroed


def test_nan_and_invalid_dvl_rejected():
    f = NavFilter()
    rng = np.random.default_rng(3)
    st, t = run_leg(f, rng, 5)
    n0 = st.n
    bad = [DvlSample(t, (0, 0, 0), 0.1, 1, False),
           DvlSample(t, (0.5, 0, 0), math.nan, 1, True),
           DvlSample(t, (0.5, 0, 0), 0.9, 1, True),
           DvlSample(t, (math.nan, 0, 0), 0.1, 1, True)]
    for b in bad:
        t += DT
        st = f.update(Sensors(t, dvl=b, att=Attitude(t, 0, 0, YAW)))
        assert st.dvl_age > 0 and np.isfinite(st.n)
    # velocity not pulled to zero by the invalid "zero" sample
    assert st.vn > 0.25
    # zero FOM is a legitimate (perfect) value and is accepted
    t += DT
    st = f.update(Sensors(t, dvl=dvl(t, fom=0.0), att=Attitude(t, 0, 0, YAW)))
    assert st.dvl_age == 0.0


def _surfaced_filter():
    rng = np.random.default_rng(4)
    f = NavFilter()
    st, t = run_leg(f, rng, 20)
    f.set_surface(True)
    pos = (st.n, st.e)
    for _ in range(5):                       # build 5 consistent epochs
        t += 1.0
        st = f.update(Sensors(t, gps=gps(t, pos[0], pos[1])))
    return f, st, t


def test_chi2_gate_function():
    ok, d2 = chi2_gate([1.0, 1.0], np.eye(2))
    assert ok and abs(d2 - 2.0) < 1e-9
    ok, d2 = chi2_gate([10.0, 0.0], np.eye(2))
    assert not ok
    assert not chi2_gate([math.nan, 0], np.eye(2))[0]


def test_gps_spike_rejected_and_near_fix_accepted_without_teleport():
    f, st, t = _surfaced_filter()
    n, e = st.n, st.e
    acc = f.gps_accepted
    t += 1.0
    st = f.update(Sensors(t, gps=gps(t, n + 100, e)))
    assert f.gps_accepted == acc and abs(st.n - n) < 1.0
    t += 1.0
    st = f.update(Sensors(t, gps=gps(t, n + 4, e + 4)))
    assert f.gps_accepted == acc + 1
    moved = math.hypot(st.n - n, st.e - e)
    target = math.hypot(4, 4)
    assert 0.05 < moved < target - 0.2     # partway, not a teleport


def test_gps_requires_surface_quality_and_epochs():
    rng = np.random.default_rng(5)
    f = NavFilter()
    st, t = run_leg(f, rng, 10)
    t += 1; f.update(Sensors(t, gps=gps(t, st.n, st.e)))
    assert f.gps_accepted == 0 and f.gps_status == "not_surface"
    f.set_surface(True)
    for i in range(4):
        t += 1; f.update(Sensors(t, gps=gps(t, st.n, st.e)))
    assert f.gps_accepted == 0            # fewer than 5 epochs
    t += 1; f.update(Sensors(t, gps=gps(t, st.n, st.e, hdop=3.0)))
    assert f.gps_status == "quality"
    for i in range(5):
        t += 1; f.update(Sensors(t, gps=gps(t, st.n, st.e, sats=4)))
    assert f.gps_accepted == 0
    for i in range(5):
        t += 1; f.update(Sensors(t, gps=gps(t, st.n, st.e)))
    assert f.gps_accepted >= 1


def test_depth_not_differentiated():
    f = NavFilter()
    t = 0.0
    for i in range(100):               # constant depth, DVL vz = 0
        t += DT
        f.update(Sensors(t, dvl=dvl(t, 0.5, 0, 0.0), att=Attitude(t, 0, 0, 0),
                         depth=DepthSample(t, 5.0)))
    for i in range(3):                 # one 3 cm quantisation step (0.3 m/s if differenced)
        t += DT
        st = f.update(Sensors(t, dvl=dvl(t, 0.5, 0, 0.0), att=Attitude(t, 0, 0, 0),
                              depth=DepthSample(t, 5.03)))
    assert abs(st.vd) < 0.02
    # real descent taken from DVL vz
    f = NavFilter()
    t = 0.0
    for i in range(100):
        t += DT
        st = f.update(Sensors(t, dvl=dvl(t, 0.5, 0, 0.1), att=Attitude(t, 0, 0, 0),
                              depth=DepthSample(t, 0.1 * t)))
    assert abs(st.vd - 0.1) < 0.03 and abs(st.d - 0.1 * t) < 0.1


def test_water_track_sets_current():
    f = NavFilter()
    t = 0.0
    for i in range(50):
        t += DT
        f.update(Sensors(t, dvl=dvl(t, 0.6, 0, 0), att=Attitude(t, 0, 0, 0)))
    for i in range(30):
        f.update_water_track(0.5, 0.0)
    st = f.update(Sensors(t, att=Attitude(t, 0, 0, 0)))
    assert abs(st.cur_n - 0.1) < 0.03 and abs(st.cur_e) < 0.03
