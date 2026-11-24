import math
import numpy as np
import pytest
from sim.replay import list_logs, load_replay, replay_stats

LOGS = list_logs()


def test_found_all_logs():
    assert len(LOGS) == 23


@pytest.mark.parametrize("path", LOGS)
def test_parse_and_monotonic(path):
    rp = load_replay(path)
    assert len(rp) > 0
    assert np.all(np.diff(rp.t) >= 0)
    assert rp.t[0] == 0.0
    for s in rp.sensors[:50]:
        assert s.gps is None
        if not s.dvl.valid:
            assert math.isnan(s.dvl.fom)
    st = replay_stats(path)
    assert st["rows"] == len(rp) and st["duration_s"] >= 0


def test_rotation_round_trip():
    from auv.nav_filter import rot_body_to_ned
    rp = load_replay([p for p in LOGS if "06-27" in p][0])
    k = 0
    for s, v, a in zip(rp.sensors, rp.vel_local, rp.yaw):
        if s.dvl.valid and np.hypot(*v) > 0.01:
            R = rot_body_to_ned(s.att.roll, s.att.pitch, s.att.yaw)
            vn = R @ np.array(s.dvl.v_body)
            assert abs(vn[0] - v[0]) < 1e-9 and abs(vn[1] - v[1]) < 1e-9
            k += 1
    assert k > 10


def test_missing_marked_invalid():
    rp = load_replay([p for p in LOGS if "T13-11-10.csv" in p][0])
    # 99.9 sentinel FOM rows must be invalid
    assert np.all(~rp.dvl_valid[rp.fom >= 50])
