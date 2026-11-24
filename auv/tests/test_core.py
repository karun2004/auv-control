import math
import os

import pytest

from auv.core import AuvCore, CsvLogger, LOG_COLUMNS, validate_mission
from auv.mavlink_io import FakeLink
from auv.types import (Sensors, DvlSample, Attitude, DepthSample, GpsSample)
from auv.mission import State

MISSION = {"name": "t", "waypoints": [{"n": 30.0, "e": 0.0, "depth": 2.0}]}


def good_sensors(t, dvl=True, gps=True, depth=0.0):
    return Sensors(
        t=t,
        dvl=DvlSample(t, (0.5, 0.0, 0.0), 0.05, 5.0, True) if dvl else None,
        att=Attitude(t, 0.0, 0.0, 0.0),
        depth=DepthSample(t, depth),
        gps=GpsSample(t, 0.0, 0.0, 3, 1.0, 10, True) if gps else None,
        battery_v=16.0, gcs_heartbeat_age=0.2)


def run_ticks(core, link, t0, n, **kw):
    t = t0
    for _ in range(n):
        link.push(good_sensors(t, **kw))
        core.step(t)
        t += 0.05
    return t


def make(tmp_path=None, **kw):
    link = FakeLink()
    core = AuvCore(link, log_dir=str(tmp_path) if tmp_path else None, source="fake", **kw)
    link.connect()
    return core, link


def test_round_trip_reaches_dive_and_commands_attitude(tmp_path):
    core, link = make(tmp_path)
    core.submit_mission(MISSION)
    t = run_ticks(core, link, 0.0, 10)
    assert core.mm.state == State.DIVE
    kinds = link.kinds()
    assert "guided" in kinds and "arm" in kinds
    run_ticks(core, link, t, 5)
    assert "attitude" in kinds or "attitude" in link.kinds()
    cmd = [p for k, p in link.sent if k == "attitude"][-1]
    assert cmd.thrust > 0
    snap = core.snapshot()
    assert snap["running"] and snap["mission"]["state"] == "DIVE"


def test_heartbeat_cadence_through_core():
    core, link = make()
    run_ticks(core, link, 0.0, 200)           # 10 s
    assert 10 <= len(link.heartbeats) <= 11


def test_abort_sends_surface_and_aborts():
    core, link = make()
    core.submit_mission(MISSION)
    t = run_ticks(core, link, 0.0, 10)
    n_before = link.kinds().count("surface")
    core.abort()
    run_ticks(core, link, t, 2)
    assert link.kinds().count("surface") > n_before
    assert core.mm.aborted and core.mm.abort_reason == "operator abort"
    assert core.mm.state in (State.ABORT_SURFACE, State.FIX)


def test_mission_refused_while_running():
    core, link = make()
    core.submit_mission(MISSION)
    t = run_ticks(core, link, 0.0, 10)
    core.submit_mission(MISSION)
    run_ticks(core, link, t, 1)
    assert "refused" in core.snapshot()["mission"]["note"]


@pytest.mark.parametrize("bad", [
    {}, {"waypoints": []}, {"waypoints": [{"n": 1, "e": 1}]},
    {"waypoints": [{"n": 1, "e": 1, "depth": 99}]},
    {"waypoints": [{"n": 1, "e": 1, "depth": 0}]},
    {"waypoints": [{"n": float("nan"), "e": 1, "depth": 2}]},
    {"waypoints": [{"n": 9e9, "e": 1, "depth": 2}]},
    {"waypoints": [{"lat": 1, "lon": 1, "depth": 2}]},
    {"waypoints": [{"n": 1, "e": 1, "depth": 2, "timeout_s": -1}]},
    {"waypoints": [{"n": True, "e": 1, "depth": 2}]},
    {"waypoints": [{"n": 1, "e": 1, "depth": 2}] * 51},
    "nope",
])
def test_validate_rejects(bad):
    with pytest.raises(ValueError):
        validate_mission(bad)


def test_validate_accepts_latlon_with_origin():
    m = validate_mission({"origin": {"lat": 10, "lon": 20},
                          "waypoints": [{"lat": 10.0001, "lon": 20.0, "depth": 3}]})
    assert m["waypoints"][0]["timeout_s"] == 600.0


def read_log(path):
    lines = open(path).read().splitlines()
    hdr = [l for l in lines if l.startswith("#")]
    cols = [l for l in lines if not l.startswith("#")]
    names = cols[0].split(",")
    rows = [dict(zip(names, r.split(","))) for r in cols[1:]]
    return hdr, names, rows


def test_logger_schema_nan_not_zero(tmp_path):
    core, link = make(tmp_path)
    core._open_log()
    core.submit_mission(MISSION)
    t = run_ticks(core, link, 0.0, 5, dvl=False, gps=False)   # no DVL, no GPS samples
    core.close()
    path = os.path.join(str(tmp_path), core.log_files()[0])
    hdr, names, rows = read_log(path)
    assert tuple(names) == LOG_COLUMNS
    assert any("wall_anchor_utc=" in h and "mono_anchor_ns=" in h for h in hdr)
    assert len(rows) == 5
    r = rows[-1]
    for c in ("dvl_vx", "dvl_vy", "dvl_vz", "dvl_fom", "dvl_alt", "gps_fix", "gps_hdop",
              "gps_sats", "gps_n", "gps_e", "xte", "depth_raw"):
        assert r[c] == "nan", c                  # missing is nan, never 0
    assert r["f_dvl_valid"] == "0" and r["f_gps_valid"] == "0" and r["f_att_healthy"] == "1"
    assert r["source"] == "fake" and int(r["t_ns"]) > 0
    ts = [int(x["t_ns"]) for x in rows]
    assert ts == sorted(ts)


def test_logger_records_present_values(tmp_path):
    core, link = make(tmp_path)
    core._open_log()
    run_ticks(core, link, 0.0, 3)
    core.close()
    _, _, rows = read_log(os.path.join(str(tmp_path), core.log_files()[0]))
    assert float(rows[-1]["dvl_vx"]) == 0.5 and rows[-1]["gps_sats"] == "10"


def test_csvlogger_fmt():
    assert CsvLogger.fmt(None) == "nan" and CsvLogger.fmt(math.nan) == "nan"
    assert CsvLogger.fmt(0.0) == "0.0" and CsvLogger.fmt(True) == "1"


def test_thread_run_and_stop(tmp_path):
    core, link = make(tmp_path, rate_hz=100.0)
    core.start()
    import time
    time.sleep(0.3)
    core.stop()
    assert core.tick_count > 5 and "connected" not in core.last_error
