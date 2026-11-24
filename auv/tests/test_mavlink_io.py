import math
from types import SimpleNamespace as NS

import pytest

from auv.mavlink_io import (Link, FakeLink, euler_to_quat, thrust_to_wire,
                            SUB_MODE_GUIDED, SUB_MODE_SURFACE)
from auv.types import AttitudeCmd, Sensors


def test_quat_identity_and_unit_norm():
    assert euler_to_quat(0, 0, 0) == pytest.approx((1, 0, 0, 0))
    for r, p, y in [(0.3, -0.2, 2.5), (-1.0, 0.7, -3.0), (0, 0, math.pi)]:
        assert sum(c * c for c in euler_to_quat(r, p, y)) == pytest.approx(1.0)


def test_quat_single_axis_and_roundtrip():
    w, x, y, z = euler_to_quat(0, 0, math.pi / 2)          # yaw 90 deg
    assert (w, z) == pytest.approx((math.cos(math.pi / 4),) * 2)
    assert x == pytest.approx(0) and y == pytest.approx(0)
    w, x, y, z = euler_to_quat(0, 0.2, 0)                   # +pitch about +y (nose up)
    assert y == pytest.approx(math.sin(0.1)) and y > 0

    r0, p0, y0 = 0.3, -0.2, 1.1
    w, x, y, z = euler_to_quat(r0, p0, y0)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    assert (roll, pitch, yaw) == pytest.approx((r0, p0, y0))


def test_thrust_mapping():
    assert thrust_to_wire(0.4) == 0.4
    assert thrust_to_wire(2.0) == 1.0 and thrust_to_wire(-1) == 0.0
    assert thrust_to_wire(float("nan")) == 0.0
    assert thrust_to_wire(0.0, "centered") == 0.5 and thrust_to_wire(1.0, "centered") == 1.0
    with pytest.raises(ValueError):
        thrust_to_wire(0.1, "bogus")


class FakeMav:
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        return lambda *a: self.calls.append((name, a))


class FakeConn:
    def __init__(self, msgs=()):
        self.mav = FakeMav()
        self.msgs = list(msgs)

    def recv_match(self, blocking=False):
        assert blocking is False
        return self.msgs.pop(0) if self.msgs else None


def msg(ty, src=1, **kw):
    kw.setdefault("get_srcSystem", None)
    m = NS(**{k: v for k, v in kw.items() if k != "get_srcSystem"})
    m.get_type = lambda: ty
    m.get_srcSystem = lambda: src
    m.get_srcComponent = lambda: 1
    return m


def test_heartbeat_cadence_1hz():
    l = Link(conn=FakeConn())
    l.connect(now=0.0)
    for k in range(101):                      # 10 s at 10 Hz
        l.service(k * 0.1)
    hb = [c for c in l.conn.mav.calls if c[0] == "heartbeat_send"]
    assert len(hb) == 10 or len(hb) == 11
    assert hb[0][1][0] == 6                   # MAV_TYPE_GCS


def test_mode_arm_attitude_commands():
    l = Link(conn=FakeConn(), thrust_mode="direct")
    l.connect(now=0.0)
    l.set_guided(); l.request_surface(); l.arm(); l.disarm()
    c = l.conn.mav.calls
    assert c[0][0] == "command_long_send" and c[0][1][2] == 176 and c[0][1][5] == SUB_MODE_GUIDED
    assert c[1][1][5] == SUB_MODE_SURFACE
    assert c[2][1][2] == 400 and c[2][1][4] == 1
    assert c[3][1][4] == 0
    l.send_attitude(AttitudeCmd(0.0, 0.1, 1.0, 0.3), now=5.0)
    name, a = l.conn.mav.calls[-1]
    assert name == "set_attitude_target_send"
    assert a[3] == 0b111 and a[4] == pytest.approx(list(euler_to_quat(0, 0.1, 1.0)))
    assert a[-1] == 0.3


def test_poll_parses_telemetry_nonblocking():
    t0 = 100.0
    msgs = [
        msg("HEARTBEAT", src=255, type=6, autopilot=8, base_mode=0, custom_mode=0),
        msg("HEARTBEAT", src=1, type=12, autopilot=3, base_mode=128, custom_mode=4),
        msg("ATTITUDE", roll=0.1, pitch=-0.2, yaw=1.0),
        msg("SCALED_PRESSURE2", press_abs=1013.25),
        msg("SCALED_PRESSURE2", press_abs=1013.25 + 3 * 9.80665 * 1000 / 100.0),
        msg("DISTANCE_SENSOR", orientation=25, current_distance=550),
        msg("VISION_SPEED_ESTIMATE", x=0.5, y=0.01, z=0.0, covariance=[0.0004] + [0] * 8),
        msg("GPS_RAW_INT", lat=int(10e7), lon=int(20e7), fix_type=3, eph=120,
            satellites_visible=9),
        msg("SYS_STATUS", voltage_battery=15800),
        msg("STATUSTEXT", text="Leak detected"),
    ]
    l = Link(conn=FakeConn(msgs))
    l.connect(now=t0)
    s = l.poll(t0 + 1.0)
    assert s.att.yaw == 1.0 and s.att.pitch == -0.2
    assert s.depth.depth == pytest.approx(3.0, rel=1e-6)
    assert s.dvl.v_body == (0.5, 0.01, 0.0)
    assert s.dvl.fom == pytest.approx(0.02) and s.dvl.altitude == pytest.approx(5.5)
    assert s.gps.valid and s.gps.hdop == pytest.approx(1.2) and s.gps.sats == 9
    assert s.battery_v == pytest.approx(15.8) and s.leak is True
    assert s.gcs_heartbeat_age == 0.0     # GCS heartbeat arrived this poll
    assert l.armed and l.mode == 4


def test_missing_data_is_none_or_nan_not_zero():
    l = Link(conn=FakeConn())
    l.connect(now=0.0)
    s = l.poll(2.0)
    assert s.att is None and s.dvl is None and s.depth is None and s.gps is None
    assert math.isnan(s.battery_v)
    assert s.gcs_heartbeat_age == pytest.approx(2.0)


def test_fakelink_roundtrip():
    f = FakeLink(); f.connect()
    f.push(Sensors(t=1.0, battery_v=16.0))
    assert f.poll(1.0).battery_v == 16.0
    assert math.isnan(f.poll(1.1).battery_v)
    f.send_attitude(AttitudeCmd(thrust=0.2)); f.request_surface()
    assert f.kinds() == ["attitude", "surface"]
