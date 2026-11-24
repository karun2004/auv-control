import math
import pytest

from auv.types import NavState, Attitude, Waypoint, CRUISE_SPEED
from auv.controllers import DepthController, HeadingController, Controller


def mk(n=0.0, e=0.0, d=0.0, vn=0.5, ve=0.0, vd=0.0):
    return NavState(t=0, n=n, e=e, d=d, vn=vn, ve=ve, vd=vd, cur_n=0, cur_e=0,
                    dvl_scale=1, yaw_bias=0, cov_pos=(0, 0))


def test_depth_sign_and_limits():
    c = DepthController()
    assert c.step(5.0, mk(d=0.0), 0.5, 0.1) < 0          # too shallow -> nose down
    c = DepthController()
    assert c.step(0.0, mk(d=5.0), 0.5, 0.1) > 0          # too deep -> nose up
    c = DepthController()
    assert c.step(100.0, mk(d=0.0), 0.5, 0.1) == pytest.approx(-math.radians(20))
    assert c.step(-100.0, mk(d=0.0), 0.5, 0.1) == pytest.approx(math.radians(20))


def test_depth_derivative_uses_vd_and_integral_clamped():
    c = DepthController()
    # on target but descending -> nose up to brake
    assert c.step(3.0, mk(d=3.0, vd=0.3), 0.5, 0.1) > 0
    c = DepthController()
    for _ in range(10000):
        c.step(3.0, mk(d=2.9), 0.5, 0.1)
    assert abs(c.integral) <= c.int_limit


def test_speed_ok_flag():
    c = DepthController()
    c.step(3.0, mk(d=0), 0.1, 0.1)
    assert not c.speed_ok
    c.step(3.0, mk(d=0), 0.5, 0.1)
    assert c.speed_ok


def test_heading_rate_limit_and_wrap():
    h = HeadingController(max_rate=math.radians(30))
    y = h.step(math.radians(179), math.radians(179), 0.1)
    assert y == pytest.approx(math.radians(179))
    # target across the +-pi seam: shortest way is +2 deg, not -358
    y = h.step(math.radians(-179), math.radians(179), 0.1)
    assert y == pytest.approx(math.radians(-179))
    h = HeadingController(max_rate=math.radians(30))
    h.step(0.0, 0.0, 0.1)
    y = h.step(math.pi / 2, 0.0, 0.1)
    assert y == pytest.approx(math.radians(3))


def test_heading_wraps_through_pi():
    h = HeadingController(max_rate=math.radians(30))
    h.step(math.radians(170), math.radians(170), 0.1)
    for _ in range(5):
        y = h.step(math.radians(-170), math.radians(170), 0.5)
        assert -math.pi <= y <= math.pi
    assert y == pytest.approx(math.radians(-170))


def test_controller_sequencing_and_complete():
    ctl = Controller()
    wps = [Waypoint(50, 0, 3.0), Waypoint(50, 50, 3.0)]
    att = Attitude(0, 0, 0, 0.0)
    cmd, st = ctl.step(mk(0, 0, 3.0), att, wps, 0.1)
    assert st["wp_index"] == 0 and not st["mission_complete"]
    assert 0.0 < cmd.thrust <= 1.0 and cmd.yaw == pytest.approx(0.0, abs=1e-6)
    cmd, st = ctl.step(mk(46, 0, 3.0), att, wps, 0.1)
    assert st["arrived"] and st["wp_index"] == 1
    cmd, st = ctl.step(mk(50, 46, 3.0), att, wps, 0.1)
    assert st["mission_complete"] and cmd.thrust == 0.0
    cmd, st = ctl.step(mk(50, 50, 3.0), att, wps, 0.1)
    assert st["mission_complete"]


def test_controller_thrust_floor_and_cruise():
    ctl = Controller()
    wps = [Waypoint(200, 0, 3.0)]
    att = Attitude(0, 0, 0, 0.0)
    cmd, _ = ctl.step(mk(0, 0, 3.0, vn=CRUISE_SPEED), att, wps, 0.1)
    assert cmd.thrust == pytest.approx(0.04, abs=0.02)
    # near capture: still above zero thrust demand floor
    for _ in range(50):
        cmd, st = ctl.step(mk(190, 0, 3.0, vn=0.3), att, wps, 0.1)
    assert cmd.thrust > 0.0


def test_controller_empty_list_no_crash():
    ctl = Controller()
    cmd, st = ctl.step(mk(), Attitude(0, 0, 0, 0.0), [], 0.1)
    assert st["mission_complete"]
