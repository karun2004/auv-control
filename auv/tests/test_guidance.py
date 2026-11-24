import math
import pytest

from auv.types import NavState, wrap_pi
from auv.guidance import LosGuidance


def mk(n, e, vn=0.0, ve=0.0):
    return NavState(t=0, n=n, e=e, d=0, vn=vn, ve=ve, vd=0, cur_n=0, cur_e=0,
                    dvl_scale=1, yaw_bias=0, cov_pos=(0, 0))


def run(current_e, e0=20.0, leg=800.0, T=1500.0, dt=0.1, u=0.514, rate=math.radians(20)):
    g = LosGuidance()
    g.set_leg((0, 0), (leg, 0))          # north leg
    n, e, psi = 0.0, e0, 0.0
    tail = []
    for i in range(int(T / dt)):
        vn = u * math.cos(psi)
        ve = u * math.sin(psi) + current_e
        psi_d, xte, along, arr = g.step(mk(n, e, vn, ve), dt, yaw=psi)
        if arr:
            break
        psi += max(-rate * dt, min(rate * dt, wrap_pi(psi_d - psi)))
        psi = wrap_pi(psi)
        n += vn * dt
        e += ve * dt
        if i * dt > T - 100:
            tail.append(xte)
    return tail, g


def test_converges_no_current():
    tail, _ = run(0.0)
    assert tail and max(abs(x) for x in tail) < 0.5


def test_converges_with_current_integral_removes_offset():
    tail, g = run(0.2)
    assert tail and max(abs(x) for x in tail) < 1.0
    assert g.integral > 0   # crabs into the current (current pushes east, + e)


def test_sign_convention():
    g = LosGuidance()
    g.set_leg((0, 0), (100, 0))
    psi, xte, along, _ = g.step(mk(10, 5, 0.5, 0), 0.1)
    assert xte > 0 and psi < 0          # right of line -> steer left
    g.set_leg((0, 0), (100, 0))
    psi, xte, _, _ = g.step(mk(10, -5, 0.5, 0), 0.1)
    assert xte < 0 and psi > 0
    assert along == pytest.approx(10)


def test_on_line_heads_along_leg():
    g = LosGuidance()
    g.set_leg((0, 0), (0, 100))   # east
    psi, xte, _, _ = g.step(mk(0, 10, 0, 0.5), 0.1)
    assert psi == pytest.approx(math.pi / 2) and xte == pytest.approx(0)


def test_arrival_radius_and_past():
    g = LosGuidance(capture_radius=8.0)
    g.set_leg((0, 0), (100, 0))
    assert not g.step(mk(50, 0), 0.1)[3]
    assert g.step(mk(93, 0), 0.1)[3]
    # past B but outside radius laterally
    assert g.step(mk(101, 20), 0.1)[3]


def test_zero_length_leg_no_crash():
    g = LosGuidance()
    g.set_leg((5, 5), (5, 5))
    psi, xte, along, arr = g.step(mk(5, 5), 0.1)
    assert arr and math.isfinite(psi)
    psi, xte, along, arr = g.step(mk(-50, 5), 0.1)
    assert not arr and psi == pytest.approx(0.0)


def test_integral_clamped():
    g = LosGuidance()
    g.set_leg((0, 0), (1e6, 0))
    for _ in range(100000):
        g.step(mk(100, 5, 0.5, 0), 1.0)
    assert abs(g.integral) <= g.int_max + 1e-9


def test_yaw_wrap_near_pi():
    g = LosGuidance()
    g.set_leg((0, 0), (-100, -1))     # heading ~ south, near +-pi
    psi, *_ = g.step(mk(-10, 0.0, -0.5, 0), 0.1)
    assert -math.pi <= psi <= math.pi
    assert abs(wrap_pi(psi - math.pi)) < 0.5
