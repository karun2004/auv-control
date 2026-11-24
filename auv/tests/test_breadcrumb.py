import math
from auv.breadcrumb import BreadcrumbTrail


def test_thinning_rate_and_distance():
    b = BreadcrumbTrail()
    assert b.record(0.0, 0, 0, 5, 0, 0.5)
    assert not b.record(0.2, 5, 0, 5, 0, 0.5)     # too soon
    assert not b.record(1.0, 0.3, 0, 5, 0, 0.5)   # too close, same yaw
    assert b.record(2.0, 1.5, 0, 5, 0, 0.5)
    assert b.record(3.0, 1.5, 0, 5, math.radians(3), 0.5)  # yaw change
    assert len(b) == 3


def test_ring_bounded():
    b = BreadcrumbTrail(maxlen=100)
    for i in range(1000):
        b.record(i, i * 2.0, 0, 5, 0, 0.5)
    assert len(b) == 100


def _lpath():
    b = BreadcrumbTrail()
    t = 0.0
    for i in range(0, 51):       # north 50 m
        b.record(t, float(i), 0.0, 5, 0.0, 0.5); t += 2
    for i in range(1, 31):       # east 30 m
        b.record(t, 50.0, float(i), 5, math.pi / 2, 0.5); t += 2
    return b


def test_positional_return_simplified_and_reversed():
    b = _lpath()
    wps = b.positional_return(tolerance=1.0)
    pts = [(w.n, w.e) for w in wps]
    assert pts == [(50.0, 0.0), (0.0, 0.0)]   # corner then start; current pos excluded
    assert len(wps) < len(b) / 10


def test_heading_return():
    b = _lpath()
    legs = b.heading_return(tolerance=1.0, speed=0.5)
    assert len(legs) == 2
    (h1, d1), (h2, d2) = legs
    assert abs(h1 - (-math.pi / 2)) < 1e-6 and abs(d1 - 60.0) < 1e-6   # west 30 m
    assert abs(abs(h2) - math.pi) < 1e-6 and abs(d2 - 100.0) < 1e-6    # south 50 m


def test_heading_return_speed_estimate_and_empty():
    b = BreadcrumbTrail()
    assert b.positional_return() == [] and b.heading_return() == []
    for i in range(10):
        b.record(i, i * 2.0, 0, 5, 0, 0.4)
    legs = b.heading_return()
    assert abs(legs[0][1] - 18.0 / 0.4) < 1e-6
