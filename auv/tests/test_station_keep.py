import math
from auv.types import NavState, Attitude
from auv.station_keep import StationKeeper

DT = 0.1


def run(cur, depth=0.0, T=600.0, start=(0.0, 0.0), target=(0.0, 0.0), yaw0=0.0, hs=0.0):
    """Kinematic loop: first-order thrust/surge, heading follows command at <=6 deg/s
    (what the sim fins give at ~0.5 m/s), ground = u*heading + current, perfect nav."""
    sk = StationKeeper()
    n, e = start
    yaw, u, Tn = yaw0, hs, 0.0
    log = []
    for k in range(int(T / DT)):
        vn, ve = u * math.cos(yaw) + cur[0], u * math.sin(yaw) + cur[1]
        nav = NavState(k * DT, n, e, depth, vn, ve, 0.0, cur[0], cur[1], 1.0, 0.0, (1, 1))
        cmd, st = sk.step(nav, Attitude(k * DT, 0, 0, yaw), (target[0], target[1], depth), DT)
        Tn += (cmd.thrust * 20.0 - Tn) * DT / 0.3
        u = max(0.0, u + (Tn - 4.1 * u * u) / 30.0 * DT)
        dy = max(-math.radians(6) * DT, min(math.radians(6) * DT, ((cmd.yaw - yaw + math.pi) % (2 * math.pi)) - math.pi))
        yaw += dy
        n += (u * math.cos(yaw) + cur[0]) * DT
        e += (u * math.sin(yaw) + cur[1]) * DT
        log.append((n, e, st))
    return log


def radius(log, tail=0.5):
    tail = log[int(len(log) * (1 - tail)):]
    return max(math.hypot(n, e) for n, e, _ in tail)


def test_hold_surface_0p1_and_0p2():
    for c in (0.1, 0.2):
        for ang in (0.0, 1.0, 2.5):
            cur = (c * math.cos(ang), c * math.sin(ang))
            lg = run(cur, start=(5.0, -4.0), yaw0=1.0)
            assert lg[-1][2]["mode"] == "hold_into_current"
            assert radius(lg) < 2.0, (c, ang, radius(lg))


def test_hold_midwater_0p2():
    lg = run((0.0, 0.2), depth=2.0, start=(3.0, 3.0))
    assert lg[-1][2]["mode"] == "hold_into_current"
    assert radius(lg) < 2.0


def test_loiter_zero_current_midwater():
    R = 10.0
    lg = run((0.0, 0.0), depth=2.0, start=(0.0, 0.0), hs=0.4)
    assert lg[-1][2]["mode"] == "loiter"
    tail = lg[len(lg) // 2:]
    assert all(abs(math.hypot(n, e) - R) < 4.0 for n, e, _ in tail)


def test_float_and_zero_speed_no_crash():
    lg = run((0.0, 0.0), depth=0.0, T=30.0)
    assert lg[-1][2]["mode"] == "float" and lg[-1][0] == 0.0
    sk = StationKeeper()
    nav = NavState(0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0.0, (1, 1))
    cmd, st = sk.step(nav, Attitude(0, 0, 0, 0), (0.0, 0.0, 0.0), 0.0)
    assert cmd.thrust == 0.0 and st["mode"] == "float"

