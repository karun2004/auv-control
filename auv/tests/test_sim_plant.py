import math
from auv.types import AttitudeCmd
from sim.plant import Plant, PlantParams, SALT_NET_WEIGHT_N


def _run(p, cmd, secs, dt=0.1, resend=True):
    for _ in range(int(secs / dt)):
        if resend:
            p.command(cmd)
        p.step(dt)
    return p.truth


def test_floats_when_thrust_zero_fresh():
    p = Plant(d0=5.0)
    tr = _run(p, AttitudeCmd(0, 0, 0, 0.0), 60)
    assert tr.d == 0.0
    assert tr.u < 0.01


def test_sinks_in_salt_trim():
    p = Plant(PlantParams(net_weight_N=SALT_NET_WEIGHT_N), d0=1.0)
    tr = _run(p, AttitudeCmd(0, 0, 0, 0.0), 20)
    assert tr.d > 3.0


def test_no_lateral_motion_and_drag_speed():
    p = Plant(d0=3.0, yaw0=math.radians(30))
    p.p.net_weight_N = 0.0
    tr = _run(p, AttitudeCmd(0, 0, math.radians(30), 0.066), 200)
    assert 0.45 < tr.u < 0.6
    # ground velocity is along heading (no current): cross-heading component ~ 0
    vx = tr.vn * math.cos(tr.yaw) + tr.ve * math.sin(tr.yaw)
    vy = -tr.vn * math.sin(tr.yaw) + tr.ve * math.cos(tr.yaw)
    assert abs(vy) < 1e-6 and vx > 0.4


def test_current_advects_vehicle():
    p = Plant(d0=3.0, cur=(0.0, 0.2))
    p.p.net_weight_N = 0.0
    tr = _run(p, AttitudeCmd(0, 0, 0.0, 0.0), 30)
    assert abs(tr.ve - 0.2) < 1e-6 and tr.e > 5.0


def _turn_rate(u_water):
    p = Plant(d0=3.0, yaw0=0.0, u0=u_water)
    p.p.net_weight_N = 0.0
    p.p.drag_k = 0.0          # hold speed constant for the authority test
    p.p.yaw_kp, p.p.yaw_ki = 50.0, 0.0     # saturate the fin
    _run(p, AttitudeCmd(0, 0, 3.0, 0.0), 3)
    return p.r


def test_fin_authority_scales_with_u_squared():
    r1, r2 = _turn_rate(1.0), _turn_rate(0.514)
    assert abs(r2 / r1 - 0.514 ** 2) < 0.03


def test_deadman_returns_fins_to_trim():
    p = Plant(d0=3.0, u0=0.5)
    p.p.net_weight_N = 0.0
    cmd = AttitudeCmd(0, 0, math.pi / 2, 0.1)
    for _ in range(10):
        p.command(cmd); p.step(0.1)
    assert abs(p.fin_r) > 0.05 and not p.deadman_active
    for _ in range(4):          # 0.4 s without command
        p.step(0.1)
    assert p.deadman_active
    for _ in range(10):
        p.step(0.1)
    assert abs(p.fin_r) < 1e-6 and abs(p.fin_e) < 1e-6


def test_set_water_buoyancy_and_density_scaling():
    from auv import water as W
    p = Plant(); p.set_water("fresh", 0.0)
    assert -1.0 < p.p.net_weight_N < 0.0                      # floats ~0.78 N
    q = Plant(); q.set_water("salt", W.required_ballast_kg(19.92, 20.0, 0.7845, "salt"))
    assert abs(q.p.net_weight_N - p.p.net_weight_N) < 0.01
    assert abs(q.rho_ratio - 1.025) < 1e-9
    wrong = Plant(); wrong.set_water("fresh", q_ballast := W.required_ballast_kg(19.92, 20.0, 0.7845, "salt"))
    assert wrong.p.net_weight_N > 3.5                          # salt ballast in fresh sinks


def test_sensor_water_errors():
    from sim.sensors import SensorSuite
    s = SensorSuite(); s.set_water("fresh"); assert abs(s.p.dvl_scale_err - 0.01215) < 1e-4
    s.set_water("salt"); assert abs(s.p.dvl_scale_err + 0.01381) < 1e-4
    assert abs(s.p.depth_scale - 1.025 / 1.024) < 1e-9
    s.set_water("salt", baro_spec_grav=1.0); assert abs(s.p.depth_scale - 1.025) < 1e-9
    s.set_water("salt", dvl_sound_speed=1521.0); assert abs(s.p.dvl_scale_err) < 1e-12
