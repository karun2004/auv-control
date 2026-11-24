import math
from auv import water as W


def test_sign_convention():
    b = W.net_buoyancy_N(19.92, 20.0, 0.0, "fresh")
    assert b > 0 and abs(W.net_weight_N(19.92, 20.0, 0.0, "fresh") + b) < 1e-12
    assert abs(b - 0.78) < 0.01


def test_salt_is_more_buoyant_by_about_5N():
    d = W.net_buoyancy_N(19.92, 20.0, 0.0, "salt") - W.net_buoyancy_N(19.92, 20.0, 0.0, "fresh")
    assert 4.7 < d < 5.1


def test_required_ballast_roundtrip_and_500g():
    target = W.net_buoyancy_N(19.92, 20.0, 0.0, "fresh")
    kg = W.required_ballast_kg(19.92, 20.0, target, "salt")
    assert 0.45 < kg < 0.6
    assert abs(W.net_buoyancy_N(19.92, 20.0, kg, "salt") - target) < 1e-9
    assert abs(W.required_ballast_kg(19.92, 20.0, target, "fresh")) < 1e-9


def test_wrong_ballast_in_fresh_sinks():
    kg = W.required_ballast_kg(19.92, 20.0, 0.78, "salt")
    assert -4.5 < W.net_buoyancy_N(19.92, 20.0, kg, "fresh") < -3.8


def test_depth_spec_grav():
    p = W.pressure_at_depth(10.0, "salt")
    assert abs(W.depth_from_pressure(p, 1.025) - 10.0) < 1e-9
    assert abs(W.depth_from_pressure(p, 1.0) - 10.25) < 1e-9       # fresh setting in salt reads deep
    assert abs(W.depth_reading_scale("salt", 1.024) - 1.025 / 1.024) < 1e-12


def test_dvl_sign_flip_and_correction():
    f = W.dvl_velocity_scale("fresh"); s = W.dvl_velocity_scale("salt")
    assert 1.011 < f < 1.013 and 0.985 < s < 0.987          # +1.2% fresh, -1.4% salt
    assert abs(f * W.dvl_velocity_correction("fresh") - 1.0) < 1e-12
    assert abs(W.FRESH.dvl_scale_factor_correction * f - 1.0) < 1e-12
    assert W.dvl_velocity_scale("salt", 1521.0) == 1.0


def test_custom_water():
    w = W.custom_water(1012.5)
    assert 1482 < w.sound_speed < 1521 and abs(w.baro_spec_grav - 1.0125) < 1e-12


def test_checklist():
    c = W.predive_checklist(19.92, 20.0, "salt", 0.78)
    assert c["BARO_SPEC_GRAV"] == 1.024 and c["dvl_sound_speed_mps"] == 1521.0
    assert 0.5 < c["ballast_kg"] < 0.6
    assert W.predive_checklist(19.92, 20.0, "fresh", 0.78)["BARO_SPEC_GRAV"] == 1.0
