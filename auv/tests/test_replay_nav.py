import json
import math
import os
import pytest
from sim.replay import list_logs
from sim.replay_nav import analyse, SELF_CONSISTENT_M
from auv.nav_filter import DEPTH_SIGMA

LOGS = list_logs()
TRACK_TOL_M = 3.0      # end-of-log filter vs logged track, logs whose own track matches own velocity
_cache = {}


def res(path):
    if path not in _cache:
        _cache[path] = analyse(path)[0]
    return _cache[path]


@pytest.mark.parametrize("path", LOGS)
def test_no_crash_no_nan(path):
    r = res(path)
    assert r["exceptions"] == 0
    assert r["nan_count"] == 0


@pytest.mark.parametrize("path", LOGS)
def test_track_within_tolerance_on_consistent_logs(path):
    r = res(path)
    if not (r["usable"] and r["log_track_consistent"] and not r["log_track_static"]):
        pytest.skip("log track not a usable reference")
    assert r["track_div_end_m"] <= TRACK_TOL_M


@pytest.mark.parametrize("path", LOGS)
def test_depth_tracks_log_except_step_glitches(path):
    r = res(path)
    if not r["usable"]:
        pytest.skip("unusable")
    # p95 residual bounded (0.1 m): filter lags real descents and rare depth steps
    assert r["depth_resid_p95_m"] <= 0.1


@pytest.mark.parametrize("path", LOGS)
def test_cov_grows_only_without_dvl(path):
    r = res(path)
    if r["usable"] and r["dvl_age_max_s"] < 1.0:
        assert r["cov_pos_rms_max_m"] < 2.0


def test_some_logs_usable_and_documented():
    rs = [res(p) for p in LOGS]
    assert sum(r["usable"] for r in rs) >= 15
    assert all(r.get("unusable_reason") for r in rs if not r["usable"])


def test_depth_sigma_kept():
    """Measured within-window depth variance (cm-rounded logs ~2e-5, 0.03-step 06-27 log ~2.4e-4)
    brackets the filter's 7.5e-5 (0.03/sqrt12): no consistent material difference, keep it."""
    vs = [res(p)["depth_var_measured_m2"] for p in LOGS]
    vs = [v for v in vs if math.isfinite(v)]
    assert min(vs) < DEPTH_SIGMA ** 2 < max(vs)
