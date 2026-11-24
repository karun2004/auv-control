"""Run every field log through NavFilter and write sim/out/replay_report.json + a PNG per log.

Run from the project root:  python -m sim.replay_nav

Per-log metrics
  dvl_valid_frac     : fraction of rows the replay marks DVL-valid (FOM present, not sentinel)
  gated_out          : replay-valid DVL rows that NavFilter still rejected (FOM > fom_thresh)
  dvl_age_max_s      : max NavState.dvl_age
  track_div_end/max  : |filter (n,e) - logged (Local_X, Local_Y)|, with the filter started at the
                       first logged position.  Logged X,Y is NOT truth (see 'log_self_inconsistency').
  log_self_inconsistency_m : |integral of logged local velocity over valid rows - logged X,Y track| at
                       the end; large => the logged track itself disagrees with the logged velocity.
  depth_resid_*      : filter depth - logged depth (rms / p95 / max abs), vs the log's own quantisation step
  cov_pos_*          : trace-sqrt of position covariance, start / end / max
  nan_count/exceptions : NaN fields in any emitted NavState / exceptions raised
Usable: >= MIN_ROWS rows and dvl_valid_frac >= MIN_VALID_FRAC.  Otherwise 'unusable_reason' is set.
"""
from __future__ import annotations
import json
import math
import os
import numpy as np
from auv.nav_filter import NavFilter, DEPTH_SIGMA
from sim.replay import list_logs, load_replay

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
PNG_DIR = os.path.join(OUT_DIR, "replay")
MIN_ROWS = 100
MIN_VALID_FRAC = 0.5
SELF_CONSISTENT_M = 3.0      # log track and its own velocity agree to this at the end
FIELDS = ("n", "e", "d", "vn", "ve", "vd", "cur_n", "cur_e", "dvl_scale", "yaw_bias")


def run_nav(rp, nf=None):
    """Feed a Replay through NavFilter. Returns (states, nan_count, exceptions)."""
    nf = nf or NavFilter(n0=float(rp.local_xy[0, 0]), e0=float(rp.local_xy[0, 1]))
    states, nan_count, exc = [], 0, 0
    for s in rp.sensors:
        try:
            st = nf.update(s)
        except Exception:
            exc += 1
            continue
        vals = [getattr(st, f) for f in FIELDS] + list(st.cov_pos) + [st.dvl_age]
        if not all(math.isfinite(v) for v in vals):
            nan_count += 1
        states.append(st)
    return states, nan_count, exc, nf


def measured_depth_var(depth, step, win=20):
    """Pooled within-window variance of logged depth over 'still' windows (range <= 2 quant steps,
    so real depth motion is excluded). Includes the log's own rounding, so it is an upper bound on
    sensor noise. nan if fewer than 5 still windows."""
    d = np.asarray(depth, float)
    d = d[np.isfinite(d)]
    if not math.isfinite(step):
        return math.nan
    vs = []
    for i in range(0, d.size - win + 1, win):
        w = d[i:i + win]
        if np.ptp(w) <= 2 * step + 1e-9:
            vs.append(np.var(w, ddof=1))
    return float(np.mean(vs)) if len(vs) >= 5 else math.nan


def analyse(path):
    rp = load_replay(path)
    name = os.path.basename(path)
    n = len(rp)
    states, nan_count, exc, nf = run_nav(rp)
    out = {"file": name, "schema": rp.schema, "rows": n, "duration_s": float(rp.t[-1]),
           "dvl_valid_frac": float(np.mean(rp.dvl_valid)),
           "nan_count": nan_count, "exceptions": exc}
    if not states:
        out["unusable_reason"] = "no states"
        return out, rp, states
    ok = np.array([nf._dvl_ok(s.dvl) for s in rp.sensors])
    out["gated_out"] = int(np.sum(rp.dvl_valid & ~ok))
    out["dvl_age_max_s"] = float(max(s.dvl_age for s in states))
    ne = np.array([(s.n, s.e) for s in states])
    div = np.hypot(*(ne - rp.local_xy[:len(ne)]).T)
    out["track_div_end_m"] = float(div[-1])
    out["track_div_max_m"] = float(div.max())
    dt = np.diff(rp.t, prepend=rp.t[0])
    vint = np.cumsum(np.where(rp.dvl_valid[:, None], np.nan_to_num(rp.vel_local), 0.0) * dt[:, None], axis=0)
    selfinc = float(np.hypot(*(vint[-1] - (rp.local_xy[-1] - rp.local_xy[0]))))
    out["log_self_inconsistency_m"] = selfinc
    out["log_track_consistent"] = bool(selfinc <= SELF_CONSISTENT_M)
    out["path_length_logged_m"] = float(np.sum(np.hypot(*np.diff(rp.local_xy, axis=0).T)))
    d_f = np.array([s.d for s in states])
    r = d_f - rp.depth[:len(d_f)]
    r = r[np.isfinite(r)]
    dd = np.unique(np.round(rp.depth[np.isfinite(rp.depth)], 6))
    steps = np.diff(dd)
    step = float(steps[steps > 1e-9].min()) if np.any(steps > 1e-9) else math.nan
    out["depth_quant_step_m"] = step
    out["depth_resid_rms_m"] = float(np.sqrt(np.mean(r ** 2))) if r.size else math.nan
    out["depth_resid_max_m"] = float(np.max(np.abs(r))) if r.size else math.nan
    out["depth_resid_p95_m"] = float(np.percentile(np.abs(r), 95)) if r.size else math.nan
    out["log_depth_jumps_gt_0p5m"] = int(np.sum(np.abs(np.diff(rp.depth[np.isfinite(rp.depth)])) > 0.5))
    out["log_track_static"] = bool(np.ptp(rp.local_xy, axis=0).max() == 0.0)
    out["depth_var_measured_m2"] = measured_depth_var(rp.depth, step)
    cp = np.array([math.sqrt(s.cov_pos[0] + s.cov_pos[1]) for s in states])
    out["cov_pos_rms_start_m"] = float(cp[0]); out["cov_pos_rms_end_m"] = float(cp[-1])
    out["cov_pos_rms_max_m"] = float(cp.max())
    reasons = []
    if n < MIN_ROWS:
        reasons.append(f"only {n} rows")
    if out["dvl_valid_frac"] < MIN_VALID_FRAC:
        reasons.append(f"DVL valid fraction {out['dvl_valid_frac']:.2f} < {MIN_VALID_FRAC}")
    if rp.schema == "rov_min":
        reasons.append("no FOM column: velocity cannot be trusted")
    out["usable"] = not reasons
    if reasons:
        out["unusable_reason"] = "; ".join(reasons)
    return out, rp, states


def plot(out, rp, states, png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
    ne = np.array([(s.n, s.e) for s in states])
    ax[0].plot(rp.local_xy[:, 1], rp.local_xy[:, 0], "k-", lw=1, label="log X/Y")
    ax[0].plot(ne[:, 1], ne[:, 0], "r-", lw=1, label="NavFilter")
    ax[0].set_xlabel("east / Local_Y (m)"); ax[0].set_ylabel("north / Local_X (m)")
    ax[0].axis("equal"); ax[0].legend(); ax[0].grid(alpha=.3)
    ax[0].set_title(f"{out['file']}  div_end={out.get('track_div_end_m', float('nan')):.1f} m")
    ax[1].plot(rp.t, rp.depth, "k.", ms=2, label="log depth")
    ax[1].plot(rp.t[:len(states)], [s.d for s in states], "r-", lw=1, label="filter")
    ax[1].invert_yaxis(); ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("depth (m)")
    ax[1].legend(); ax[1].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(png, dpi=80); plt.close(fig)


def main():
    os.makedirs(PNG_DIR, exist_ok=True)
    logs = []
    for p in list_logs():
        out, rp, states = analyse(p)
        if states:
            plot(out, rp, states, os.path.join(PNG_DIR, os.path.splitext(out["file"])[0] + ".png"))
        logs.append(out)
    var = [l["depth_var_measured_m2"] for l in logs if l.get("depth_var_measured_m2") == l.get("depth_var_measured_m2")]
    summary = {
        "logs": len(logs),
        "total_nan": sum(l["nan_count"] for l in logs),
        "total_exceptions": sum(l["exceptions"] for l in logs),
        "usable": [l["file"] for l in logs if l.get("usable")],
        "unusable": {l["file"]: l.get("unusable_reason") for l in logs if not l.get("usable")},
        "depth_sigma_filter_m": DEPTH_SIGMA,
        "depth_var_filter_m2": DEPTH_SIGMA ** 2,
        "depth_var_measured_median_m2": float(np.median(var)) if var else math.nan,
        "depth_var_measured_max_m2": float(np.max(var)) if var else math.nan,
    }
    with open(os.path.join(OUT_DIR, "replay_report.json"), "w") as fh:
        json.dump({"summary": summary, "logs": logs}, fh, indent=1)
    return summary, logs


if __name__ == "__main__":
    s, logs = main()
    print(json.dumps(s, indent=1))
    for l in logs:
        print(l["file"][-26:], *(f"{k}={l[k]:.3g}" if isinstance(l.get(k), float) else f"{k}={l.get(k)}" for k in
              ("rows", "dvl_valid_frac", "gated_out", "track_div_end_m", "track_div_max_m", "log_self_inconsistency_m", "depth_resid_max_m", "depth_quant_step_m", "depth_var_measured_m2", "cov_pos_rms_max_m", "dvl_age_max_s")))
