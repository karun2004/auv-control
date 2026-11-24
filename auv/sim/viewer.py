"""Animated AUV viewer driven by field logs (no hardware).

Run from the project root:
    .venv/bin/python -m sim.viewer                 # first log, interactive window
    .venv/bin/python -m sim.viewer --list          # list logs with index
    .venv/bin/python -m sim.viewer --log 7         # by index or filename substring
    .venv/bin/python -m sim.viewer --log 7 --save out.gif   # headless render, no window

What you see: the logged values replayed through NavFilter.
  Top view  : logged track (grey), filter estimate (blue, grows as the AUV travels),
              torpedo-shaped AUV drawn at the filter position, pointing along logged heading.
  Side view : water column, surface line, seabed from DVL altitude, AUV at filter depth.
  Status    : time, depth, heading, speed, DVL valid/gated, position uncertainty, dvl_age.
Keys: space pause/play, left/right previous/next log, up/down speed, r restart.
"""
from __future__ import annotations
import argparse
import math
import os
import numpy as np

from sim.replay import list_logs, load_replay
from sim.replay_nav import run_nav

PLAYBACK = (0.5, 1, 2, 4, 8, 16, 32)


def prepare(path):
    """Replay one log through NavFilter. Returns dict of arrays aligned per row."""
    rp = load_replay(path)
    states, nan_count, exc, _ = run_nav(rp)
    n = min(len(states), len(rp.sensors))
    t = np.asarray(rp.t[:n], float)
    nav = np.array([[s.n, s.e, s.d, s.dvl_age, math.sqrt(max(s.cov_pos[0], 0) + max(s.cov_pos[1], 0))]
                    for s in states[:n]])
    alt = np.array([(s.dvl.altitude if (s.dvl is not None and s.dvl.valid) else math.nan)
                    for s in rp.sensors[:n]], float)
    valid = np.asarray(rp.dvl_valid[:n], bool)
    vel = np.asarray(rp.vel_local)[:n]
    return dict(name=os.path.basename(path), t=t, nav=nav, log_xy=np.asarray(rp.local_xy)[:n],
                yaw=np.asarray(rp.yaw)[:n], depth=np.asarray(rp.depth)[:n], alt=alt,
                valid=valid, speed=np.hypot(vel[:, 0], vel[:, 1]), nan=nan_count, exc=exc)


def sitl_items(csv_path):
    """One (label, loader) per scenario in a single sim/out/sitl_*.csv log (see run_sitl.LOG_COLS)."""
    import csv
    by = {}
    with open(csv_path, newline="") as fh:
        for r in csv.DictReader(fh):
            by.setdefault(r["scenario"], []).append(r)

    def make(name, rows):
        def load():
            g = lambda k: np.array([float(r[k]) for r in rows])
            t, n, e, d = g("t"), g("n"), g("e"), g("d")
            dt = np.maximum(np.diff(t, prepend=t[0] - 0.1), 1e-3)
            nav = np.column_stack([g("nav_n"), g("nav_e"), d, np.zeros_like(t), np.zeros_like(t)])
            return dict(name="SITL " + name, t=t, nav=nav, log_xy=np.column_stack([n, e]),
                        yaw=g("yaw"), depth=d, alt=np.full_like(t, np.nan),
                        valid=np.ones_like(t, bool), speed=np.hypot(np.gradient(n, t), np.gradient(e, t)),
                        nan=0, exc=0)
        return load
    return [("SITL " + k, make(k, v)) for k, v in by.items()]


def log_items(paths):
    return [(os.path.basename(p), (lambda p=p: prepare(p))) for p in paths]


def auv_shape(n, e, yaw, length):
    """Torpedo hull + tail fins as (east, north) polygons; yaw rad, 0 = north, clockwise."""
    hull = np.array([[1.0, 0.0], [0.7, 0.22], [-0.6, 0.22], [-1.0, 0.1], [-1.0, -0.1],
                     [-0.6, -0.22], [0.7, -0.22]]) * (length / 2)
    fins = np.array([[-0.75, 0.22], [-1.05, 0.5], [-0.95, 0.22], [-0.95, -0.22], [-1.05, -0.5],
                     [-0.75, -0.22]]) * (length / 2)
    fe, fn = math.sin(yaw), math.cos(yaw)         # forward unit vector in (east, north)
    out = []
    for p in (hull, fins):                        # p[:,0] forward, p[:,1] port
        east = e + p[:, 0] * fe - p[:, 1] * fn
        north = n + p[:, 0] * fn + p[:, 1] * fe
        out.append(np.column_stack([east, north]))
    return out


class Viewer:
    def __init__(self, items, index, fig=None, bind_keys=True):
        """items: list of (label, loader) from log_items()/sitl_items(); fig: reuse a shared figure."""
        import matplotlib.pyplot as plt
        self.plt, self.items = plt, items
        self.speed_i, self.paused, self.frame = 3, False, 0
        self.fig = fig or plt.figure(figsize=(12, 7))
        self.build()
        if bind_keys:
            self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.load(index)

    def build(self, rect=None):
        """(Re)create this viewer's axes. rect=(l,b,w,h) in figure fraction, or whole figure."""
        from matplotlib.patches import Polygon
        if rect is None:
            gs = self.fig.add_gridspec(2, 2, width_ratios=[1.6, 1], height_ratios=[2, 1])
            self.ax_top = self.fig.add_subplot(gs[:, 0])
            self.ax_side = self.fig.add_subplot(gs[0, 1])
            self.ax_txt = self.fig.add_subplot(gs[1, 1])
        else:
            l, b, w, h = rect
            self.ax_top = self.fig.add_axes([l + 0.05 * w, b + 0.08 * h, 0.55 * w, 0.82 * h])
            self.ax_side = self.fig.add_axes([l + 0.68 * w, b + 0.45 * h, 0.30 * w, 0.45 * h])
            self.ax_txt = self.fig.add_axes([l + 0.66 * w, b + 0.02 * h, 0.32 * w, 0.36 * h])
        self.axes = [self.ax_top, self.ax_side, self.ax_txt]
        self.hull = Polygon(np.zeros((3, 2)), closed=True, fc="#1f77b4", ec="k", zorder=5)
        self.fins = Polygon(np.zeros((3, 2)), closed=True, fc="#ff7f0e", ec="k", zorder=5)
        self.ax_top.add_patch(self.hull)
        self.ax_top.add_patch(self.fins)

    def load(self, index):
        ax1, ax2 = self.ax_top, self.ax_side
        self.idx = index % len(self.items)
        self.label, loader = self.items[self.idx]
        self.d = d = loader()
        self.frame = 0
        for a in list(ax1.lines):
            a.remove()
        ax1.set_title(f"Top view: {d['name']}")
        allx = np.concatenate([d["log_xy"][:, 1], d["nav"][:, 1]])
        ally = np.concatenate([d["log_xy"][:, 0], d["nav"][:, 0]])
        span = max(np.ptp(allx), np.ptp(ally), 1.0)
        pad = 5 + 0.1 * span
        ax1.set_xlim(allx.min() - pad, allx.max() + pad)
        ax1.set_ylim(ally.min() - pad, ally.max() + pad)
        ax1.set_aspect("equal", adjustable="datalim")
        ax1.set_xlabel("East (m)")
        ax1.set_ylabel("North (m)")
        ax1.grid(alpha=0.3)
        ax1.plot(d["log_xy"][:, 1], d["log_xy"][:, 0], color="0.7", lw=1, label="logged track")
        (self.trail,) = ax1.plot([], [], color="#1f77b4", lw=2, label="filter estimate")
        ax1.legend(loc="upper right", fontsize=8)
        self.length = max(2.0, 0.05 * span)
        floor = d["depth"] + d["alt"]
        self.dmax = max(float(np.nanmax(np.concatenate([d["depth"], floor[np.isfinite(floor)]]))) * 1.1, 1.0)
        ax2.clear()
        ax2.set_xlim(0, max(d["t"][-1], 1))
        ax2.set_ylim(self.dmax, -0.06 * self.dmax)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Depth (m)")
        ax2.set_title("Side view")
        ax2.axhspan(-0.06 * self.dmax, 0, color="#bde0fe", alpha=0.6)
        ax2.axhline(0, color="#3a86ff", lw=1)
        ax2.fill_between(d["t"], np.where(np.isfinite(floor), floor, np.nan), self.dmax,
                         color="#c2a878", alpha=0.7)
        ax2.plot(d["t"], d["depth"], color="0.6", lw=1)
        (self.dtrail,) = ax2.plot([], [], color="#1f77b4", lw=2)
        (self.dmark,) = ax2.plot([], [], "o", color="#ff7f0e", ms=9, zorder=5)
        self.update(0)

    def update(self, i):
        d = self.d
        i = int(min(max(i, 0), len(d["t"]) - 1))
        yaw = d["yaw"][i] if np.isfinite(d["yaw"][i]) else 0.0
        hull, fins = auv_shape(d["nav"][i, 0], d["nav"][i, 1], yaw, self.length)
        self.hull.set_xy(hull)
        self.fins.set_xy(fins)
        self.trail.set_data(d["nav"][:i + 1, 1], d["nav"][:i + 1, 0])
        self.dtrail.set_data(d["t"][:i + 1], d["nav"][:i + 1, 2])
        self.dmark.set_data([d["t"][i]], [d["nav"][i, 2]])
        self.ax_txt.clear()
        self.ax_txt.axis("off")
        lines = [f"t         {d['t'][i]:7.1f} s   x{PLAYBACK[self.speed_i]}",
                 f"depth     {d['nav'][i, 2]:7.2f} m",
                 f"heading   {math.degrees(yaw) % 360:7.1f} deg",
                 f"speed     {d['speed'][i]:7.2f} m/s",
                 f"DVL       {'valid' if d['valid'][i] else 'INVALID / gated'}",
                 f"dvl_age   {d['nav'][i, 3]:7.1f} s",
                 f"pos 1-sig {d['nav'][i, 4]:7.2f} m",
                 f"log {self.idx + 1}/{len(self.items)}  NaN {d['nan']}  exc {d['exc']}",
                 "space | <- -> log | up/down speed | r"]
        self.ax_txt.text(0, 1, "\n".join(lines), va="top", family="monospace", fontsize=10,
                         color="black" if d["valid"][i] else "#b00020")
        return [self.hull, self.fins, self.trail, self.dtrail, self.dmark]

    def step(self, _):
        d = self.d
        if not self.paused:
            t_now = d["t"][self.frame] + (1 / 30) * PLAYBACK[self.speed_i]
            self.frame = min(int(np.searchsorted(d["t"], t_now)), len(d["t"]) - 1)
            if self.frame >= len(d["t"]) - 1:
                self.paused = True
        return self.update(self.frame)

    def on_key(self, ev):
        k = ev.key
        if k == " ":
            self.paused = not self.paused
        elif k == "right":
            self.load(self.idx + 1)
        elif k == "left":
            self.load(self.idx - 1)
        elif k == "up":
            self.speed_i = min(self.speed_i + 1, len(PLAYBACK) - 1)
        elif k == "down":
            self.speed_i = max(self.speed_i - 1, 0)
        elif k == "r":
            self.frame, self.paused = 0, False
        self.fig.canvas.draw_idle()


def pick(paths, spec):
    if spec is None:
        return 0
    if spec.isdigit():
        return int(spec) % len(paths)
    for i, p in enumerate(paths):
        if spec in os.path.basename(p):
            return i
    raise SystemExit(f"no log matches {spec!r}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", help="index or filename substring")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--save", help="write an animated GIF headless instead of opening a window")
    ap.add_argument("--frames", type=int, default=120)
    args = ap.parse_args()
    paths = list_logs()
    if args.list:
        for i, p in enumerate(paths):
            print(i, os.path.basename(p))
        return
    if args.save:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.animation as animation
    v = Viewer(log_items(paths), pick(paths, args.log))
    if args.save:
        idxs = np.linspace(0, len(v.d["t"]) - 1, args.frames).astype(int)
        anim = animation.FuncAnimation(v.fig, lambda k: v.update(idxs[k]), frames=len(idxs))
        anim.save(args.save, writer="pillow", fps=15)
        print("saved", args.save)
        return
    v.anim = animation.FuncAnimation(v.fig, v.step, interval=33, blit=False, cache_frame_data=False)
    v.plt.show()


if __name__ == "__main__":
    main()
