"""AUV Studio: one window for everything (no hardware needed).

    .venv/bin/python -m sim.studio                  # open the window
    .venv/bin/python -m sim.studio --shot code out.png   # headless screenshot of a page
    .venv/bin/python -m sim.studio --diagram docs/ALGO.png

Pages (keys 1-4 or buttons):
  1 Animate   AUV moving along a path. Sources: latest SITL log (sim/out/sitl_*.csv) and the field
              logs in logs/ replayed through NavFilter. space pause | <- -> source | up/down speed | r
  2 Results   Pass/fail table of the last SITL run (sim/out/results.json)
  3 Algorithm Block diagram of the control stack
  4 Code      Browse the algorithm source files. <- -> file | up/down scroll | pageup/pagedown
Key s (or the button) runs the SITL: writes results.json, plots and ONE sitl_<timestamp>.csv log.
"""
from __future__ import annotations
import argparse
import glob
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, "out")
PAGES = ("animate", "results", "algorithm", "code")
CONTENT = (0.0, 0.09, 1.0, 0.91)       # figure-fraction rect above the button row


def draw_diagram(ax):
    """Control stack block diagram. Boxes are (x, y, w, h, title, body, colour)."""
    from matplotlib.patches import FancyBboxPatch
    ax.set_xlim(0, 104)
    ax.set_ylim(0, 81)
    ax.axis("off")
    B = {
        "sens": (2, 62, 26, 12, "SENSORS", "DVL A50 | IMU + compass | Bar100\nGPS (surface only) | battery | leak", "#e8f1fb"),
        "nav": (34, 62, 32, 12, "NAV FILTER  (EKF, 10 states)",
                "DVL gate: valid and FOM < 0.5\nGPS gate: chi-square 2-DOF d2 <= 13.82\nfix applied as measurement, no teleport", "#fff4d6"),
        "fs": (72, 62, 26, 12, "FAILSAFE LADDER  H0-H8",
               "DVL dead > 5 s | attitude | depth | leak\nbattery | current > 0.3 | heartbeat", "#fde2e0"),
        "sk": (2, 42, 26, 12, "STATION KEEPER", "hold into current | loiter | float\nused in FIX / SYNC / end of mission", "#e6f6ea"),
        "guid": (34, 42, 32, 12, "GUIDANCE  (LOS + integral)",
                 "psi_d = leg bearing - atan((e + k*int(e))/delta)\ncurrent crab feed-forward, capture radius", "#e6f6ea"),
        "mm": (72, 42, 26, 12, "MISSION FSM", "WAIT_GPS > ARM > DIVE > RUN\n> RETURN / ABORT > SURFACE > FIX > DONE", "#ece6f6"),
        "dep": (2, 24, 26, 12, "DEPTH PID", "depth error > pitch (+-20 deg)\nintegral trim, D from vertical speed", "#e6f6ea"),
        "hd": (34, 24, 32, 12, "HEADING + THRUST", "yaw command (rate limited)\nspeed PI on water speed, floor 0.25 m/s", "#e6f6ea"),
        "bc": (72, 24, 26, 12, "BREADCRUMB RETURN", "positional (RDP) if DVL alive\nreversed headings if DVL dead", "#ece6f6"),
        "cmd": (2, 4, 30, 12, "ATTITUDE CMD  >  MAVLINK", "SET_ATTITUDE_TARGET + thrust\n1 Hz GCS heartbeat, SURFACE on abort", "#e8f1fb"),
        "px": (38, 4, 30, 12, "PIXHAWK 2.4.8", "attitude PID + Lua fin mixer (50 Hz)\n250 ms dead-man: fins to trim", "#fff4d6"),
        "veh": (74, 4, 24, 12, "VEHICLE", "T200 + 4 fins\nreal hardware or SITL plant", "#e8f1fb"),
    }
    for x, y, w, h, title, body, col in B.values():
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.3,rounding_size=1.2",
                                    fc=col, ec="#445", lw=1.2))
        ax.text(x + w / 2, y + h - 2.2, title, ha="center", va="center", fontsize=9, weight="bold")
        ax.text(x + w / 2, y + h / 2 - 1.8, body, ha="center", va="center", fontsize=7)

    def c(k, side):
        x, y, w, h = B[k][:4]
        return {"l": (x, y + h / 2), "r": (x + w, y + h / 2), "t": (x + w / 2, y + h),
                "b": (x + w / 2, y)}[side]

    def arrow(a, b, rad=0.0, txt=None, col="#334"):
        ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="-|>", color=col, lw=1.3,
                                                         connectionstyle=f"arc3,rad={rad}"))
        if txt:
            ax.text((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + 1.2, txt, fontsize=6.5, ha="center", color=col)

    def route(pts, end, txt, txt_xy, col="#334"):
        """Right-angle path along pts, then an arrow into `end`."""
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=col, lw=1.3)
        ax.annotate("", xy=end, xytext=pts[-1], arrowprops=dict(arrowstyle="-|>", color=col, lw=1.3))
        ax.text(*txt_xy, txt, fontsize=6.5, ha="center", color=col)

    arrow(c("sens", "r"), c("nav", "l"))
    arrow(c("nav", "r"), c("fs", "l"), txt="state + health")
    arrow(c("fs", "b"), c("mm", "t"), txt="action")
    arrow(c("nav", "b"), c("guid", "t"), txt="pos, vel")
    arrow(c("mm", "l"), c("guid", "r"), txt="waypoints")
    arrow(c("mm", "b"), c("bc", "t"))
    arrow(c("bc", "l"), c("hd", "r"), txt="return legs")
    route([(72, 52), (69, 52), (69, 57.5), (15, 57.5)], c("sk", "t"), "hold target", (42, 58.3))
    arrow(c("guid", "b"), c("hd", "t"), txt="psi_d")
    arrow(c("sk", "b"), c("dep", "t"))
    arrow(c("dep", "b"), c("cmd", "t"), txt="pitch")
    arrow(c("hd", "b"), c("cmd", "r"), rad=0.2, txt="yaw, thrust")
    arrow(c("cmd", "r"), c("px", "l"))
    arrow(c("px", "r"), c("veh", "l"))
    route([(98, 10), (101, 10), (101, 77), (15, 77)], c("sens", "t"), "measurements (feedback)",
          (58, 78), col="#b00020")
    ax.set_title("AUV control stack (Pi: outer loops, Pixhawk: inner loop). Red = feedback", fontsize=10)


def code_files():
    out = []
    for d in ("auv", "sim"):
        out += sorted(p for p in glob.glob(os.path.join(ROOT, d, "*.py")) if not p.endswith("__init__.py"))
    return out


class Studio:
    def __init__(self, page="animate"):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button
        from sim.viewer import Viewer, log_items, sitl_items
        from sim.replay import list_logs
        self.plt = plt
        self.fig = plt.figure(figsize=(13, 8))
        self.page, self.page_axes = None, []
        self._log_items = log_items(list_logs())
        self.viewer = Viewer(self.items(sitl_items), 0, fig=self.fig, bind_keys=False)
        for ax in self.viewer.axes:          # Viewer builds axes on creation; pages add them on demand
            self.fig.delaxes(ax)
        self.files, self.file_i, self.scroll = code_files(), 0, 0
        self.buttons = []
        names = ["1 Animate", "2 Results", "3 Algorithm", "4 Code", "s Run SITL"]
        for i, nme in enumerate(names):
            ax = self.fig.add_axes([0.02 + i * 0.125, 0.015, 0.115, 0.05])
            b = Button(ax, nme)
            b.on_clicked((lambda _e, i=i: self.show(PAGES[i]) if i < 4 else self.run_sitl()))
            self.buttons.append(b)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.show(page)

    def items(self, sitl_items):
        sitl = sorted(glob.glob(os.path.join(OUT, "sitl_*.csv")))
        return (sitl_items(sitl[-1]) if sitl else []) + self._log_items

    def clear(self):
        for ax in self.page_axes:
            if ax in self.fig.axes:
                self.fig.delaxes(ax)
        self.page_axes = []

    def show(self, page):
        self.clear()
        self.page = page
        if page == "animate":
            idx = self.viewer.idx
            self.viewer.build(CONTENT)
            self.page_axes = list(self.viewer.axes)
            self.viewer.load(idx)
        elif page == "results":
            self.page_axes = [self.fig.add_axes([0.03, 0.1, 0.94, 0.85])]
            self.draw_results(self.page_axes[0])
        elif page == "algorithm":
            self.page_axes = [self.fig.add_axes([0.02, 0.1, 0.96, 0.86])]
            draw_diagram(self.page_axes[0])
        else:
            self.page_axes = [self.fig.add_axes([0.02, 0.1, 0.96, 0.87])]
            self.draw_code(self.page_axes[0])
        self.fig.canvas.draw_idle()

    def draw_results(self, ax):
        ax.axis("off")
        path = os.path.join(OUT, "results.json")
        if not os.path.exists(path):
            ax.text(0.5, 0.5, "No results yet. Press s to run the SITL.", ha="center")
            return
        rows = json.load(open(path))
        f = lambda v: "-" if v is None else f"{v:.2f}"
        cells = [[r["scenario"], "PASS" if r.get("pass") else "FAIL", f(r.get("cross_track_rms_m")),
                  f(r.get("depth_rms_m")), f(r.get("closure_error_to_last_wp_m")),
                  f(r.get("max_depth_m")), r.get("max_fail_level", ""), r.get("final_state", ""),
                  "yes" if r.get("surfaced") else "no", ", ".join(r.get("why_failed", []))]
                 for r in rows if "error" not in r]
        cols = ["scenario", "result", "xtrack RMS m", "depth RMS m", "closure m", "max depth m",
                "max fail", "final", "surfaced", "why failed"]
        tb = ax.table(cellText=cells, colLabels=cols, loc="upper center", cellLoc="center")
        tb.auto_set_font_size(False)
        tb.set_fontsize(9)
        tb.scale(1, 1.5)
        for i, r in enumerate(cells, start=1):
            tb[i, 1].set_facecolor("#c8efd4" if r[1] == "PASS" else "#f7c6c1")
            for j, lim in ((2, 2.0), (3, 0.3)):
                if r[j] != "-" and float(r[j]) >= lim:
                    tb[i, j].set_facecolor("#fff1c2")
        ax.set_title("Last SITL run (yellow = over target: xtrack < 2 m, depth < 0.3 m)", fontsize=10)

    def draw_code(self, ax):
        ax.axis("off")
        path = self.files[self.file_i]
        lines = open(path).read().splitlines()
        n = 46
        self.scroll = max(0, min(self.scroll, max(len(lines) - n, 0)))
        shown = lines[self.scroll:self.scroll + n]
        body = "\n".join(f"{self.scroll + i + 1:4d}  {ln[:150]}" for i, ln in enumerate(shown))
        ax.text(0, 1, f"{os.path.relpath(path, ROOT)}   [{self.file_i + 1}/{len(self.files)}]   "
                      f"lines {self.scroll + 1}-{self.scroll + len(shown)} of {len(lines)}",
                va="top", fontsize=10, weight="bold", transform=ax.transAxes)
        ax.text(0, 0.97, body, va="top", family="monospace", fontsize=7.2, transform=ax.transAxes)

    def run_sitl(self):
        from sim import run_sitl
        from sim.viewer import sitl_items
        print("running SITL ...")
        run_sitl.main([])
        self.viewer.items = self.items(sitl_items)
        self.viewer.idx = 0
        self.show(self.page)

    def on_key(self, ev):
        k = ev.key
        if k in ("1", "2", "3", "4"):
            return self.show(PAGES[int(k) - 1])
        if k == "s":
            return self.run_sitl()
        if self.page == "animate":
            self.viewer.on_key(ev)
        elif self.page == "code":
            if k in ("right", "left"):
                self.file_i = (self.file_i + (1 if k == "right" else -1)) % len(self.files)
                self.scroll = 0
            elif k in ("down", "up"):
                self.scroll += 10 if k == "down" else -10
            elif k in ("pagedown", "pageup"):
                self.scroll += 40 if k == "pagedown" else -40
            else:
                return
            self.clear()
            self.page_axes = [self.fig.add_axes([0.02, 0.1, 0.96, 0.87])]
            self.draw_code(self.page_axes[0])
            self.fig.canvas.draw_idle()

    def tick(self, _):
        if self.page == "animate":
            self.viewer.step(_)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--page", choices=PAGES, default="animate")
    ap.add_argument("--shot", nargs=2, metavar=("PAGE", "PNG"), help="headless screenshot of a page")
    ap.add_argument("--diagram", metavar="PNG", help="write the algorithm diagram and exit")
    args = ap.parse_args()
    import matplotlib
    if args.shot or args.diagram:
        matplotlib.use("Agg")
    if args.diagram:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(13, 8))
        draw_diagram(ax)
        fig.savefig(args.diagram, dpi=110)
        print("saved", args.diagram)
        return
    import matplotlib.animation as animation
    s = Studio(args.shot[0] if args.shot else args.page)
    if args.shot:
        if args.shot[0] == "animate":
            s.viewer.frame = len(s.viewer.d["t"]) // 2
            s.viewer.update(s.viewer.frame)
        s.fig.savefig(args.shot[1], dpi=80)
        print("saved", args.shot[1])
        return
    s.anim = animation.FuncAnimation(s.fig, s.tick, interval=33, blit=False, cache_frame_data=False)
    s.plt.show()


if __name__ == "__main__":
    main()
