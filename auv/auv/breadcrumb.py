"""Breadcrumb trail for abort return. Ring buffer, ~2 Hz, thinned."""
from __future__ import annotations
import math
from collections import deque
from .types import Waypoint, CRUISE_SPEED, wrap_pi


def _rdp_keep(pts, tol):
    """Iterative Ramer-Douglas-Peucker on [(n,e),...]; returns kept indices."""
    n = len(pts)
    if n <= 2:
        return list(range(n))
    keep = [False] * n
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        (an, ae), (bn, be) = pts[a], pts[b]
        dn, de = bn - an, be - ae
        L = math.hypot(dn, de)
        best, bi = -1.0, -1
        for i in range(a + 1, b):
            pn, pe = pts[i]
            if L < 1e-9:
                d = math.hypot(pn - an, pe - ae)
            else:
                d = abs(dn * (pe - ae) - de * (pn - an)) / L
            if d > best:
                best, bi = d, i
        if best > tol:
            keep[bi] = True
            stack.append((a, bi))
            stack.append((bi, b))
    return [i for i, k in enumerate(keep) if k]


class BreadcrumbTrail:
    def __init__(self, maxlen: int = 4000, min_dt: float = 0.5,
                 min_dist: float = 1.0, min_dyaw: float = math.radians(2.0)):
        self.buf = deque(maxlen=maxlen)   # (t,n,e,d,yaw,speed)
        self.min_dt = min_dt
        self.min_dist = min_dist
        self.min_dyaw = min_dyaw

    def __len__(self):
        return len(self.buf)

    def record(self, t, n, e, d, yaw, speed) -> bool:
        if self.buf:
            lt, ln, le, _, ly, _ = self.buf[-1]
            if t - lt < self.min_dt:
                return False
            if (math.hypot(n - ln, e - le) < self.min_dist
                    and abs(wrap_pi(yaw - ly)) < self.min_dyaw):
                return False
        self.buf.append((t, n, e, d, yaw, speed))
        return True

    def _simplified(self, tol):
        pts = [(p[1], p[2]) for p in self.buf]
        idx = _rdp_keep(pts, tol)
        return [self.buf[i] for i in idx]

    def positional_return(self, tolerance: float = 2.0, timeout_s: float = 300.0):
        """Waypoints retracing the trail, newest->oldest (current position excluded)."""
        if len(self.buf) < 2:
            return []
        simp = self._simplified(tolerance)
        return [Waypoint(p[1], p[2], p[3], timeout_s) for p in reversed(simp[:-1])]

    def speed_estimate(self) -> float:
        sp = sorted(p[5] for p in self.buf if p[5] > 0.1)
        return sp[len(sp) // 2] if sp else CRUISE_SPEED

    def heading_return(self, tolerance: float = 2.0, speed: float = None):
        """[(heading_rad, duration_s)] reversed legs (heading + 180 deg) for DVL-dead return."""
        if len(self.buf) < 2:
            return []
        v = speed if speed and speed > 0.05 else self.speed_estimate()
        simp = self._simplified(tolerance)
        legs = []
        for a, b in zip(simp[:-1], simp[1:]):
            dn, de = b[1] - a[1], b[2] - a[2]
            dist = math.hypot(dn, de)
            if dist < 1e-6:
                continue
            legs.append((wrap_pi(math.atan2(de, dn) + math.pi), dist / v))
        legs.reverse()
        return legs
