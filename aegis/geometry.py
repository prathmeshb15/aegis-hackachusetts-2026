"""Small 2D geometry helpers shared by the simulator, planner and supervisor.

Shapes are plain tuples so hot loops stay cheap:
- circle: (cx, cy, r)
- rect:   (xmin, ymin, xmax, ymax)  axis-aligned
"""
from __future__ import annotations

from math import hypot, sqrt


def circle_circle_gap(ax: float, ay: float, ar: float, bx: float, by: float, br: float) -> float:
    """Signed surface-to-surface distance between two disks (negative = overlap)."""
    return hypot(ax - bx, ay - by) - ar - br


def point_rect_distance(px: float, py: float, rect: tuple) -> float:
    """Euclidean distance from a point to an axis-aligned rectangle (0 if inside)."""
    xmin, ymin, xmax, ymax = rect
    dx = max(xmin - px, 0.0, px - xmax)
    dy = max(ymin - py, 0.0, py - ymax)
    return sqrt(dx * dx + dy * dy)


def circle_rect_gap(px: float, py: float, r: float, rect: tuple) -> float:
    """Signed gap between a disk and a rectangle. Inside the rectangle counts as -r."""
    return point_rect_distance(px, py, rect) - r


def bounds_gap(px: float, py: float, r: float, width: float, height: float) -> float:
    """Gap between a disk and the inside walls of the field [0,width]x[0,height]."""
    return min(px, py, width - px, height - py) - r


def segment_intersects_rect(x0: float, y0: float, x1: float, y1: float, rect: tuple) -> bool:
    """Liang-Barsky clip test: does segment (x0,y0)-(x1,y1) cross the rectangle?"""
    xmin, ymin, xmax, ymax = rect
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 - xmin), (dx, xmax - x0), (-dy, y0 - ymin), (dy, ymax - y0)):
        if abs(p) < 1e-12:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            if t > t1:
                return False
            t0 = max(t0, t)
        else:
            if t < t0:
                return False
            t1 = min(t1, t)
    return t0 <= t1
