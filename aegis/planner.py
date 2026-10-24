"""Collision-aware global path planner: A* on an inflated occupancy grid.

The grid is built from the planner's *known* obstacles (mapped shelves plus any
obstacles the supervisor has detected). Each cell is blocked when its centre is
closer than ``inflation`` metres to an obstacle, so a disk robot of radius
``inflation - margin`` following the path keeps at least ``margin`` clearance
at cell centres. The raw 8-connected A* path is then shortcut-smoothed with an
exact geometric line-of-sight check against the same inflated shapes.
"""
from __future__ import annotations

import heapq
from math import hypot, sqrt

from .geometry import point_rect_distance

SQRT2 = sqrt(2.0)


class GridPlanner:
    def __init__(self, width: float, height: float, resolution: float = 0.2, inflation: float = 0.45):
        self.width = width
        self.height = height
        self.res = resolution
        self.inflation = inflation
        self.nx = int(round(width / resolution))
        self.ny = int(round(height / resolution))

    # ------------------------------------------------------------------ geometry
    def clearance(self, x: float, y: float, rects, circles) -> float:
        """Distance from point to the nearest obstacle surface or field wall."""
        best = min(x, y, self.width - x, self.height - y)
        for rect in rects:
            d = point_rect_distance(x, y, rect)
            if d < best:
                best = d
        for cx, cy, r in circles:
            d = hypot(x - cx, y - cy) - r
            if d < best:
                best = d
        return best

    def _cell_center(self, i: int, j: int):
        return ((i + 0.5) * self.res, (j + 0.5) * self.res)

    def _cell_of(self, x: float, y: float):
        i = min(self.nx - 1, max(0, int(x / self.res)))
        j = min(self.ny - 1, max(0, int(y / self.res)))
        return i, j

    def occupancy(self, rects, circles):
        occ = [[False] * self.ny for _ in range(self.nx)]
        for i in range(self.nx):
            for j in range(self.ny):
                x, y = self._cell_center(i, j)
                occ[i][j] = self.clearance(x, y, rects, circles) < self.inflation
        return occ

    def segment_free(self, a, b, rects, circles, step: float = 0.05) -> bool:
        n = max(1, int(hypot(b[0] - a[0], b[1] - a[1]) / step))
        for k in range(n + 1):
            t = k / n
            if self.clearance(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, rects, circles) < self.inflation:
                return False
        return True

    # ------------------------------------------------------------------ search
    def plan(self, start, goal, rects, circles):
        """Return a smoothed list of (x, y) waypoints from start to goal, or None.

        ``start`` may lie inside an inflated region (e.g. after an evasive move);
        cells within 0.7 m of the start are treated as passable so the robot can
        leave that region, but the goal must be in free space.
        """
        occ = self.occupancy(rects, circles)
        si, sj = self._cell_of(*start)
        gi, gj = self._cell_of(*goal)
        if occ[gi][gj]:
            return None

        def passable(i, j):
            if not occ[i][j]:
                return True
            cx, cy = self._cell_center(i, j)
            return hypot(cx - start[0], cy - start[1]) < 0.7

        def h(i, j):
            dx, dy = abs(i - gi), abs(j - gj)
            return (dx + dy) + (SQRT2 - 2) * min(dx, dy)

        g = {(si, sj): 0.0}
        parent = {(si, sj): None}
        heap = [(h(si, sj), 0.0, si, sj)]
        closed = set()
        found = False
        while heap:
            _, gc, i, j = heapq.heappop(heap)
            if (i, j) in closed:
                continue
            closed.add((i, j))
            if (i, j) == (gi, gj):
                found = True
                break
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    ni, nj = i + di, j + dj
                    if not (0 <= ni < self.nx and 0 <= nj < self.ny) or not passable(ni, nj):
                        continue
                    # No diagonal corner-cutting through blocked cells.
                    if di and dj and (not passable(i + di, j) or not passable(i, j + dj)):
                        continue
                    ng = gc + (SQRT2 if di and dj else 1.0)
                    if ng < g.get((ni, nj), float('inf')):
                        g[(ni, nj)] = ng
                        parent[(ni, nj)] = (i, j)
                        heapq.heappush(heap, (ng + h(ni, nj), ng, ni, nj))
        if not found:
            return None

        cells = []
        node = (gi, gj)
        while node is not None:
            cells.append(node)
            node = parent[node]
        cells.reverse()
        raw = [tuple(start)] + [self._cell_center(i, j) for i, j in cells[1:-1]] + [tuple(goal)]
        return self._shortcut(raw, rects, circles)

    def _shortcut(self, pts, rects, circles):
        """Greedy line-of-sight smoothing; keeps only waypoints that are needed."""
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.segment_free(pts[i], pts[j], rects, circles):
                j -= 1
            out.append(pts[j])
            i = j
        return [(round(x, 3), round(y, 3)) for x, y in out]


def path_length(path) -> float:
    return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
