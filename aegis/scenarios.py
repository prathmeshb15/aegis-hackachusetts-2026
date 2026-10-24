"""Seeded warehouse scenario generator.

Every scenario is a pure function of ``(scenario_type, seed)``. The same
``Scenario`` object (layout, unmapped obstacles, scripted mover trajectories,
sensor noise level) is used for the baseline and the AEGIS run, so a paired
comparison differs *only* in the controller.

Movers are open-loop scripted: they follow time-parameterised polylines and do
not react to the robot. That is deliberately adversarial (real people usually
yield) and keeps trajectories identical across paired runs.

Adversarial timing: most dynamic types aim the mover at the point where the
*nominal* robot would be, using a straight-line constant-speed estimate of the
robot schedule plus a seeded jitter. This is disclosed as adversarial design,
not a model of typical warehouse traffic.
"""
from __future__ import annotations

import json
import random
from dataclasses import dataclass, field
from math import hypot

from .constants import A_MAX, FIELD_H, FIELD_W, V_MAX

AISLE_LO = 2.6           # aisle spans y in [AISLE_LO, AISLE_HI]; shelves outside
AISLE_HI = 7.4
GAP_HALF = 0.75          # half-width of each cross-aisle gap
PED_R = 0.30
FORKLIFT_R = 0.50


@dataclass
class Mover:
    id: int
    kind: str
    radius: float
    waypoints: list  # [(t, x, y), ...] strictly non-decreasing t

    def active(self, t: float) -> bool:
        return self.waypoints[0][0] <= t <= self.waypoints[-1][0]

    def pos(self, t: float):
        wp = self.waypoints
        if t <= wp[0][0]:
            return wp[0][1], wp[0][2]
        for (t0, x0, y0), (t1, x1, y1) in zip(wp, wp[1:]):
            if t <= t1:
                if t1 - t0 < 1e-9:
                    return x1, y1
                a = (t - t0) / (t1 - t0)
                return x0 + (x1 - x0) * a, y0 + (y1 - y0) * a
        return wp[-1][1], wp[-1][2]

    def vel(self, t: float):
        wp = self.waypoints
        for (t0, x0, y0), (t1, x1, y1) in zip(wp, wp[1:]):
            if t0 <= t < t1 and t1 - t0 > 1e-9:
                return (x1 - x0) / (t1 - t0), (y1 - y0) / (t1 - t0)
        return 0.0, 0.0


@dataclass
class Scenario:
    type: str
    seed: int
    category: str
    description: str
    start: tuple
    goal: tuple
    shelves: list                    # mapped rectangles, known to both controllers
    statics: list                    # unmapped circles (x, y, r), unknown a priori
    movers: list
    sensor_sigma: float = 0.0
    params: dict = field(default_factory=dict)

    def signature(self) -> str:
        """Canonical JSON of the environment, used to prove paired runs match."""
        return json.dumps({
            'type': self.type, 'seed': self.seed, 'start': self.start, 'goal': self.goal,
            'shelves': self.shelves, 'statics': self.statics, 'sigma': self.sensor_sigma,
            'movers': [(m.kind, m.radius, m.waypoints) for m in self.movers],
        }, sort_keys=True)

    def to_json(self):
        return {
            'type': self.type, 'seed': self.seed, 'category': self.category,
            'description': self.description, 'start': list(self.start), 'goal': list(self.goal),
            'shelves': [list(r) for r in self.shelves], 'field': [FIELD_W, FIELD_H],
            'statics': [list(c) for c in self.statics],
            'sensor_sigma': self.sensor_sigma, 'params': self.params,
        }


# --------------------------------------------------------------------------- helpers
def _r(v, nd=3):
    return round(v, nd)


def _layout(rng):
    gx1 = rng.uniform(3.2, 5.0)
    gx2 = rng.uniform(7.0, 8.8)
    xs = [(0.0, gx1 - GAP_HALF), (gx1 + GAP_HALF, gx2 - GAP_HALF), (gx2 + GAP_HALF, FIELD_W)]
    shelves = []
    for a, b in xs:
        shelves.append((_r(a), 0.0, _r(b), AISLE_LO))
        shelves.append((_r(a), AISLE_HI, _r(b), FIELD_H))
    return [_r(gx1), _r(gx2)], shelves


def _route(points, speed, anchor_idx, anchor_time, dwell=None):
    """Time-parameterise a polyline so that ``points[anchor_idx]`` is reached at
    ``anchor_time``. ``dwell`` maps a point index to a pause duration."""
    dwell = dwell or {}
    times = [0.0]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        times.append(times[-1] + hypot(x1 - x0, y1 - y0) / speed)
    offset = anchor_time - times[anchor_idx]
    wps, extra = [], 0.0
    for k, ((x, y), t) in enumerate(zip(points, times)):
        wps.append((_r(t + offset + extra), _r(x), _r(y)))
        if k in dwell:
            extra += dwell[k]
            wps.append((_r(t + offset + extra), _r(x), _r(y)))
    return wps


class _Ctx:
    """Per-scenario seeded parameters shared by the type generators."""

    def __init__(self, stype, seed):
        self.rng = random.Random(f'aegis:{stype}:{seed}')
        rng = self.rng
        self.gaps, self.shelves = _layout(rng)
        self.start = (1.0, _r(rng.uniform(3.6, 6.4)))
        self.goal = (11.0, _r(rng.uniform(3.6, 6.4)))
        self.movers = []
        self.statics = []
        self.params = {'gaps': self.gaps}

    def lane_y(self, x):
        sx, sy = self.start
        gx, gy = self.goal
        return sy + (gy - sy) * (x - sx) / (gx - sx)

    def t_arrive(self, x):
        """Rough nominal robot arrival time at abscissa x (straight line, cruise speed)."""
        return (x - self.start[0]) / V_MAX + V_MAX / (2 * A_MAX)

    def add(self, kind, radius, wps):
        self.movers.append(Mover(len(self.movers), kind, radius, wps))

    def crossing(self, speed, jitter, gap=None, upward=None, dwell=0.0, kind='pedestrian', radius=PED_R):
        rng = self.rng
        gx = self.gaps[rng.randrange(2)] if gap is None else gap
        up = (rng.random() < 0.5) if upward is None else upward
        ly = self.lane_y(gx)
        y0, y1 = (0.3, FIELD_H - 0.3) if up else (FIELD_H - 0.3, 0.3)
        arrive = self.t_arrive(gx) + jitter
        self.add(kind, radius, _route([(gx, y0), (gx, ly), (gx, y1)], speed, 1, arrive,
                                      dwell={1: dwell} if dwell else None))
        return gx, up, arrive


# --------------------------------------------------------------------------- types
def _crossing(c):
    sp, jit = c.rng.uniform(0.7, 1.5), c.rng.uniform(-1.2, 1.2)
    gx, up, _ = c.crossing(sp, jit)
    c.params.update(speed=_r(sp), jitter=_r(jit), crossing_x=gx, upward=up)


def _crossing_fast(c):
    sp, jit = c.rng.uniform(1.6, 2.2), c.rng.uniform(-0.5, 0.5)
    gx, up, _ = c.crossing(sp, jit)
    c.params.update(speed=_r(sp), jitter=_r(jit), crossing_x=gx, upward=up)


def _slow_crosser(c):
    sp, jit = c.rng.uniform(0.3, 0.5), c.rng.uniform(-0.8, 0.8)
    gx, up, _ = c.crossing(sp, jit)
    c.params.update(speed=_r(sp), jitter=_r(jit), crossing_x=gx, upward=up)


def _diagonal(c):
    rng = c.rng
    sp, jit = rng.uniform(0.8, 1.4), rng.uniform(-1.0, 1.0)
    ga, gb = (c.gaps if rng.random() < 0.5 else c.gaps[::-1])
    up = rng.random() < 0.5
    lo, hi = (AISLE_LO, AISLE_HI) if up else (AISLE_HI, AISLE_LO)
    ly = c.lane_y((ga + gb) / 2)
    frac = (ly - lo) / (hi - lo)
    xc = ga + (gb - ga) * frac
    pts = [(ga, 0.3 if up else FIELD_H - 0.3), (ga, lo), (xc, ly), (gb, hi), (gb, FIELD_H - 0.3 if up else 0.3)]
    c.add('pedestrian', PED_R, _route(pts, sp, 2, c.t_arrive(xc) + jit))
    c.params.update(speed=_r(sp), jitter=_r(jit), from_gap=ga, to_gap=gb)


def _head_on(c):
    rng = c.rng
    sp, t0 = rng.uniform(0.6, 1.2), rng.uniform(0.0, 1.5)
    a = (11.6, c.goal[1] + rng.uniform(-0.3, 0.3))
    b = (0.4, c.start[1] + rng.uniform(-0.3, 0.3))
    c.add('pedestrian', PED_R, _route([a, b], sp, 0, t0))
    c.params.update(speed=_r(sp), start_time=_r(t0))


def _overtaker(c):
    rng = c.rng
    # Start window 1.5-3.0 s: earlier starts spawned the walker ~0.2 m behind the
    # robot (a spawn artefact rather than an overtaking test; see 'ambush').
    sp, t0 = rng.uniform(2.0, 2.5), rng.uniform(1.5, 3.0)
    a = (0.35, c.start[1] + rng.uniform(-0.2, 0.2))
    b = (11.7, c.goal[1] + rng.uniform(-0.2, 0.2))
    c.add('pedestrian', PED_R, _route([a, b], sp, 0, t0))
    c.params.update(speed=_r(sp), start_time=_r(t0))


def _stop_and_go(c):
    rng = c.rng
    sp, jit, dw = rng.uniform(0.8, 1.3), rng.uniform(-1.5, 0.0), rng.uniform(1.5, 3.0)
    gx, up, _ = c.crossing(sp, jit, dwell=dw)
    c.params.update(speed=_r(sp), jitter=_r(jit), dwell=_r(dw), crossing_x=gx, upward=up)


def _multi(c):
    rng = c.rng
    n = rng.randint(2, 4)
    for _ in range(n):
        c.crossing(rng.uniform(0.6, 1.6), rng.uniform(-2.0, 2.0))
    c.params.update(pedestrians=n)


def _blocked(c):
    rng = c.rng
    x = rng.uniform(4.0, 8.0)
    r = rng.uniform(0.45, 0.8)
    y = c.lane_y(x) + rng.uniform(-0.3, 0.3)
    c.statics.append((_r(x), _r(y), _r(r)))
    c.params.update(pallet=[_r(x), _r(y), _r(r)])


def _chicane(c):
    rng = c.rng
    s = 1 if rng.random() < 0.5 else -1
    x1 = rng.uniform(3.8, 5.5)
    x2 = x1 + rng.uniform(1.8, 2.6)
    for x, sign in ((x1, s), (x2, -s)):
        r = rng.uniform(0.45, 0.65)
        y = c.lane_y(x) + sign * rng.uniform(0.4, 0.8)
        y = min(AISLE_HI - r, max(AISLE_LO + r, y))
        c.statics.append((_r(x), _r(y), _r(r)))
    c.params.update(pallets=len(c.statics))


def _dead_end(c):
    xw = (c.gaps[0] + c.gaps[1]) / 2
    y = 2.9
    while y <= 7.11:
        c.statics.append((_r(xw), _r(y), 0.6))
        y += 0.84
    c.params.update(wall_x=_r(xw))


def _noisy(c):
    rng = c.rng
    sp, jit, sig = rng.uniform(0.7, 1.5), rng.uniform(-1.2, 1.2), rng.uniform(0.15, 0.35)
    gx, up, _ = c.crossing(sp, jit)
    c.params.update(speed=_r(sp), jitter=_r(jit), sensor_sigma=_r(sig), crossing_x=gx)
    return sig


def _forklift(c):
    rng = c.rng
    sp, jit = rng.uniform(2.6, 3.2), rng.uniform(-0.15, 0.15)
    gx = c.gaps[rng.randrange(2)]
    up = c.lane_y(gx) < (AISLE_LO + AISLE_HI) / 2   # emerge from the nearer shelf row
    c.crossing(sp, jit, gap=gx, upward=up, kind='forklift', radius=FORKLIFT_R)
    c.params.update(speed=_r(sp), jitter=_r(jit), crossing_x=gx)


def _ambush(c):
    """Physically unavoidable by construction: a forklift appears 1.0 m (centre to
    centre, ~0.22 m surface gap) from the robot's nominal position, moving straight
    at it at 3 m/s. Contact happens within ~0.1 s, while the robot can change its
    velocity by at most A_MAX*0.1 = 0.2 m/s. Models a blind door / sensing failure
    and exists to check that collisions are reported honestly."""
    rng = c.rng
    t_hit = rng.uniform(3.0, 5.0)
    x = c.start[0] + V_MAX * (t_hit - V_MAX / (2 * A_MAX))
    y = c.lane_y(x)
    side = -1 if y < (AISLE_LO + AISLE_HI) / 2 else 1
    sp, d0 = 3.0, 1.0
    p0 = (x, y + side * d0)
    p1 = (x, y - side * 3.0)
    c.add('forklift', FORKLIFT_R, _route([p0, p1], sp, 0, t_hit - 0.05))
    c.params.update(appear_time=_r(t_hit - 0.05), aim=[_r(x), _r(y)])


def _loiter(c):
    rng = c.rng
    gx = c.gaps[rng.randrange(2)]
    up = rng.random() < 0.5
    ly = c.lane_y(gx) + rng.uniform(-0.2, 0.2)
    sp, arrive = rng.uniform(0.8, 1.2), rng.uniform(0.0, 2.0)
    y0 = 0.3 if up else FIELD_H - 0.3
    c.add('pedestrian', PED_R, _route([(gx, y0), (gx, ly), (gx, ly)], sp, 1, arrive, dwell={1: 60.0}))
    c.params.update(stand_at=[gx, _r(ly)], arrive=_r(arrive))


CATALOG = {
    # type: (category, description, generator)
    'crossing': ('dynamic', 'Pedestrian crosses the aisle from an occluded cross-aisle gap', _crossing),
    'crossing_fast': ('dynamic', 'Fast pedestrian (1.6-2.2 m/s) emerges from an occluded gap', _crossing_fast),
    'slow_crosser': ('dynamic', 'Slow pedestrian lingers in the robot lane while crossing', _slow_crosser),
    'diagonal': ('dynamic', 'Pedestrian cuts diagonally across the aisle between gaps', _diagonal),
    'head_on': ('dynamic', 'Pedestrian walks down the aisle toward the robot', _head_on),
    'overtaker': ('dynamic', 'Pedestrian faster than the robot approaches from behind', _overtaker),
    'stop_and_go': ('dynamic', 'Pedestrian stops in the lane for 1.5-3 s, then resumes', _stop_and_go),
    'multi': ('dynamic', 'Two to four pedestrians cross at random times', _multi),
    'blocked': ('static', 'Unmapped pallet on the planned route (requires replanning)', _blocked),
    'chicane': ('static', 'Two staggered unmapped pallets form a chicane', _chicane),
    'dead_end': ('static', 'Unmapped pallets block the aisle completely (no path exists)', _dead_end),
    'loiter': ('static', 'Pedestrian walks into the lane and stands still', _loiter),
    'noisy': ('perception', 'Crossing with Gaussian position noise on every observation', _noisy),
    'forklift': ('stress', 'Forklift (2.6-3.2 m/s) emerges from the nearer gap timed at the robot',
                 _forklift),
    'ambush': ('unavoidable', 'Forklift appears 0.2 m away at 3 m/s: no physically feasible escape', _ambush),
}

SCENARIO_TYPES = tuple(CATALOG)


def make_scenario(stype: str, seed: int) -> Scenario:
    if stype not in CATALOG:
        raise ValueError(f'unknown scenario {stype!r}; expected one of {", ".join(SCENARIO_TYPES)}')
    category, desc, gen = CATALOG[stype]
    c = _Ctx(stype, int(seed))
    sigma = gen(c) or 0.0
    return Scenario(stype, int(seed), category, desc, c.start, c.goal, c.shelves, c.statics,
                    c.movers, _r(sigma), c.params)
