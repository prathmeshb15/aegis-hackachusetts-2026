"""Property-based tests (Hypothesis) over random seeds, commands and maps."""
from math import hypot

from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from aegis.constants import A_MAX, DT, FIELD_H, FIELD_W, PLANNER_INFLATION, PLANNER_RES, V_MAX
from aegis.engine import Simulation, accel_limit
from aegis.planner import GridPlanner
from aegis.scenarios import SCENARIO_TYPES, make_scenario

types = st.sampled_from(SCENARIO_TYPES)
seeds = st.integers(min_value=0, max_value=1_000_000)
speed = st.floats(min_value=-5, max_value=5, allow_nan=False)


@given(speed, speed, speed, speed)
def test_accel_limit_respects_physics(vx, vy, cx, cy):
    s = hypot(vx, vy)
    if s > V_MAX:                       # start from a physically reachable velocity
        vx, vy = vx * V_MAX / s, vy * V_MAX / s
    nvx, nvy = accel_limit(vx, vy, cx, cy)
    assert hypot(nvx - vx, nvy - vy) <= A_MAX * DT + 1e-9
    assert hypot(nvx, nvy) <= V_MAX + 1e-9


@given(types, seeds)
def test_scenarios_are_pure_functions_of_seed(stype, seed):
    a, b = make_scenario(stype, seed), make_scenario(stype, seed)
    assert a.signature() == b.signature()
    # Start/goal are in the free aisle and not inside any shelf or pallet.
    for (x, y) in (a.start, a.goal):
        assert 2.6 < y < 7.4
        for (cx, cy, r) in a.statics:
            assert hypot(x - cx, y - cy) > r


@given(types, seeds)
def test_mover_scripts_have_bounded_speed_and_continuity(stype, seed):
    sc = make_scenario(stype, seed)
    for m in sc.movers:
        for (t0, x0, y0), (t1, x1, y1) in zip(m.waypoints, m.waypoints[1:]):
            assert t1 >= t0
            if t1 > t0:
                assert hypot(x1 - x0, y1 - y0) / (t1 - t0) <= 3.3


@settings(max_examples=40, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(st.lists(st.tuples(st.floats(2.0, 10.0), st.floats(1.0, 9.0), st.floats(0.2, 1.0)), max_size=6))
def test_planner_paths_are_collision_free_or_none(circles):
    pl = GridPlanner(FIELD_W, FIELD_H, PLANNER_RES, PLANNER_INFLATION)
    start, goal = (1.0, 5.0), (11.0, 5.0)
    circles = [c for c in circles if hypot(c[0] - goal[0], c[1] - goal[1]) > c[2] + 1.0
               and hypot(c[0] - start[0], c[1] - start[1]) > c[2] + 1.0]
    path = pl.plan(start, goal, [], circles)
    if path is None:
        return
    assert path[0] == start and path[-1] == goal
    for a, b in zip(path, path[1:]):
        n = max(1, int(hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
        for k in range(n + 1):
            x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
            assert pl.clearance(x, y, [], circles) >= PLANNER_INFLATION - 1e-6


@settings(max_examples=25, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(types, seeds, st.booleans())
def test_episode_invariants(stype, seed, aegis):
    s = Simulation(stype, aegis, seed, record=False)
    while not s.finished():
        s.tick()
        assert hypot(s.vx, s.vy) <= V_MAX + 1e-9
    assert s.max_dv <= A_MAX * DT + 1e-9
    assert (s.collision is not None) == any(e['type'] == 'COLLISION' for e in s.events)
    assert not (s.completed and s.collision)
    if not aegis:
        assert s.interventions == 0 and s.replans == 0
    if s.completed:
        assert hypot(s.x - s.goal[0], s.y - s.goal[1]) < 0.25
