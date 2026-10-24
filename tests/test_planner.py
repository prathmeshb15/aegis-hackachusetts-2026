"""A* planner: reaches goals, avoids static obstacles, reports infeasibility."""
from math import hypot

from aegis.constants import FIELD_H, FIELD_W, PLANNER_INFLATION, PLANNER_RES
from aegis.engine import Simulation, run_episode
from aegis.planner import GridPlanner, path_length
from aegis.scenarios import make_scenario


def _planner():
    return GridPlanner(FIELD_W, FIELD_H, PLANNER_RES, PLANNER_INFLATION)


def _assert_clear(pl, path, rects, circles):
    for a, b in zip(path, path[1:]):
        n = max(1, int(hypot(b[0] - a[0], b[1] - a[1]) / 0.02))
        for k in range(n + 1):
            x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
            assert pl.clearance(x, y, rects, circles) >= PLANNER_INFLATION - 1e-6, (x, y)


def test_open_field_is_straight_line():
    path = _planner().plan((1, 5), (11, 5), [], [])
    assert path[0] == (1, 5) and path[-1] == (11, 5)
    assert len(path) == 2


def test_routes_around_circle():
    pl = _planner()
    circles = [(6.0, 5.0, 0.8)]
    path = pl.plan((1, 5), (11, 5), [], circles)
    assert path is not None and path[-1] == (11, 5)
    assert len(path) > 2                       # had to bend
    assert path_length(path) < 10 * 1.15       # but not absurdly long
    _assert_clear(pl, path, [], circles)


def test_routes_through_gap_between_walls():
    pl = _planner()
    rects = [(5.5, 0.0, 6.5, 6.0), (5.5, 7.4, 6.5, 10.0)]   # 1.4 m gap at y 6.0-7.4
    path = pl.plan((1, 3), (11, 3), rects, [])
    assert path is not None
    assert any(6.0 < p[1] < 7.4 for p in path)
    _assert_clear(pl, path, rects, [])


def test_no_path_when_wall_blocks():
    assert _planner().plan((1, 5), (11, 5), [(5.5, 0.0, 6.5, 10.0)], []) is None


def test_goal_inside_obstacle_is_infeasible():
    assert _planner().plan((1, 5), (11, 5), [], [(11, 5, 0.5)]) is None


def test_all_scenario_prior_maps_are_plannable_and_clear():
    pl = _planner()
    from aegis.scenarios import SCENARIO_TYPES
    for st in SCENARIO_TYPES:
        for seed in range(5):
            sc = make_scenario(st, seed)
            path = pl.plan(sc.start, sc.goal, sc.shelves, [])
            assert path is not None, (st, seed)
            _assert_clear(pl, path, sc.shelves, [])


def test_baseline_and_aegis_share_initial_plan():
    a, b = Simulation('blocked', True, 4), Simulation('blocked', False, 4)
    assert a.plan == b.plan


def test_aegis_replans_and_reaches_goal_around_unmapped_pallets():
    for st in ('blocked', 'chicane'):
        for seed in range(10):
            r = run_episode(st, True, seed)
            assert r['completed'] and not r['collision'], (st, seed, r)
            assert r['replans'] >= 1
            assert r['min_clearance_static'] > 0


def test_dead_end_is_reported_not_faked():
    for seed in range(5):
        a = run_episode('dead_end', True, seed)
        assert a['outcome'] == 'blocked_safe_stop' and not a['collision'] and not a['completed']
        s = Simulation('dead_end', True, seed, record=False)
        while not s.finished():
            s.tick()
        assert any(e['type'] == 'NO_PATH' for e in s.events)
