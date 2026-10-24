"""Core simulator / supervisor invariants."""
import json

import pytest

from aegis.constants import A_MAX, DT, EPISODE_LIMIT, V_MAX
from aegis.engine import (CANDIDATE_ORDER, FALLBACK_SET, Simulation, Track, accel_limit, benchmark,
                          replay, run_episode, run_pair)
from aegis.scenarios import SCENARIO_TYPES, make_scenario


def test_deterministic_pair():
    a = json.dumps(run_pair('crossing', 8), sort_keys=True)
    b = json.dumps(run_pair('crossing', 8), sort_keys=True)
    assert a == b


@pytest.mark.parametrize('scenario', SCENARIO_TYPES)
def test_finite_termination_and_bounded_dynamics(scenario):
    for enabled in (True, False):
        s = Simulation(scenario, enabled, 3, record=False)
        while not s.finished():
            s.tick()
        assert s.steps <= int(round(EPISODE_LIMIT / DT)) + 1
        assert len(s.trail) == s.steps + 1
        assert s.completed or s.collision is not None or s.t >= EPISODE_LIMIT - 1e-9
        # Physically meaningful velocity-change constraint holds in both modes.
        assert s.max_dv <= A_MAX * DT + 1e-9


@pytest.mark.parametrize('scenario', SCENARIO_TYPES)
def test_seeds_produce_distinct_environments(scenario):
    sigs = {make_scenario(scenario, seed).signature() for seed in range(20)}
    assert len(sigs) == 20


def test_baseline_is_unfiltered():
    for st in ('crossing', 'head_on', 'blocked'):
        r = run_episode(st, False, 1)
        assert r['interventions'] == 0 and r['replans'] == 0


def test_guardian_intervenes_on_crossing():
    assert run_episode('slow_crosser', True, 0)['interventions'] > 0


def test_invalid_scenario():
    with pytest.raises(ValueError):
        Simulation('fake')


def test_accel_limit_examples():
    vx, vy = accel_limit(0.0, 0.0, 10.0, 0.0)
    assert vx == pytest.approx(A_MAX * DT) and vy == 0
    vx, vy = accel_limit(V_MAX, 0.0, -V_MAX, 0.0)   # cannot reverse instantly
    assert vx == pytest.approx(V_MAX - A_MAX * DT)


def test_track_least_squares_exact_without_noise():
    tr = Track(0, 'pedestrian', 0.3, 4)
    for k in range(4):
        t = k * DT
        tr.obs.append((t, 1.0 + 0.8 * t, 2.0 - 1.2 * t))
    x, y, vx, vy, sp, sv, unknown = tr.estimate(0.3, 0.0)
    assert not unknown and sp == 0 and sv == 0
    assert (vx, vy) == pytest.approx((0.8, -1.2))
    assert (x, y) == pytest.approx((1.24, 1.64))


def test_supervisor_never_proceeds_on_unsafe_forecast():
    """Whenever a recorded decision is PROCEED, its own forecast slack was >= 0, and
    fallbacks only use brake/evade manoeuvres."""
    for st in ('crossing', 'head_on', 'forklift', 'noisy', 'stop_and_go'):
        for seed in range(3):
            s = Simulation(st, True, seed)
            while not s.finished():
                s.tick()
                d = s.decision
                names = [c['name'] for c in d['candidates']]
                assert set(names) <= set(CANDIDATE_ORDER)
                if d['fallback']:
                    assert d['action'] in FALLBACK_SET
                    assert all(not c['safe'] for c in d['candidates'])
                else:
                    chosen = next(c for c in d['candidates'] if c['name'] == d['action'])
                    assert chosen['safe'] and chosen['slack'] >= 0
                if d['action'] == 'PROCEED':
                    assert d['nominal_slack'] >= 0


def test_noise_only_affects_aegis_observations():
    s = Simulation('noisy', True, 2)
    assert s.sc.sensor_sigma > 0
    while not s.finished() and not s.tracks:
        s.tick()
    tr = next(iter(s.tracks.values()))
    t, ox, oy = tr.obs[-1]
    tx, ty = s.sc.movers[tr.id].pos(t)
    assert (ox, oy) != (tx, ty)


def test_replay_contains_forecast_and_rationale():
    r = replay('crossing', True, 7)
    decided = [st for st in r['states'] if 'decision' in st]
    assert decided and all(st['decision']['rationale'] for st in decided)
    assert any(st['decision']['forecast_robot'] for st in decided)
    assert r['events'][0]['type'] == 'START'


def test_collision_reporting_is_consistent():
    for st in SCENARIO_TYPES:
        for aegis in (False, True):
            s = Simulation(st, aegis, 5, record=False)
            while not s.finished():
                s.tick()
            has_event = any(e['type'] == 'COLLISION' for e in s.events)
            assert has_event == (s.collision is not None)
            if s.collision:
                assert s.outcome() == 'collision' and not s.completed


def test_benchmark_shape():
    r = benchmark(range(2), ('crossing', 'blocked'))
    assert [x['scenario'] for x in r['results']] == ['crossing', 'blocked']
    assert all(x['pairs'] == 2 for x in r['results'])
    assert r['totals']['pairs'] == 4
