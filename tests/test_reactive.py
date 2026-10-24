"""Reactive baseline + three-way experiment: fairness, information limits, regression."""
import copy
import json
from pathlib import Path

import pytest

from aegis.constants import A_MAX, DT, V_MAX
from aegis.engine import (CONTROLLERS, Simulation, benchmark3, replay, resolve_controller, run_episode)
from aegis.scenarios import SCENARIO_TYPES

GOLDEN = json.loads((Path(__file__).parent / 'golden_v02_summaries.json').read_text())


def _run(st, seed, controller, mutate=None):
    s = Simulation(st, seed=seed, record=False, controller=controller)
    if mutate:
        mutate(s)
    while not s.finished():
        s.tick()
    return s


# ---------------------------------------------------------------- backwards compatibility
def test_v02_basic_and_aegis_results_unchanged():
    """Golden summaries captured from the unmodified v0.2 engine before this branch."""
    for key, expected in GOLDEN.items():
        st, seed, a = key.split('|')
        assert run_episode(st, bool(int(a)), int(seed)) == expected, key


def test_controller_selection_is_backward_compatible():
    assert resolve_controller(True) == 'aegis' and resolve_controller(False) == 'basic'
    assert resolve_controller(True, 'reactive') == 'reactive'
    assert Simulation('crossing', False, 1).controller == 'basic'
    with pytest.raises(ValueError):
        Simulation('crossing', controller='magic')
    assert set(CONTROLLERS) == {'basic', 'reactive', 'aegis'}


def test_explicit_controller_matches_boolean_api():
    for st in ('crossing', 'blocked'):
        a = run_episode(st, True, 3)
        b = run_episode(st, seed=3, controller='aegis')
        assert {k: b[k] for k in a} == a
        c = run_episode(st, False, 3)
        d = run_episode(st, seed=3, controller='basic')
        assert {k: d[k] for k in c} == c


# ---------------------------------------------------------------- fairness
@pytest.mark.parametrize('stype', SCENARIO_TYPES)
def test_all_controllers_share_environment_dynamics_and_plan(stype):
    sims = {c: Simulation(stype, seed=2, record=False, controller=c) for c in CONTROLLERS}
    sigs = {s.sc.signature() for s in sims.values()}
    assert len(sigs) == 1
    assert len({json.dumps(s.plan) for s in sims.values()}) == 1
    for s in sims.values():
        while not s.finished():
            s.tick()
            assert (s.vx ** 2 + s.vy ** 2) ** 0.5 <= V_MAX + 1e-9
        assert s.max_dv <= A_MAX * DT + 1e-9


def test_reactive_uses_same_perception_layer_as_aegis():
    """Same detection events (range/occlusion/noise stream) until the trajectories diverge."""
    r = Simulation('noisy', seed=4, record=False, controller='reactive')
    a = Simulation('noisy', seed=4, record=False, controller='aegis')
    assert r.perceives and a.perceives and r.window == a.window
    r.tick(), a.tick()
    assert {k: list(v.obs) for k, v in r.tracks.items()} == {k: list(v.obs) for k, v in a.tracks.items()}


def test_reactive_has_no_access_to_future_mover_trajectories():
    """Changing a mover's script AFTER time T cannot change Reactive's behaviour BEFORE T."""
    for st, seed in (('crossing', 0), ('head_on', 1), ('stop_and_go', 2), ('multi', 3)):
        base = Simulation(st, seed=seed, record=False, controller='reactive')
        m = base.sc.movers[0]
        t_change = m.waypoints[-2][0]
        if t_change <= 0.5:
            continue

        def mutate(s, t_change=t_change):
            wp = s.sc.movers[0].waypoints
            tl, xl, yl = wp[-1]
            wp[-1] = (tl, xl + 3.0, yl)        # different motion only after t_change

        a = _run(st, seed, 'reactive')
        b = _run(st, seed, 'reactive', mutate)
        k = int(t_change / DT) - 1
        n = min(k, len(a.trail), len(b.trail))
        assert a.trail[:n] == b.trail[:n], (st, seed)


def test_reactive_never_extrapolates_tracks():
    s = Simulation('crossing', seed=5, record=False, controller='reactive')
    while not s.finished() and not s.tracks:
        s.tick()
    obs = s._forecasts(at_last_seen=True)
    tr = next(iter(s.tracks.values()))
    x, y = tr.estimate(tr.last_seen, s.sc.sensor_sigma)[:2]
    assert (obs[0]['x'], obs[0]['y']) == (x, y)


# ---------------------------------------------------------------- competence
def test_reactive_brakes_or_avoids_for_people():
    s = _run('slow_crosser', 0, 'reactive')
    assert s.interventions > 0
    assert any(e['type'] in ('SLOW', 'AVOID', 'EMERGENCY_STOP') for e in s.events)


def test_reactive_replans_around_unmapped_pallets():
    for st in ('blocked', 'chicane'):
        for seed in range(5):
            r = run_episode(st, seed=seed, controller='reactive')
            assert r['completed'] and not r['collision'] and r['replans'] >= 1, (st, seed)


def test_reactive_dead_end_is_safe_stop():
    for seed in range(3):
        r = run_episode('dead_end', seed=seed, controller='reactive')
        assert r['outcome'] == 'blocked_safe_stop' and not r['collision']


def test_reactive_beats_basic_on_development_seeds():
    """Sanity check that the baseline is competent, not a strawman."""
    types = ('crossing', 'slow_crosser', 'blocked', 'loiter', 'noisy')
    basic = sum(run_episode(t, seed=s, controller='basic')['collision'] for t in types for s in range(4))
    react = sum(run_episode(t, seed=s, controller='reactive')['collision'] for t in types for s in range(4))
    assert react < basic


def test_reactive_replay_records_rationale():
    r = replay('crossing', seed=7, controller='reactive')
    assert r['final']['controller'] == 'reactive'
    assert all('decision' in s for s in r['states'][1:])
    assert all(set(s['decision']) == {'action', 'rationale'} for s in r['states'][1:])


# ---------------------------------------------------------------- benchmark
def test_benchmark3_shape_and_consistency():
    res = benchmark3(range(2), ('crossing', 'dead_end'))
    assert res['controllers'] == list(CONTROLLERS)
    assert res['totals']['episodes_per_controller'] == 4
    for row in res['results']:
        for c in CONTROLLERS:
            assert row[c]['collisions'] + row[c]['completed'] + row[c]['timeouts'] + \
                row[c]['safe_stops_no_path'] == row['episodes']
    rva = res['totals']['reactive_vs_aegis']
    assert rva['both_completed'] + rva['aegis_only_completed'] <= 4
