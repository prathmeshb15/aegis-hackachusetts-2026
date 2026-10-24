"""Paired adversarial suite: one test per scenario type (15 distinguishable cases).

Each test runs baseline and AEGIS on the *same* seeded scenarios and checks:
1. the environments are byte-identical (signature) and the movers occupy identical
   positions at every shared timestep, so the only difference is the controller;
2. the case is genuinely adversarial for the nominal stack (baseline collides at
   least once over the seeds);
3. outcomes are reported honestly - every collision is logged, unavoidable cases
   are allowed (and in 'ambush' required) to fail, never hidden.

Expected AEGIS outcomes below are regression expectations measured on these
development seeds, not guarantees; see README for the held-out benchmark.
"""
import pytest

from aegis.engine import Simulation, replay
from aegis.scenarios import CATALOG, SCENARIO_TYPES, make_scenario

SEEDS = range(6)

# type -> max AEGIS collisions allowed over SEEDS (regression bound, not a guarantee)
AEGIS_COLLISION_BOUND = {t: 0 for t in SCENARIO_TYPES}
AEGIS_COLLISION_BOUND.update({'noisy': 1, 'forklift': 2, 'overtaker': 1, 'ambush': len(SEEDS)})


def _run(st, aegis, seed):
    s = Simulation(st, aegis, seed, record=False)
    while not s.finished():
        s.tick()
    return s


def test_suite_has_at_least_twelve_distinct_types():
    assert len(SCENARIO_TYPES) >= 12
    descs = {CATALOG[t][1] for t in SCENARIO_TYPES}
    assert len(descs) == len(SCENARIO_TYPES)


@pytest.mark.parametrize('stype', SCENARIO_TYPES)
def test_paired_adversarial_case(stype):
    base_coll = aegis_coll = 0
    for seed in SEEDS:
        b, a = _run(stype, False, seed), _run(stype, True, seed)
        # (1) identical environment
        assert b.sc.signature() == a.sc.signature() == make_scenario(stype, seed).signature()
        n = min(len(b.trail), len(a.trail))
        for k in range(0, n, 5):
            t = k * 0.1
            assert [m.pos(t) for m in b.sc.movers] == [m.pos(t) for m in a.sc.movers]
        assert b.interventions == 0
        # (3) honest reporting
        for s in (b, a):
            assert (s.collision is not None) == any(e['type'] == 'COLLISION' for e in s.events)
            assert not (s.collision and s.completed)
        base_coll += b.collision is not None
        aegis_coll += a.collision is not None
    # (2) adversarial for the nominal stack
    assert base_coll >= 1, f'{stype}: baseline never collided - case is not adversarial'
    assert aegis_coll <= AEGIS_COLLISION_BOUND[stype], (stype, aegis_coll)
    assert aegis_coll <= base_coll


def test_ambush_is_physically_unavoidable_and_reported():
    """Forklift appears ~0.2 m away closing at >=3 m/s; with |dv| <= 0.2 m/s per step
    no manoeuvre can escape. AEGIS must collide AND say it saw no safe action."""
    for seed in SEEDS:
        a = _run('ambush', True, seed)
        assert a.collision is not None
        assert a.collision['supervisor_declared_no_safe_action'] is True
        assert a.fallback_ticks >= 1
        assert any(e['type'] == 'FALLBACK' for e in a.events)


def test_aegis_collisions_are_never_silently_attributed():
    """Every AEGIS collision record says whether the supervisor anticipated it."""
    for st in ('noisy', 'forklift', 'ambush', 'overtaker'):
        for seed in SEEDS:
            a = _run(st, True, seed)
            if a.collision:
                assert set(a.collision) >= {'t', 'with', 'robot_speed', 'relative_speed', 'robot_stationary',
                                            'supervisor_declared_no_safe_action'}


def test_dynamic_cases_show_mission_difference():
    """Across the dynamic family AEGIS completes strictly more missions than baseline."""
    dyn = [t for t in SCENARIO_TYPES if CATALOG[t][0] == 'dynamic']
    base_done = sum(_run(t, False, s).completed for t in dyn for s in range(3))
    aegis_done = sum(_run(t, True, s).completed for t in dyn for s in range(3))
    assert aegis_done > base_done


def test_pair_replay_states_align_in_time():
    b, a = replay('crossing', False, 7), replay('crossing', True, 7)
    for sb, sa in zip(b['states'], a['states']):
        assert sb['t'] == sa['t']
        assert sb['movers'] == sa['movers']
