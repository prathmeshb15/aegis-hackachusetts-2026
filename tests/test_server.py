"""HTTP API smoke tests via FastAPI's TestClient."""
from fastapi.testclient import TestClient

from aegis.scenarios import SCENARIO_TYPES
from server import app

client = TestClient(app)


def test_index_and_health():
    assert client.get('/').status_code == 200
    assert client.get('/healthz').json() == {'ok': True}


def test_scenarios_listed():
    r = client.get('/api/scenarios').json()
    assert [x['type'] for x in r] == list(SCENARIO_TYPES)


def test_compare_returns_pair_on_identical_scenario():
    r = client.get('/api/compare', params={'scenario': 'blocked', 'seed': 3})
    assert r.status_code == 200
    d = r.json()
    assert d['baseline']['final']['aegis'] is False and d['aegis']['final']['aegis'] is True
    assert d['scenario']['seed'] == 3 and d['scenario']['statics']


def test_run_endpoint_backward_compatible():
    d = client.get('/api/run', params={'scenario': 'crossing', 'aegis': 'false', 'seed': 7}).json()
    assert d['states'] and d['final']['scenario'] == 'crossing'


def test_bad_inputs_rejected():
    assert client.get('/api/compare', params={'scenario': 'nope'}).status_code == 400
    assert client.get('/api/compare', params={'seed': -1}).status_code == 422
    assert client.get('/api/benchmark', params={'count': 31}).status_code == 422


def test_benchmark_endpoint_small():
    d = client.get('/api/benchmark', params={'start': 0, 'count': 1}).json()
    assert len(d['results']) == len(SCENARIO_TYPES) and d['totals']['pairs'] == len(SCENARIO_TYPES)
