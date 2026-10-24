"""FastAPI app serving the AEGIS dashboard and simulation API.

All endpoints are read-only and unauthenticated (public demo). Expensive inputs
are bounded (seed range, benchmark size) and benchmark results are cached.
"""
from functools import lru_cache
from pathlib import Path

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse

from aegis.engine import benchmark, replay, run_pair
from aegis.scenarios import CATALOG, SCENARIO_TYPES

app = FastAPI(title='AEGIS Robotics Digital Twin', version='0.2.0')
ROOT = Path(__file__).parent
MAX_SEED = 1_000_000


def _check(scenario: str):
    if scenario not in CATALOG:
        raise HTTPException(status_code=400, detail=f'unknown scenario {scenario!r}')


@app.get('/')
def index():
    return FileResponse(ROOT / 'web' / 'index.html')


@app.get('/healthz')
def health():
    return {'ok': True}


@app.get('/api/scenarios')
def scenarios():
    return [{'type': k, 'category': v[0], 'description': v[1]} for k, v in CATALOG.items()]


@app.get('/api/run')
def run(scenario: str = 'crossing', aegis: bool = True, seed: int = Query(7, ge=0, le=MAX_SEED)):
    _check(scenario)
    return replay(scenario, aegis, seed)


@app.get('/api/compare')
def compare(scenario: str = 'crossing', seed: int = Query(7, ge=0, le=MAX_SEED)):
    _check(scenario)
    return run_pair(scenario, seed)


@lru_cache(maxsize=8)
def _bench(start: int, count: int):
    return benchmark(range(start, start + count), SCENARIO_TYPES)


@app.get('/api/benchmark')
def results(start: int = Query(1000, ge=0, le=MAX_SEED), count: int = Query(20, ge=1, le=30)):
    return _bench(start, count)
