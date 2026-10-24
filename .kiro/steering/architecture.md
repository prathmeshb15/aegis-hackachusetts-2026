# Architecture and agent collaboration

Python engine in `aegis/engine.py` owns simulation and safety decisions. FastAPI `server.py` owns HTTP endpoints. Standalone browser dashboard lives in `web/index.html`; no build step required. Tests are in `tests/`. Dependency versions defined in `requirements.txt`.

Endpoints:
- GET `/api/run?scenario=crossing|blocked|noisy&aegis=true|false&seed=7` returns `states` and `final` objects.
- GET `/api/benchmark` returns evidence generated at request time.

Implement changes with tests. No fabricated measurements. Agents should not simultaneously modify shared modules. Do not alter physical-safety scope or submit externally without user approval.
