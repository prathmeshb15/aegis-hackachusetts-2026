# Implementation task board

## Existing baseline (implemented before Kiro development)
- [x] Simulator, safety filter, benchmark endpoint and browser dashboard scaffold (v0.1)
- [x] Initial deterministic regression tests

## Kiro implementation work (done in a Kiro agent session; see KIRO_USAGE.md)
- [x] Replace hardcoded detour waypoints with an A* grid planner plus exact-geometry smoothing (`aegis/planner.py`); tests for reaching the goal, avoiding obstacles and reporting infeasibility (`tests/test_planner.py`)
- [x] Seeded, varied scenarios: 15 types, a different environment per seed, identical environment per baseline/AEGIS pair (`aegis/scenarios.py`)
- [x] Acceleration-limited dynamics, commit-then-brake rollouts, growing-uncertainty forecasts, conservative fallback, honest collision attribution (`aegis/engine.py`)
- [x] Unavoidable case (`ambush`) that is required to collide and to be reported as anticipated
- [x] Automated tests: invariants, paired adversarial suite (one test per type), Hypothesis property-based tests, API tests (81 tests)
- [x] Split-screen synchronized replay with forecasts, candidate table, rationale, event log, scrubber and benchmark table (`web/index.html`)
- [x] Benchmark CLI (`python -m aegis`) and held-out results in the README
- [x] PostFileSave hook file that runs pytest (`.kiro/hooks/pytest-on-python-save.json`); not yet observed firing
- [ ] Deploy a public demo (Dockerfile written but not built or tested)
- [ ] Capture honest screenshots and the under-3-minute demo video
- [ ] Create the public GitHub repository and commit (no git repository exists in this folder yet)
- [ ] Submit via SundAI (requires human approval)
