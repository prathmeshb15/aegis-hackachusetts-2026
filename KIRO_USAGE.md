# How Kiro was used on AEGIS

This file only records what actually happened. Anything that was not done or not observed is marked as such.

## Starting point (before Kiro)
The v0.1 scaffold was created outside Kiro: a single-file engine, hardcoded detour waypoints, a dashboard, 8 tests, and the `.kiro/steering` and `.kiro/specs` documents. The v0.1 benchmark repeated one deterministic case per scenario, because seeds did not vary obstacles.

## Kiro surface
All v0.2 work was done by the Kiro agent in one autonomous agent session in this workspace. The prompt we used is `KIRO_START_PROMPT.md`. The agent read `README.md`, `.kiro/steering/product.md`, `.kiro/steering/architecture.md` and `.kiro/specs/aegis/{requirements,design,tasks}.md` before editing.

## How the steering and spec shaped the work
- `product.md`: avoid claims of calibrated probability, learned AI or hardware validation. As a result, the dashboard and README show forecast slack in metres instead of a "risk %", and every limitation is disclosed.
- `architecture.md`: the engine owns simulation and decisions, FastAPI owns HTTP, there is no front-end build step, and no measurements may be fabricated. This module split was kept, and the dashboard is still a single static HTML file.
- `tasks.md`: the open items (A* planner, seeded scenarios, braking constraints, unavoidable cases, property tests, split-screen UI) were implemented in priority order. The board was then updated to show what is done and what is still open.
- `requirements.md` and `design.md` were rewritten afterwards to describe the system as built.

This was a spec-guided session: the agent worked from the existing spec files. It did not use Kiro's interactive "requirements → design → tasks" approval flow.

## What the agent implemented (files)
| Area | Files |
|---|---|
| A* planner + smoothing | `aegis/planner.py`, `aegis/geometry.py` |
| 15 seeded scenario types | `aegis/scenarios.py` |
| Dynamics, perception, tracking, supervisor, fallback, replanning, reporting | `aegis/engine.py`, `aegis/constants.py` |
| Benchmark CLI | `aegis/__main__.py` |
| API (bounded inputs, cached benchmark, `/api/compare`) | `server.py` |
| Split-screen dashboard | `web/index.html` |
| Tests (81) | `tests/test_engine.py`, `tests/test_planner.py`, `tests/test_adversarial.py`, `tests/test_properties.py`, `tests/test_server.py`, `pytest.ini` |
| Setup / deploy | `requirements.txt`, `requirements-dev.txt`, `Dockerfile`, `.dockerignore`, `.gitignore`, `README.md` |

## Testing that actually ran
- `python -m pytest -q`: 81 passed (final run after the last edits).
- Property-based testing: yes, with Hypothesis (`tests/test_properties.py`). Five properties are tested: the acceleration limit, scenario purity, mover speed bounds, collision-free A* paths on random maps, and whole-episode invariants on random scenario/seed/mode combinations.
- The dashboard JavaScript was syntax-checked with `node --check` and run headlessly in Node against mocked DOM/canvas and real API JSON, with no runtime errors. It was not checked in a real browser by the agent.
- Live HTTP smoke test through uvicorn: `/`, `/healthz`, `/api/scenarios`, `/api/compare`, `/api/benchmark` (cold 10–15 s for 10 pairs/type, cached ≤0.03 s), and the 400/422 validation responses.

## Development process, including corrections
- Controller parameters were tuned only on seeds 0–19. The parameters were then frozen, and the README reports seeds 1000–1029.
- Problems found during tuning and fixed:
  - Robots deadlocked behind a standing pedestrian. Fixed by padding temporary obstacles beyond the forecast margin, replanning when held, and a recovery preference after a 3 s hold.
  - The fallback could choose a reduced-speed PROCEED. Fallback is now restricted to brake/evade manoeuvres.
  - Noisy two-sample velocity estimates spiked. Fixed with a velocity-trust threshold and a plausibility speed cap.
- Scenario change made during development, disclosed here: `overtaker` start times moved from 0.5–2.0 s to 1.5–3.0 s. Earlier starts spawned the walker about 0.2 m behind the robot, which is a spawn artefact rather than an overtaking test. A separate `ambush` type was added for deliberately unavoidable contact.

## Hooks
- Created with Kiro's hook tool: `.kiro/hooks/pytest-on-python-save.json` (PostFileSave on `aegis/`, `tests/` or `server.py` Python files → `.venv\Scripts\python.exe -m pytest -q -x`).
- **Not yet observed firing.** Kiro reported that the hook becomes active at the next session start. Every test run listed above was started by the agent directly, not by the hook. Check it by saving a Python file in a new session.

## Not used
No MCP servers, Powers, Kiro CLI or sub-agent crews were used. No external datasets were used.

## Still to do by the team (not done by Kiro)
Create the public GitHub repository and commit (this folder is not a git repository yet). Host the demo. Record the under-3-minute video. Submit on SundAI. Before submitting, re-check this file against the git history.
