# AEGIS — Predict Failure. Protect the Mission.

**Hackachusetts 2026 · Robotics & Hardware · Prototype v0.2**

AEGIS is a predictive runtime safety supervisor for a simulated autonomous warehouse robot. It forecasts how nearby people, forklifts and obstacles will move. It then checks candidate commands by simulating the robot's real acceleration limits, and it brakes, evades or replans with A* before a predicted conflict happens. A split-screen dashboard replays AEGIS and an unsupervised baseline side by side on the identical seeded scenario.

> **Prototype disclosure.** This is an educational 2D simulation, not certified safety software. It has not been validated on hardware, it uses no learned perception, and it outputs no calibrated collision probability. Pedestrians are scripted and do not react to the robot. Results below describe this simulator only.

## Quick start (Windows, PowerShell)

Requires Python 3.10+ (tested with 3.12.1).

```powershell
cd c:\Prathmesh\aegis-hackachusetts-2026
python -m venv .venv
.\.venv\Scripts\Activate.ps1          # if blocked: Set-ExecutionPolicy -Scope Process Bypass
pip install -r requirements-dev.txt   # runtime + test dependencies
python -m uvicorn server:app --host 127.0.0.1 --port 8000
```

Then open **http://127.0.0.1:8000** in a browser. Press `Ctrl+C` to stop the server.

macOS/Linux: use `source .venv/bin/activate` and the same commands.

Runtime-only install (for hosting): `pip install -r requirements.txt`.

### Using the dashboard
1. Pick a **scenario** (grouped by category) and a **seed**, or press 🎲 for a random seed. Then press **Run paired replay**.
2. The left panel is the baseline and the right panel is AEGIS. Both are driven by the same server-side simulation of the same scenario and stay synchronized frame by frame.
3. The controls are play/pause (`Space`), step (`←`/`→`), the time scrubber, the 0.5–4× speed selector, and **First intervention**.
4. The AEGIS panel overlays the chosen manoeuvre's predicted path (purple, or red during fallback) and the growing forecast disks for each tracked obstacle. Dashed pallets are ones AEGIS has not yet detected.
5. **AEGIS decision** shows the rationale and every candidate evaluated this frame, with its worst forecast slack and the object that limited it.
6. **Paired benchmark** runs on the server. A cold run took 10–15 s for 10 pairs per type in our runs, and the result is then cached.

## Tests

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

The latest run gave **81 passed** in about 20 s:

| File | What it checks |
|---|---|
| `tests/test_engine.py` | determinism, termination, the velocity-change limit in both modes, a distinct environment for every seed, the baseline being unfiltered, the supervisor never PROCEEDing on an unsafe forecast, fallback restricted to brake/evade, noise only on AEGIS observations, and consistent collision reporting |
| `tests/test_planner.py` | A* reaches the goal, avoids circles and walls, passes through gaps, returns `None` when blocked, and keeps all prior maps clear. It also checks AEGIS replans around pallets and reaches the goal, and `dead_end` is reported as a safe stop |
| `tests/test_adversarial.py` | one paired test per scenario type (15), checking an identical environment and mover positions, that the case really challenges the baseline, and AEGIS collision bounds. `ambush` must collide and must be flagged as anticipated |
| `tests/test_properties.py` | Hypothesis property-based tests: the acceleration limit, scenario purity, mover speed bounds, collision-free A* on random maps, and whole-episode invariants |
| `tests/test_server.py` | API endpoints, backward-compatible `/api/run`, and input validation (400/422) |

## Benchmark

```powershell
.\.venv\Scripts\python.exe -m aegis --start 1000 --count 30          # Markdown table
.\.venv\Scripts\python.exe -m aegis --start 1000 --count 30 --json   # full JSON
```

### Method
- **Paired design:** for each scenario type and seed, the baseline and AEGIS run on the byte-identical scenario: layout, start/goal, unmapped pallets, mover scripts and noise level. Tests verify this.
- **Dev/held-out split:** controller parameters were tuned only on seeds 0–19. The parameters were then frozen and seeds **1000–1029** were run, giving 30 pairs × 15 types = 450 pairs. The results below come from that run and were reproduced identically in the final verification (runtime ≈ 35 s).
- **Adversarial by design:** most dynamic movers are timed to reach the robot's lane when a nominal robot would arrive, with a seeded jitter. This makes the scenarios harder than typical traffic.
- **Baseline:** the same A* plan on the prior map and the same tracker and dynamics. It has no perception, no replanning and no safety layer.

### Held-out results (seeds 1000–1029, 30 pairs per type)

| Scenario | Category | Baseline collisions | AEGIS collisions | Baseline completed | AEGIS completed | AEGIS safe stop (no path) | AEGIS mean time (s) |
|---|---|---|---|---|---|---|---|
| crossing | dynamic | 19/30 | 0/30 | 11/30 | 30/30 | 0 | 7.85 |
| crossing_fast | dynamic | 29/30 | 0/30 | 1/30 | 30/30 | 0 | 7.73 |
| slow_crosser | dynamic | 30/30 | 0/30 | 0/30 | 30/30 | 0 | 9.12 |
| diagonal | dynamic | 26/30 | 0/30 | 4/30 | 30/30 | 0 | 8.29 |
| head_on | dynamic | 30/30 | 0/30 | 0/30 | 30/30 | 0 | 10.51 |
| overtaker | dynamic | 19/30 | 0/30 | 11/30 | 30/30 | 0 | 8.53 |
| stop_and_go | dynamic | 30/30 | 0/30 | 0/30 | 30/30 | 0 | 9.81 |
| multi | dynamic | 20/30 | 0/30 | 10/30 | 30/30 | 0 | 8.79 |
| blocked | static | 30/30 | 0/30 | 0/30 | 30/30 | 0 | 7.12 |
| chicane | static | 30/30 | 0/30 | 0/30 | 30/30 | 0 | 7.22 |
| dead_end | static | 30/30 | 0/30 | 0/30 | 0/30 | 30 | — |
| loiter | static | 30/30 | 0/30 | 0/30 | 30/30 | 0 | 10.82 |
| noisy | perception | 20/30 | 0/30 | 10/30 | 30/30 | 0 | 8.97 |
| forklift | stress | 30/30 | 0/30 | 0/30 | 30/30 | 0 | 8.05 |
| ambush | unavoidable | 30/30 | **30/30** | 0/30 | 0/30 | 0 | — |
| **Total (450 pairs)** | | **403** | **30** | **47** | **390** | **30** | |

AEGIS had 0 timeouts, and there were 0 pairs where AEGIS collided but the baseline did not.

### How to read this honestly
- **All 30 AEGIS collisions are `ambush`.** That case is unavoidable by construction: a forklift appears about 0.2 m away, closing at 3 m/s, and the robot can change velocity by only 0.2 m/s per step. In every one of those collisions, the supervisor had already declared that no safe action existed. Nothing is hidden.
- **`dead_end` is a mission failure, not a success.** AEGIS stops safely because no route exists, but it does not complete the mission.
- **The development seeds were worse.** On seeds 0–19, AEGIS collided in 2 of 20 `forklift` runs. During tuning there were also collisions in `noisy` and `overtaker`, which led to the fallback and tracking fixes. Zero collisions on 1000–1029 does not mean zero risk. Other seeds or scenario types can still cause collisions.
- **AEGIS is slower** because it brakes, waits and detours. In the dynamic types, its mean completion time is about 7.7–10.8 s.
- **The baseline is deliberately minimal.** A real robot stack would have at least an emergency stop, so the gap shown here is larger than it would be against a production system.

## Architecture

```
web/index.html        split-screen Canvas replay (no build step)
server.py             FastAPI: /, /healthz, /api/scenarios, /api/compare, /api/run, /api/benchmark
aegis/scenarios.py    15 seeded scenario types (pure function of type + seed)
aegis/engine.py       Simulation (owns all state + decisions), tracker, perception, supervisor, benchmark
aegis/planner.py      A* on inflated occupancy grid + line-of-sight smoothing
aegis/geometry.py     distances, occlusion test
aegis/constants.py    physical / supervisor constants (frozen)
aegis/__main__.py     benchmark CLI
```

Each 100 ms AEGIS tick runs these steps:
1. **Perceive.** Detect movers and pallets within 6 m that are not hidden behind shelves, adding Gaussian noise in `noisy`.
2. **Track.** Fit least-squares velocities, coasting for 1 s through occlusion.
3. **Replan.** Run A* when new pallets are detected, or around a person who has stood still for 1.5 s.
4. **Evaluate.** Score PROCEED, SLOW_60, SLOW_30, STOP, EVADE_LEFT, EVADE_RIGHT and RETREAT. Each candidate is held for 0.4 s, then followed by maximum braking at 2 m/s², simulated with the real acceleration limit over a 2 s horizon. The forecast disks grow with lookahead, noise and velocity uncertainty. The first candidate with clearance ≥ 0.15 m plus growth wins.
5. **Fall back** if nothing is safe: choose the brake/evade manoeuvre with the largest slack, and log `FALLBACK`.
6. **Report.** Ground-truth contact is checked with sub-steps, and each collision records the robot speed, the relative speed, whether the robot was stationary, and whether it was anticipated.

## API
- `GET /api/scenarios`: catalogue of types and categories
- `GET /api/compare?scenario=crossing&seed=7`: baseline and AEGIS replays for the same scenario
- `GET /api/run?scenario=crossing&aegis=true&seed=7`: a single replay (v0.1-compatible)
- `GET /api/benchmark?start=1000&count=10`: paired benchmark with `count` ≤ 30, cached

All endpoints are read-only and **unauthenticated**. Inputs are bounded: `seed` must be 0–1,000,000 and `count` 1–30. If you host it publicly, put it behind a rate limiter or proxy, because a cold benchmark call uses roughly 10–35 s of CPU.

## Deployment
- Any Python host: `pip install -r requirements.txt`, then `uvicorn server:app --host 0.0.0.0 --port $PORT`.
- A `Dockerfile` is included: Python 3.12-slim, non-root user, and it honours `$PORT`. **It has not been built or tested.**

## Limitations
- 2D holonomic disk robot with no heading or turning constraints. Velocity change is limited, but there is no full dynamics model.
- Movers are scripted and non-reactive. There is no model of human interaction.
- Constant-velocity forecasts with hand-set uncertainty growth. No learned or calibrated prediction.
- Perception is idealised: object class and radius are known exactly, pallets do not occlude, and only position noise is modelled.
- Parameters were hand-tuned on the same scenario catalogue. The held-out seeds guard against overfitting to particular seeds, but not to the scenario designs.
- The dashboard was checked headlessly (Node with a mocked DOM) and over live HTTP. It has not been checked for cross-browser compatibility or with assistive technology. Full WCAG validation needs manual testing.

## Submission checklist (not done yet)
- [ ] Create the public GitHub repository and commit the source with the root `.kiro/` (this folder is not a git repository yet)
- [ ] Host a runnable demo
- [ ] Record the under-3-minute demo video
- [ ] Review `KIRO_USAGE.md` and submit on SundAI by **8 PM EDT, October 24, 2026** (pushing code is not a submission)

## Third-party material
Python, FastAPI, Uvicorn and browser-native Canvas, plus pytest, Hypothesis and httpx for testing. No external datasets are used.
