# Architecture design (v0.2, as implemented)

## Modules
- `aegis/constants.py`: physical and supervisor constants (SI units). Frozen before the held-out benchmark.
- `aegis/geometry.py`: disk/rectangle distances, segment-rectangle intersection (Liang-Barsky) for occlusion.
- `aegis/planner.py`: A* on a 0.2 m occupancy grid inflated by robot radius + 0.17 m, 8-connected, no corner cutting, then line-of-sight shortcut smoothing checked against exact geometry. Returns `None` when no route exists.
- `aegis/scenarios.py`: 15 seeded scenario types. Each scenario is a pure function of `(type, seed)`: a 12 x 10 m warehouse with two shelf rows and two cross-aisle gaps, random start/goal, unmapped pallets, and scripted movers (time-parameterised polylines).
- `aegis/engine.py`: one `Simulation` owns all world state and every controller decision for both modes.
- `aegis/__main__.py`: benchmark CLI.
- `server.py`: FastAPI API with bounded inputs and a cached benchmark.
- `web/index.html`: split-screen synchronized replay (no build step).

## Robot and world
- Holonomic disk robot, radius 0.28 m, max speed 1.5 m/s. Velocity changes are limited to at most A_MAX*DT = 0.2 m/s per 100 ms step (A_MAX = 2.0 m/s^2). The same limit applies in both modes and inside the supervisor's rollouts.
- Ground-truth contact is checked against movers, pallets, shelves and walls using 4 sub-steps per tick.
- Movers do not react to the robot (open loop), so paired runs see identical trajectories.

## Nominal stack (both modes)
A* on the prior map (shelves only), followed by pure-pursuit tracking (0.7 m lookahead) with goal-approach speed limiting. The baseline is exactly this stack: it has no perception, no replanning and no safety layer.

## AEGIS runtime layer
1. Perception: movers and pallets within 6 m and with line of sight that is not blocked by shelves. In the `noisy` type, observations get Gaussian position noise (seeded stream, separate from the world).
2. Tracking: least-squares constant velocity over the last 4 observations (10 when noisy). Speed is capped at a plausibility limit. When the velocity standard error exceeds 0.5 m/s, the track is treated as "unknown velocity". Tracks coast for 1 s when occluded.
3. Forecast: each obstacle is a disk whose radius grows with lookahead. Growth = min(0.2 m/s * t, 0.4 m) + 2*sigma terms, plus a 0.9 m/s * t reachable-set term when the velocity is unknown.
4. Safety filter: the candidates are PROCEED, SLOW_60, SLOW_30, STOP, EVADE_LEFT, EVADE_RIGHT and RETREAT, tried in preference order. Each candidate is held for 0.4 s, then followed by maximum braking. It is simulated with the real acceleration limit over a 2 s horizon and must keep at least 0.15 m clearance plus growth to every forecast disk and at least 0.05 m to known statics, shelves and walls. The first safe candidate wins. If none is safe, the conservative fallback picks the brake/evade manoeuvre with the largest slack (never a reduced-speed PROCEED) and logs FALLBACK.
5. Replanning: A* replans when a new pallet is detected. A person who stays near-stationary for 1.5 s while the robot is held becomes a temporary padded obstacle. After 3 s held, safe moves that create space are preferred. With no route, the robot holds position and the run ends as `blocked_safe_stop`.
6. Reporting: each collision records the robot speed, the relative speed, whether the robot was stationary, and whether the supervisor had declared "no safe action" within the previous 1.5 s.

## Known technical debt
No orientation or turning constraints, no learned or calibrated prediction, no interaction-aware human models, oracle object class and radius, pallets do not occlude, grid resolution limits narrow passages, and the parameters were hand-tuned on the same scenario catalogue (but evaluated on held-out seeds).
