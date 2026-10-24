# AEGIS — requirements

## User story 1: Demonstration
As a judge, I can select a hazardous warehouse scenario and seed and watch the baseline and AEGIS side by side on the identical scenario.

Acceptance: at least 12 distinguishable scenario types (15 implemented). Synchronized split-screen replay showing the robot, movers, pallets, goal, A* plan, predicted future paths, the chosen action with its rationale, the evaluated candidates, the event log and the outcome. Play, pause, step, scrub, speed and jump-to-intervention controls are available.

## User story 2: Safety supervisor
As a researcher, I can inspect interventions produced by a physics-aware short-horizon forecast and conservative constraints.

Acceptance: velocity changes are limited to A_MAX*DT in both modes and in the forecasts. Forecast uncertainty grows with lookahead and sensor noise. A conservative fallback applies when no candidate is verified safe. Every decision is logged with a rationale. No heuristic is labelled as a probability.

## User story 3: Evaluation
As a judge, I can run repeatable paired benchmarks comparing collisions, mission completion, timeouts, safe stops and time.

Acceptance: each seed changes the environment, and each pair shares the identical environment. Tests prove determinism, termination, identical pairing and honest collision reporting. At least one physically unavoidable case is included and reported as a failure. Parameters are tuned on development seeds and reported on held-out seeds. Limitations are documented.

## User story 4: Hackathon submission
As a participant, I can commit the code with the root `.kiro` directory, document actual Kiro involvement, produce a working demo and record a public demo video under three minutes.

Acceptance: README, setup instructions, dependency files, an honest Kiro usage log, and a submission approved by a human. Status: repository, hosting, video and submission are still open (see tasks.md).
