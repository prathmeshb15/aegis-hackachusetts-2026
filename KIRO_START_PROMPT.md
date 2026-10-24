# Paste this prompt into Kiro IDE or Kiro Web (not itself evidence of execution)

You are the primary implementing agent for AEGIS (Hackachusetts Robotics & Hardware). Read README.md, .kiro/steering/*.md and .kiro/specs/aegis/*.md before editing. The supplied code is an existing tested v0.1 scaffold. You must **genuinely implement** important improvements and run all tests; document your specific contributions.

Priority A: enhance the 2D simulator with truly seeded, varied obstacle trajectories and at least 12 distinguishable paired adversarial tests. Use identical seeds/environment parameters for baseline and AEGIS, then show meaningful collision/mission completion differences. Avoid conveniently rigged paths and disclose limitations.

Priority B: replace hardcoded static-detour waypoints with A* or another real collision-aware path planner. Ensure the same simulator owns all states and controller decisions. Add tests for reaching the goal and avoiding static obstacles.

Priority C: improve supervisor with physically meaningful velocity-change constraints, robust forecast and conservative fallback. Add adversarial cases where accidents are unavoidable and ensure honest reporting, rather than claiming perfection.

Priority D: upgrade dashboard to synchronized split-screen AEGIS vs baseline replay, predicted future paths, action rationale and polished judge-friendly controls.

Priority E: run tests, clean exceptions, update truthful README benchmark numbers, add setup/deployment details. Keep .kiro folder tracked. If Kiro hooks or property-based testing are supported in this environment, configure/use them meaningfully and describe precisely what ran. Never fabricate Kiro usage or results.

Prioritize a finished working project over extra features. Work independently where routine choices are needed, use bounded dependencies, never modify other repositories, never commit credentials, and do not submit or publish external material without my approval. Return commands to run, test output, list of edited files and limitations.
