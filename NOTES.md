# File Notes

Status log for files in this repo. Dates are last meaningful edit / finalization.

## Code
- `main.cpp` — Current C++ harness (FIX 1-25). Last edited 2026-08-27 (added clearance + resolution sweep, FIX 22-25). Finalized/active. [pushed 2026-10-03]
- `main_v2.cpp` — Prior harness (FIX 1-21). Last edited 2026-08-22, superseded by main.cpp. Archived backup. [pushed 2026-10-03]
- `run_eij_sim.py` — EI/J policy simulator. Finalized 2026-08-28. Active, works on any raw sweep CSV. [pushed 2026-10-03]
- `CMakeLists.txt` — Build config. Last edited 2026-07-22, stable. Active. [pushed 2026-10-03]

## Visualizations
- `eij_visualization_upload.html` — Mission visualizer with drag-and-drop CSV upload (demo tool). Finalized 2026-08-28. Active. [pushed 2026-10-03]
- `eij_energy_visualization.html` — Mission visualizer with data baked in. Finalized 2026-08-28. Active. [pushed 2026-10-03]

## Environment meshes
- `Apartment_env.dae` — Primary environment, benchmarked. Active input. [pushed 2026-10-03]
- `cubicles_env.dae` — Secondary environment. Input, queued for reruns, not yet benchmarked. [pushed 2026-10-03]
- `Twistycool_env.dae` — Secondary environment. Input, queued for reruns, not yet benchmarked. [pushed 2026-10-03]
- `cubicles_robot.dae` — Alternate cubicles variant. Reference input. [pushed 2026-10-03]

## Config
- `.gitignore` — Excludes build artifacts and large CSVs from the repo. Active. [pushed 2026-10-03]
