# Energy-Efficient Path Planning (UC Merced)

Energy-aware motion planning for autonomous UAVs under finite battery budgets.
The system benchmarks OMPL sampling-based planners by compute energy (Intel RAPL)
and path quality, then uses an **EI/J metaheuristic** (Expected Improvement per
Joule) to pick the best planner and parameters at each replan step along a route,
adapting to remaining battery and path progress.

Research under Prof. Stefano Carpin, UC Merced Robotics Lab.

---

## Pipeline

```
[C++ harness]  ->  raw sweep CSV  ->  [EI/J sim]  ->  step CSV  ->  [HTML viz]
 main.cpp          (benchmark data)   run_eij_sim.py  (decisions)   drop-in upload
```

1. **Benchmark** — `main.cpp` sweeps 7 planners x configs x 25 route positions,
   logging RAPL energy, timing, path length, smoothness, waypoints, and obstacle
   clearance. Output is a raw sweep CSV.
2. **Decide** — `run_eij_sim.py` flies a simulated mission, replanning every 5%,
   choosing each operator by EI/J (quality per joule) with a hard clearance
   safety gate and battery/progress-adaptive weighting. Output is a step CSV.
3. **Visualize** — the HTML files render the mission: cumulative energy vs fixed
   baselines, planner switches, and a per-step operator inspector.

---

## Files

| File | Purpose | Status |
|------|---------|--------|
| `main.cpp` | Current C++ harness (FIX 1-25): route-sampled sweep, RAPL energy, resolution + clearance logging | Active, primary |
| `main_v2.cpp` | Prior harness (FIX 1-21), before clearance/resolution additions | Archived backup |
| `run_eij_sim.py` | EI/J policy simulator; runs on any raw sweep CSV | Active |
| `CMakeLists.txt` | Build config for the C++ harness | Active |
| `eij_visualization_upload.html` | Mission visualizer with drag-and-drop CSV upload (for demos) | Active |
| `eij_energy_visualization.html` | Mission visualizer with data baked in | Active |
| `Apartment_env.dae` | Primary test environment (benchmarked) | Active input |
| `cubicles_env.dae` | Secondary environment, queued for reruns | Input, not yet benchmarked |
| `Twistycool_env.dae` | Secondary environment, queued for reruns | Input, not yet benchmarked |
| `cubicles_robot.dae` | Alternate cubicles variant | Reference input |
| `.gitignore` | Excludes build artifacts and large generated CSVs | Active |

Large sweep CSVs (e.g. `sweep_v3.csv`, ~150 MB) are intentionally **not** in the
repo; they regenerate from the harness.

---

## Build & run

```bash
# dependencies (Ubuntu)
sudo apt install -y build-essential cmake libompl-dev libassimp-dev libboost-all-dev python3-pip
pip install numpy pandas scikit-learn --break-system-packages

# build the harness
mkdir -p build && cd build && cmake .. && make -j$(nproc) && cd ..

# make RAPL energy counters readable (resets each boot)
sudo chmod -R a+r /sys/class/powercap/intel-rapl

# run a benchmark sweep
./build/run_cpp_benchmark --dry-run                 # preview the plan
./build/run_cpp_benchmark --out sweep.csv           # full sweep

# run the EI/J mission sim on the sweep
python3 run_eij_sim.py --raw sweep.csv --out steps.csv

# then open eij_visualization_upload.html and drag in steps.csv
```

Harness flags: `--planner`, `--query`, `--starts`, `--trials`, `--seed`,
`--out`, `--dry-run`. Sim flags: `--raw`, `--out`, `--query`, `--slowdown`.

---

## Notes & caveats

- Energy is measured per `solve()` call via RAPL, bracketed and batch-summed to
  survive the ~1 ms counter tick; raw per-trial reads are logged alongside.
- The EI/J sim uses placeholder vehicle constants (hover/flight power, speed,
  battery, compute slowdown) set at the top of `run_eij_sim.py` — swap in real
  UAV specs without retraining.
- The path-quality surrogate is gradient-boosted trees (beat an MLP on held-out
  tests for this dataset). Clearance is taken from lookup, not the surrogate,
  since it does not predict reliably across geometry.
- Current scope: single-environment proof of concept. Multi-environment reruns
  and kinodynamic planning are the next phases.

Last updated: 2026-10-03
