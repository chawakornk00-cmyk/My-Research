# Tri-Sensor Self-Verification & Fault Isolation in 3D Maze Exploration

> **Status — Run #1 (`tri_sensor_maze_v1`):** Exploratory simulation research benchmark (**simulation-only**; no real-hardware validation).

An independent multi-sensor self-verification and fault-isolation study evaluating how three heterogeneous localization sensors operating in the same 3D environment cross-check one another without ground truth, isolate faulty channels under drift or sensor failures, strictly exclude corrupted measurements from both **display** and **control** outputs, and **fail closed** when no verified majority exists.

---

## 1. Architecture Overview

Three independent simulated localization channels track the robot inside a 3D subterranean maze (`10.0 x 10.0 x 2.5 m`, 5 static pillars, `0.25 m` voxel grid):

| Channel | Simulated Modality | Nominal $\sigma$ |
| --- | --- | ---: |
| `S1_VIO` | Visual-Inertial Odometry | `0.030 m` |
| `S2_LIDAR` | LiDAR Scan-Matching Odometry | `0.035 m` |
| `S3_UWB` | UWB Beacon + IMU Fusion | `0.045 m` |

The proposed **`TriSensorVerifier`** (`my_sensor_research/tri_sensor_verifier.py`) applies four sequential verification layers with zero ground-truth leakage:

1. **Physical Validity & Arena Bounds Guard:** Immediately rejects `NaN`, `Inf`, or out-of-bounds readings (`SENSOR_BLACKOUT`).
2. **Retimestamped-Freeze Detector:** Detects sensors whose physical reading stays static while peers exhibit genuine motion, even when metadata and timestamps appear fresh.
3. **Pairwise Normalized Disagreement + Deterministic Page CUSUM:** Computes pairwise normalized parity residuals across `(S1, S2)`, `(S2, S3)`, and `(S1, S3)`, combining an instantaneous spike gate ($z > 3.2$) with deterministic Page CUSUM ($k = 1.65$, $h = 4.0$) for gradual and heavy drift. *(Scientific note: Page CUSUM reflects at zero and is **not** a nonnegative supermartingale or Ville-anytime-valid.)*
4. **Majority Isolation, Quarantine & Fail-Closed Abstention:** Isolates any single disagreeing sensor (`weight = 0.0`), fuses healthy sensors via inverse-variance weighting, logs rejected values separately in `audit_quarantine_events.jsonl` (strictly excluded from `display_pos` and `control_pos`), and triggers `ABSTAIN_SAFE_STOP` when fewer than two mutually agreeing sensors remain.

---

## 2. Benchmark Results — Run #1 (`20 Scenarios x 3 Arms = 60 Maze Runs`)

Evaluated across `NOMINAL`, six fault categories tested individually on every sensor (`GRADUAL_DRIFT`, `HEAVY_DRIFT`, `SUDDEN_BIAS`, `OUTLIER_BURSTS`, `RETIMESTAMPED_FREEZE`, `SENSOR_BLACKOUT`), and a simultaneous dual-sensor failure stress test (`DUAL_S1_DRIFT_S2_FREEZE`):

| Arm | Mean RMSE | Max RMSE | Worst Single-Tick Error | Fault-Window RMSE | Maze Collisions (20 runs) | Mean Maze Coverage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| **Naive Average Baseline** | `0.6037 m` | `1.7375 m` | `3.0365 m` | `0.8045 m` | `111` | `57.7%` |
| **Two-Sensor Min-Sigma Baseline** | `0.7039 m` | `4.4080 m` | `7.3839 m` | `0.9232 m` | `65` | `59.4%` |
| **Tri-Sensor Verifier (Proposed)** | **`0.0413 m`** | **`0.0540 m`** | **`0.1131 m`** | **`0.0462 m`** | **`0`** | **`61.7%`** |

* **Mean Fault Isolation Accuracy:** `96.1%` (`557` faulty readings excluded from display & control)
* **Separated Audit Log Events:** `626` (`345` quarantine isolations, `133` physical rejections, `116` freeze detections, `32` fail-closed abstentions)
* **Unit Tests:** `14/14` passing (`pytest`)

### Benchmark Figures (300 DPI)

![Display Position RMSE Across All 20 Maze Scenarios](my_sensor_research/logs/tri_sensor_maze_v1/figures/fig1_rmse_across_all_scenarios.png)

![Live Fault Isolation: S1_VIO Heavy Drift + Page CUSUM + Quarantine](my_sensor_research/logs/tri_sensor_maze_v1/figures/fig3_drift_cusum_quarantine.png)

![3 Sensors x 6 Fault Types Matrix](my_sensor_research/logs/tri_sensor_maze_v1/figures/fig4_sensor_fault_heatmap.png)

![3D Subterranean Maze Trajectories Showcase](my_sensor_research/logs/tri_sensor_maze_v1/figures/fig2_maze_trajectories_showcase.png)

---

## 3. Identified Limitations & Counterexamples (Research Integrity)

1. **CUSUM Spike-Poisoning on Intermittent Outliers:** While false alarm rate is `0.0%` across 17 of 20 scenarios, large single-tick `OUTLIER_BURSTS` (`2.4 m`) saturate the Page CUSUM accumulator near its cap (`40.0`), leading to `50.0%–69.2%` false alarm rates on subsequent clean ticks while CUSUM decays slowly. Planned fix for Run #2: decouple instantaneous spike gating from the Page CUSUM residual input via Winsorized clipping.
2. **Gradual Drift Onset Latency:** Slow linear drift (`0.030 m/step`) requires `5–6` steps to exceed the CUSUM threshold (`81.8%` fault-window isolation; max displayed error `<= 0.113 m`).
3. **Simulation-Only Scope:** All experiments use deterministic synthetic Gaussian noise and fault models; correlated real-world sensor degradation, clock skew, and hardware-in-the-loop validation are out of scope for Run #1.

---

## 4. Repository Structure

```text
my_sensor_research/
├── sensor_suite.py                   # 3-sensor simulator & 6 GitHub-aligned fault modes
├── tri_sensor_verifier.py            # 4-layer self-verification, Page CUSUM, quarantine & baselines
├── maze_experiment_runner.py         # 20-scenario x 3-arm 3D maze benchmark & logger
├── plot_results.py                   # 300-DPI figure generation suite
├── build_dashboard.py                # Interactive 5-tab HTML dashboard builder
├── run_all.py                        # One-click full pipeline runner
├── tests/
│   └── test_tri_sensor_verifier.py   # 14 standalone pytest unit tests
└── logs/tri_sensor_maze_v1/
    ├── RUN_01_SUMMARY.txt            # Formal plain-text experiment report
    ├── experiment_config.json        # Full reproducible configuration & seeds
    ├── summary_table.csv             # 60-run aggregate metrics table
    ├── summary_report.json           # JSON summary report
    ├── audit_quarantine_events.jsonl # Separated quarantine & fail-closed audit log
    ├── dashboard.html                # Interactive 5-tab browser dashboard
    ├── figures/                      # 4 publication-grade PNG charts (300 DPI)
    └── per_scenario/                 # 20 tick-by-tick CSV telemetry logs
```

---

## 5. Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Run standalone unit tests (14 tests — no external repo required)
pytest -v my_sensor_research/tests/test_tri_sensor_verifier.py
```

To re-run the full 3D subterranean maze benchmark (`maze_experiment_runner.py`), place `src/` from [`Chalermsak1/nexvia-aegis`](https://github.com/Chalermsak1/nexvia-aegis) (for `aegis.navigation.frontier_exploration`) on `PYTHONPATH`:

```bash
PYTHONPATH=/path/to/nexvia-aegis/src:. python my_sensor_research/run_all.py
```

You can also inspect the recorded Run #1 interactive 5-tab visualization directly in any browser by opening `my_sensor_research/logs/tri_sensor_maze_v1/dashboard.html`.

---

## Attribution

The 3D subterranean maze voxel grid and frontier exploration planner used during simulation experiments are credited to **NEXVIA CORE / NEXVIA AEGIS** (`Chalermsak1/nexvia-aegis`, `src/aegis/navigation/frontier_exploration.py`). All upstream repository files are intentionally excluded from this repository.
