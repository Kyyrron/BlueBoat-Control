# Pinger filter — Phase 0 findings (2026-11-24)

Evidence behind the pinger-pipeline rewrite: live filter, "log pinger only" mode, non-loop path end. Every number
below is recomputed by `docs/img/pinger_findings/make_figures.py`, which reads the field data read-only and writes the
figures and `phase0_metrics.json` beside itself:

```bash
/usr/bin/python3 docs/img/pinger_findings/make_figures.py      # ~8 s
```

The analysis was done before any code changed: Control `44ec6aa`, MCS `9665f79`.

## 1. Short answers

| Question | Answer (evidence: section) |
|---|---|
| Is "0.5 Hz" the device rate? | **No.** It is `uwgps_log`'s 0.5 s poll period. At that poll the device gives **1.5–1.7 new raw fixes/s in the pool** and 0.8–1.5/s in the harbour. 15–20 % of polls return the previous fix. The true device rate (≥ 1.7 Hz) is aliased by the poll (§3) |
| Is `/blueboat/odom` usable, and on which clock? | Values are good. **Header stamps are 0 in every run** (`use_sim_time` on the real launch), so receive time is the timing truth. The rate is **3.8–15.9 Hz, different from run to run** (§4) |
| Is the MAVROS twist non-zero while moving? | Yes: > 0.05 m/s in 59–100 % of samples, up to 0.4–1.7 m/s (§4) |
| y sign of the Water Linked acoustic frame | **As received, y already points left.** No flip is needed: the flip is worse on 19 of 20 runs (whole-run consistency) and puts more fixes outside the surveyed pool on 14 of 20. Not rotating at all is worse on 20 of 20 (§5) |
| Mount yaw | **Not identifiable from these data.** A per-run robust regression gives −9° to +9° around 0, but the grid search is flat and runs into its ±30° limits on 4 runs. **Default 0°, provisional** (§5) |
| Latency | **1.5 s for raw fixes, pooled. Not sharply identifiable** (flat between 1.0 and 1.75 s). WL "filtered" lags a further ≥ 1 s (pooled optimum ≥ 2.5 s, at the 3 s edge of the grid). **Provisional; a still-pinger calibration is mandatory** (§6) |
| Is there still-pinger data? | **No.** The owner confirmed the pinger was always on the brush robot, a slow crawler moving continuously at about 0.1–0.3 m/s, and no idle moment is known. The runs test the pipeline. They cannot calibrate it (§2) |
| What do "the 2026-10-28 follow runs" say about the y sign? | **There are no follow runs on 2026-10-28.** Every `run_state` of those runs has `use_pinger: false`: path mode, with the pinger only recorded in the bag. The 2026-10-27 harbour runs used the pinger layout but were **driven by hand** (owner). The thrust differential does not correlate with the pinger bearing (r = −0.12…+0.21). No recorded run shows the follow law at work (§2, §5) |
| Does today's pinger drag with the boat? | **Partly, and not where the prompt expected** (§7). Between consecutive *new* fixes the legacy world pinger hardly moves with the boat (pooled slope **0.11**). The dead reckoning holds it in between, and new fixes arrive at about 1.65 Hz against the 2 Hz poll. It *is* dragged at duplicate re-applies (slope 0.6–1.0, about 20 % of polls). **The dominant error is a different one: a 1.5–2.5 s old vector is applied at the current pose.** With a perfect, noise-free still pinger on the real motion, the legacy error is p50 0.3–2.8 m and p95 4–9 m, ≈ v·τ ahead along travel and ρ·r·τ sideways in turns |
| Why does "every path act like a loop"? | **The mission files say `loop: true`** (H1, with H2). The designer's single "Loop mission (close last → first)" box adds the closing leg **and** makes Control repeat the path forever. Control is correct (§8) |

## 2. Data inventory

The Makurazaki days are 2026-10-27 (pool in the morning, harbour in the afternoon) and 2026-10-28 (pool). Pool:
50 × 20 m, **0.85 m deep** (`report_Kagoshima/gps_accuracy_study`). The pinger rode a brush robot, a slow continuous
crawler. In the harbour that robot was on a ship's hull.

| Set | Where | What survives | Pinger data |
|---|---|---|---|
| 2026-10-27 pool 10:07–10:44 | `Robot_data/ControllerData/{path-8shape,mpcRandom}` | 6 run bags | `/uw_gps_data` + `/blueboat/pinger_coordinates`. The 10:07 run is all zeros |
| 2026-10-27 pool 11:31–11:55 and harbour 14:57–16:12 | `Processed_Robot_data/pinger*` (= `~/Desktop/randomData/report_Kagoshima/{pool,harbour}`) | **Exports only**; the run folders and bags are gone | Pinger-layout CSV at 3 Hz: raw (`aco_*`), WL filtered (`filaco_*`), legacy world (`corrected_pinger_*`). The pool exports were rebuilt from bags by the log reviewer |
| 2026-10-28 pool 10:49–11:30 | `Processed_Robot_data/pinger_pool_10h49`, `…11h25…`, `…11h30…` | Exports only | As above |
| 2026-10-28 pool 11:39–12:52 | `Robot_data/2026_10_28-*`, `ControllerData/MPC_small_run` | 15 run bags + poslogs + `-path.yaml` | Path-mode runs (`use_pinger: false` in every `run_state`), pinger recorded in the bag |
| 2026-10-28 09:48 GPS test | `Robot_data/2026_10_28-09_48_16-GPS_Accuracy_Test` | bag | UGPS all zeros (pinger off) |

That leaves **20 bags with acoustic fixes and 13 exports**.

## 3. Device rate, polling and transport

One row per bag. "new raw /s" counts *distinct* raw acoustic xyz. "dup %" is the share of polls repeating the last
fix. "HTTP" is the bag receive time minus the packet's own date fields, which are taken at poll start: the duration of
the three sequential requests.

| Run | dur s | new raw /s | dup % | zero pk | gap max s | HTTP p95 / max s | odom Hz | range p50 m |
|---|---|---|---|---|---|---|---|---|
| 10_28-11_44_28 MPC lawnmower BOXReal | 102 | 1.63 | 19 | 0 | 2.1 | 0.12 / 1.2 | 8.1 | 13.7 |
| 10_28-11_47_19 MPC lawnmower BOXReal | 209 | 1.68 | 15 | 0 | 2.1 | 0.11 / 1.2 | 6.0 | 5.4 |
| 10_28-11_56_33 MPC line | 38 | 1.72 | 16 | 0 | 1.1 | 0.11 / 1.1 | 13.5 | 9.3 |
| 10_28-11_58_26 MPC SIN | 28 | 1.51 | 21 | 0 | 2.3 | 1.25 / 1.3 | 13.0 | 9.9 |
| 10_28-11_59_10 MPC SIN | 53 | 1.49 | 19 | 0 | 4.2 | 0.26 / 3.2 | 5.9 | 11.4 |
| 10_28-12_00_34 MPC SIN | 53 | 1.72 | 15 | 0 | 1.2 | 0.09 / 0.3 | 10.0 | 10.6 |
| 10_28-12_02_02 PID SIN | 70 | 1.72 | 14 | 0 | 2.3 | 0.23 / 1.3 | 7.9 | 11.0 |
| 10_28-12_04_07 LOS SIN | 84 | 1.68 | 15 | 0 | 1.8 | 0.08 / 1.3 | 5.6 | 12.2 |
| 10_28-12_07_03 LOS SIN 20m | 75 | 1.64 | 16 | 0 | 2.1 | 0.09 / 1.6 | 13.9 | 9.6 |
| 10_28-12_08_57 MPC SIN 20m | 47 | 1.66 | 15 | 0 | 1.6 | 0.19 / 1.2 | 12.0 | 9.4 |
| 10_28-12_13_18 PID SIN 20m | 78 | 1.59 | 18 | 0 | 1.8 | 0.48 / 1.5 | 10.7 | 10.0 |
| 10_28-12_27_32 BrushRobotClose | 252 | 1.67 | 15 | 0 | 3.3 | 0.25 / 3.3 | 4.0 | 1.0 |
| 10_28-12_36_12 BrushRobotClose | 481 | **0.28** | 12 | **51** | 4.8 | 9.22 / **102.9** | 6.5 | 7.4 |
| 10_28-12_52_16 BrushRobotClose | 566 | 1.61 | 15 | 0 | 7.2 | 0.21 / 6.7 | 3.9 | 14.2 |
| 10_28-11_39_16 MPC lawnmower BOX | 82 | 1.52 | 17 | 0 | 7.0 | 0.13 / 6.9 | 15.9 | 22.5 |
| 10_27-10_44_08 (mpcRandom) | 190 | 1.27 | 14 | 0 | 11.9 | 0.97 / 11.9 | 10.7 | 7.2 |
| 10_27-10_19_17 (8-shape PID) | 137 | 1.39 | 20 | 0 | 11.1 | 0.43 / 10.6 | 3.8 | 11.0 |
| 10_27-10_22_10 (8-shape LoS) | 140 | 1.46 | 23 | 0 | 4.6 | 0.28 / 4.1 | 3.9 | 33.0 |
| 10_27-10_26_18 (8-shape MPC) | 66 | 1.45 | 19 | 0 | 3.6 | 1.05 / 3.1 | 3.8 | 20.4 |
| 10_27-10_27_32 (8-shape PID) | 134 | 1.57 | 17 | 0 | 3.8 | 0.24 / 3.4 | 3.9 | 18.5 |

| Export (3 Hz rows) | dur s | new raw /s | zero rows | gap max s | depth distinct / median m |
|---|---|---|---|---|---|
| harbour 14h57 part 1 | 394 | 1.15 | 110 | 45.5 | 11 / 0.19 |
| harbour 14h57 part 2 | 455 | 0.78 | 299 | 47.2 | 4 / 0.19 |
| harbour 15h12 (better_log) | 1189 | 1.47 | 0 | 56.4 | 14 / 0.19 |
| harbour 15h32 (random_data) | 84 | 0.81 | 0 | 5.0 | 4 / 0.38 |
| harbour 15h33 | 1099 | 0.90 | 466 | 110.5 | 27 / 0.38 |
| harbour 15h52 (gettingtheAUV) | 116 | 1.19 | 0 | 4.6 | 2 / 0.19 |
| harbour 16h12 | 503 | 1.42 | 62 | 20.1 | 2 / 0.19 |
| pool 10-27 11h31 (random_data1) | 81 | 1.78 | 0 | 1.3 | 1 / 0.19 |
| pool 10-27 11h41 (robotMoving) | 233 | 1.73 | 0 | 1.3 | 2 / 0.19 |
| pool 10-27 11h55 (LoSLawnmower) | 181 | 1.67 | 0 | 1.3 | 3 / 0.19 |
| pool 10-28 10h49 | 727 | 1.57 | 0 | 11.6 | 2 / 0.19 |
| pool 10-28 11h25 | 126 | 1.31 | 66 | 32.0 | 2 / 0.19 |
| pool 10-28 11h30 | 132 | 1.56 | 0 | 3.6 | 2 / 0.19 |

- **F2 confirmed.** `requests.get` has no timeout. In the 10-28 12:36 run one request hung for **102.9 s**, and with
  it all pinger data. The median poll cycle is 30 ms, and p95 is 0.1–1.0 s (`fig5_transport.png`).
- **F1 confirmed.** A failed request publishes zeros. The harbour exports hold 62–466 zero rows per run and dropouts of
  up to 110 s. During them the legacy pinger is drawn on the boat.
- **Depth.** The locator z is near-constant at 0.19 m: 1–27 distinct values per run, in a pool 0.85 m deep. Either the
  depth is not measured or it never reaches the UGPS. The model and locator type are unknown (owner). This goes on the
  field checklist, and the new `uwgps_log` records the WL configuration endpoints.
- The receivers are 4 separate units, one at each corner of the boat (owner), not a single antenna housing.

## 4. Boat pose

| Check | Result |
|---|---|
| `/mavros/local_position/odom` and `/blueboat/odom` header stamps | **0.0 in 100 % of messages in all 20 runs.** Every timing below uses the bag's receive time |
| Odom rate | 3.8–15.9 Hz. It changes between runs, from 4 Hz in most 2026-10-27 runs and the BrushRobotClose runs up to 16 Hz |
| MAVROS body twist | Non-zero while moving (> 0.05 m/s in 59–100 % of samples, max 0.4–1.7 m/s) |
| Boat inside the surveyed pool outline | Min distance to the walls −0.12 to +7.9 m. The local ENU frame and the GPS study's corners agree to within the 0.5 m GPS accuracy |

## 5. Frames

`fig3_frames.png`. Raw fixes are placed in the world with the pose interpolated at `t_rx − τ`, best τ per hypothesis.

| Run | CEP50 as received / y flipped / no rotation (m) | > 1 m outside pool: as received / y flipped / WL filtered |
|---|---|---|
| 10_28-11_44_28 | 6.6 / 9.6 / 10.5 | 0.23 / 0.35 / 0.03 |
| 10_28-11_47_19 | 2.5 / 3.1 / 2.9 | 0.07 / 0.19 / 0.03 |
| 10_28-11_56_33 | 3.6 / 4.3 / 9.3 | 0.08 / 0.03 / 0.08 |
| 10_28-11_58_26 | 3.5 / 8.4 / 7.1 | 0.03 / 0.11 / 0.06 |
| 10_28-11_59_10 | 4.4 / 8.6 / 10.7 | 0.08 / 0.26 / 0.01 |
| 10_28-12_00_34 | 4.0 / 6.7 / 10.0 | 0.19 / 0.15 / 0.02 |
| 10_28-12_02_02 | 5.4 / 6.4 / 10.8 | 0.26 / 0.23 / 0.01 |
| 10_28-12_04_07 | 6.8 / 8.2 / 12.5 | 0.32 / 0.21 / 0.15 |
| 10_28-12_07_03 | 3.7 / 5.8 / 8.5 | 0.07 / 0.10 / 0.00 |
| 10_28-12_08_57 | 5.1 / 6.7 / 9.7 | 0.14 / 0.18 / 0.04 |
| 10_28-12_13_18 | 7.2 / 8.6 / 10.1 | 0.17 / 0.26 / 0.05 |
| 10_28-12_27_32 | 1.0 / 1.1 / 1.8 | 0.01 / 0.01 / 0.00 |
| 10_28-12_36_12 | 3.7 / 7.1 / 5.3 | 0.10 / 0.23 / 0.00 |
| 10_28-12_52_16 | 12.8 / 12.1 / 12.9 | 0.56 / 0.53 / 0.27 |
| 10_28-11_39_16 | 7.4 / 13.5 / 22.9 | 0.42 / 0.59 / 0.05 |
| 10_27-10_44_08 | 3.6 / 7.1 / 6.3 | 0.13 / 0.31 / 0.09 |
| 10_27-10_19_17 | 4.8 / 9.7 / 11.9 | 0.19 / 0.32 / 0.06 |
| 10_27-10_22_10 | 17.1 / 24.9 / 30.7 | 0.72 / 0.70 / 0.35 |
| 10_27-10_26_18 | 6.9 / 14.2 / 18.4 | 0.42 / 0.67 / 0.29 |
| 10_27-10_27_32 | 5.0 / 14.5 / 19.4 | 0.28 / 0.53 / 0.09 |

- **y as received = left.** Flipping y is worse on 19 of 20 runs by world consistency and on 14 of 20 by the pool
  outline. A world-aligned frame (no rotation) is worse on 20 of 20. The Water Linked documentation describes a
  forward-right-down acoustic frame. In this installation the 4-corner receiver layout entered in the WL UI defines
  the frame, and what arrives is x forward, y left. **Default `y_sign = +1`.**
- **Mount yaw.** A robust per-run regression of the latency- and yaw-induced displacement gives −9° to +9°, centred on
  0. The local-consistency grid (±30°) is flat and reaches its limits on 4 runs. A moving target cannot separate yaw
  from motion. **Default 0°, provisional.**
- **Lever arm:** unknown. Default 0.
- The 2026-10-27 harbour exports (manual driving) agree on y: local CEP as received is below or equal to flipped on 6
  of 7 runs (`phase0_metrics.json`, `exports`).

## 6. Latency and noise

**Latency** (`fig2_latency.png`). For each τ, every fix is compared with the median of its ±5 s neighbours. Pooled
and normalised per run:

| τ (s) | 0 | 0.5 | 1.0 | 1.25 | **1.5** | 1.75 | 2.0 | 2.5 | 3.0 |
|---|---|---|---|---|---|---|---|---|---|
| raw | 1.168 | 1.147 | 1.083 | 1.077 | **1.071** | 1.087 | 1.116 | 1.172 | 1.199 |
| WL filtered | 1.297 | 1.252 | 1.214 | 1.188 | 1.150 | 1.112 | 1.102 | 1.070 | **1.064** |

The raw optimum is 1.5 s on a flat floor from 1.0 to 1.75 s, matching the 1.5 s of Sørensen et al. (2022). WL filtered
adds at least 1 s on top. A per-run robust regression gives 0.1–1.1 s with large spread. With a moving target,
**latency is not sharply identifiable**: the default of **1.5 s is provisional.**

**Noise** (`fig4_noise_vs_range.png`). Distance of each raw fix (τ = 1.5 s) to the median of its ±5 s neighbours. A
0.1–0.3 m/s crawler moves at most ±1.5 m in that window.

| Range | 0–3 m | 3–6 m | 6–10 m | 10–15 m | 15–20 m | 20–30 m |
|---|---|---|---|---|---|---|
| n | 493 | 474 | 675 | 1065 | 577 | 306 |
| p50 (m) | 0.41 | 1.47 | 2.80 | 5.99 | 7.25 | 7.64 |
| p90 (m) | 1.95 | 4.46 | 9.78 | 17.3 | 21.9 | 25.7 |
| p99 (m) | 14.8 | 8.7 | 15.0 | 25.2 | 30.9 | 44.4 |

- Across range this is about **13–17° (median)**, against a spec of < 1°, with heavy tails.
- **1–72 % of raw fixes per run fall more than 1 m outside the 20 m-wide pool** (median run 16 %); WL filtered output:
  0–35 %. The 12:52 "ring" (`fig1_world_fixes.png`) is 56 % impossible positions, not robot motion.
- Probable causes, untested: multipath in a 0.85 m deep concrete pool, the small baseline of receivers on the boat's
  corners next to the thrusters, and the missing locator depth.
- **Consequence for the filter.** A spec-based measurement model (1°) would reject nearly every fix. The starting
  `sigma_bearing_deg` is 8° and robustness must be strong; Phase 1 tunes both on NIS.

## 7. Legacy pinger: reproduction and the drag

**Reproduction.** Today's `robot_interface` algorithm was replayed on each bag's recorded
`/mavros/local_position/odom`, `/mavros/imu/data` and `/uw_gps_data`, in receive order, with the odom-to-odom `dt`.
The recorded `/blueboat/pinger_coordinates` comes back to:
- median 0.00–0.34 mm in all 20 runs;
- p99 ≤ 4.8 mm in 15 runs.

The remaining large deviations (p99 25 mm–1.2 m in 5 runs) are single messages where the recorder's order differs
from the node's callback order: a UGPS packet landing on the other side of an odom. The algorithm moves into
`legacy_pinger.py` arithmetic-for-arithmetic, and its bit-identity is tested on identical event streams.

**Drag between new fixes (real data).**
- Between consecutive new WL-filtered fixes, the legacy world pinger's displacement against the boat's has a pooled
  slope of **0.11** (3358 intervals).
- New fixes arrive at about 1.65 Hz against a 2 Hz poll, so only ~20 % of intervals contain a duplicate re-apply.
- Between re-applies, the dead reckoning keeps the world point roughly still.

**Mechanism (semi-synthetic, `fig6_legacy_mechanism.png`).** Each run's own motion, odom/IMU/poll timing and duplicate
pattern drive a **noise-free still pinger** through the legacy algorithm, with a 1.5 s latency.

| Run | legacy error p50 / p95 (m) | error along travel p50 (m) | boat speed p50 (m/s) | jump at duplicate re-apply vs boat displacement (slope) |
|---|---|---|---|---|
| 10_28-11_44_28 | 0.76 / 5.1 | 0.73 | 0.49 | 0.12 |
| 10_28-11_47_19 | 0.85 / 8.8 | 0.77 | 0.47 | 0.61 |
| 10_28-11_56_33 | 1.62 / 4.3 | 0.94 | 0.54 | 0.63 |
| 10_28-12_04_07 | 1.17 / 5.1 | 0.49 | 0.33 | 0.94 |
| 10_28-12_07_03 | 1.91 / 4.5 | 0.37 | 0.32 | 0.94 |
| 10_28-11_39_16 | 0.96 / 4.0 | 0.80 | 0.52 | 0.92 |
| 10_27-10_26_18 | 1.69 / 6.5 | 1.27 | 0.97 | 0.76 |
| all 20 | p50 0.3–2.8, p95 1.3–8.8 (30 in the 103 s hang run) | ≈ v · 1.5 s | | 0.6–1.0 on most runs |

- The **dominant legacy error is lag, not drag.** Each new fix was measured τ ago from where the boat was then, and is
  applied at the current pose. The shown pinger therefore sits about v·τ ahead along travel. In turns it swings
  sideways by ρ·r·τ: at 10 m range, r = 0.3 rad/s and τ = 1.5 s that is 4.5 m.
- WL's filtered output, which legacy uses, adds about 1 s more.
- At duplicate re-applies the old vector is re-applied at the newer pose, which is the "drag in 0.5 s steps" (slope
  0.6–1.0).
- **A2 is therefore measured as:**
  - the slope at duplicate re-applies;
  - the error against a known still pinger on real motion: semi-synthetic;
  - the between-fix world displacement of the new estimate, which must be exactly 0.

## 8. Path end: root cause

| Hypothesis | Verdict | Evidence |
|---|---|---|
| H1 the YAML carries `loop: true` | **Confirmed** | 19 of the 23 recorded `-path.yaml` snapshots have `loop: true`: `pool_line_and_back`, `pool_try`, `pool_try_fast`. Every 10-28 run from 11:58 on flew `pool_line_and_back`, whatever its name. 8 deployed and 4 saved missions in `~/.config/blueboat_mcs/trajectories` also carry it |
| H2 closed by design | Partial | Every `loop: true` file ends where it starts, and every `loop: false` file is open. The closure *comes from* the loop box (`mcs/designer/sampling.py:77`). The pattern generators add no return leg |
| H3 origin served when no YAML | Refuted | `path_generation` serves (0, 0) only before the first load. An unlink or a parse failure keeps the old path. No target jump > 2 m at any run end |
| H4 end-of-path controller behaviour | Refuted | After the end every pose of the window is the clamped final pose. MPC regulates to it; PID/LoS engage the hold. 11:47 (`loop: false`): target fixed from 92 s to 209 s, boat 0.14–0.73 m from it, 13.6–14.1 m from the start |
| H5 tau reset or wrap | Refuted | tau resets only at the first ready tick, at START and at a replanner's first path. The only modulo is `yaml_trajectory.py:74`, under `loop` |
| H6 display only | Refuted | The map draws an open line. The closing segment of loop missions is in the samples |

- **Cause.** The designer's single "Loop mission (close last → first)" box adds the closing leg (`sampling.py:77`) and
  writes `loop: true` (`io_yaml.py:110`). At runtime, `YamlTrajectory.pose` then wraps `t % duration` forever. A
  saved mission keeps the box through `.meta.yaml`.
- **Fix (owner's choice).** Split the box: "Return to start" (closing leg, flown once, then hold) and "Repeat forever"
  (writes `loop`, default off). Old missions keep their behaviour until re-saved. Control gains a `path_complete`
  report.
- Side risk, an open item: a START whose new session file fails to parse re-serves the previous run's path.

## 9. Consumers of the topics this work touches

| Topic | Code consumers |
|---|---|
| `/blueboat/pinger_coordinates` | `master_control` (follow target), GCS `blueboat_gcs/ros/pinger_listener.py` (body→world on arrival, drops all-zero), MCS `ros/bridge_node.py` → `models/store.py:on_pinger_body`, `bag_replay` (LIVE_TOPICS), `log_reviewer/reviewer/pinger_bag.py`, `tools/sim_record/gcs_driver.py` (simulation, never fed), `test/mc_harness.py` |
| `/uw_gps_data` | `robot_interface` (CSV + today's reseed), MCS bridge (freshness only), `bag_replay`, `pinger_bag` |
| `/controller_target` | published only in `master_control`'s pinger branch, stored (never read) by `robot_interface`, `bag_replay`, `mc_harness` |

No consumer exists in Belief, Perception, Replanner, BlueBoat-SSS-Sim, `link_guardian` or the Experiments repository.
In simulation nothing publishes the pinger: `simulation_interface` hard-codes `use_UWgps = False`.

Mismatches found on the way:
- `master_control` chases the all-zero vector, i.e. the boat, before the first fix.
- MCS never sends `path_source`.
- MCS never resets its `PingerState` between launches or runs.

## 10. Decisions taken from this evidence

The design follows the approved plan; this section lists only the parts set by the evidence above.

- **Estimator node.** A separate `pinger_estimator` node owns the pinger pipeline and is the single publisher of
  `/blueboat/pinger_coordinates`, `/blueboat/pinger/{legacy_body,estimate,fix,filter_config}`.
- **Filter input.** The filter takes **raw** fixes (WL filtered lags ≥ 1 s more).
  - It compensates latency through a pose buffer stamped with receive times.
  - It keeps the world estimate still between fixes and publishes `b = R(ψ)ᵀ(p̂ − p)` at odom rate.
- **Starting values** (provisional until the Phase 5 calibration):

  | Parameter | Value | Basis |
  |---|---|---|
  | `latency_s` | 1.5 | §6 |
  | `y_sign` | +1 | §5 |
  | `mount_yaw_deg` | 0 | §5 |
  | lever arm | 0 | unknown |
  | `sigma_bearing_deg` | 8 | §6; Phase 1 tunes it on NIS |
  | `v_max` | 0.5 m/s | crawler 0.1–0.3 m/s |
  | `sigma_a` | 0.05 m/s² | |

  Robust (Huber) down-weighting is the default, given the heavy tails.
- **`uwgps_log`.** Request timeouts and a keep-alive session; raw polled at 4 Hz, which will also measure the true
  device rate; the legacy `/uw_gps_data` packet stays byte-compatible on its 2 Hz cycle.
- **Evaluation without a still pinger:**
  - semi-synthetic still pinger on real motion, timing and residuals: CEP, lag, drag;
  - real fixes: fraction outside the pool outline, output speed against the crawler's 0.1–0.3 m/s, NIS, decision
    rates.
- **Field calibration mandatory** before the latency and mapping defaults are trusted (`docs/PINGER_FIELD_CALIBRATION.md`).

## 11. Phase 1 results: the core, its tests and the tuned defaults

**Code** (all ROS-free, `blueboat_control/src/_custom_libraries/`):

| File | Content |
|---|---|
| `config/pinger_filter.yaml` | The one parameter schema (name, group, type, unit, default, range, description) and the presets `slow_rov` (default) and `static_target` |
| `pinger_params.py` | Loads and validates the schema; merges presets, YAML files and overrides; computes the 16-hex hash |
| `pinger_input.py` | `/uw_gps/fix` JSON and `/uw_gps_data` packets → `Fix` |
| `legacy_pinger.py` | Today's algorithm, arithmetic for arithmetic |
| `pinger_filter.py` | Pose buffer, mapping, robust KF, status machine, RTS smoother |
| `pinger_estimator_core.py` | The event-driven pipeline used both online and offline |
| `mcap_lite.py` | The MCAP reader, moved from `log_reviewer`, which re-exports it |
| `pinger_filter_replay.py` | CLI with three commands: `run`, `grid`, `evaluate` |

`test/test_pinger_filter.py`: 148 checks, all passing (129 at Phase 1; 19 added after the final review). `/usr/bin/python3 test/test_pinger_filter.py`, ~3 s.

**A1 (synthetic):**

| Criterion | Result |
|---|---|
| Still target, spec noise + 10 % heavy outliers, filtered RMS ≤ 50 % of raw | Defaults: 0.08 / 0.30 / 0.43 / 0.69 m against raw 4.9–6.0 m at 5 / 12 / 20 / 30 m. `static_target`: 0.02–0.19 m |
| 3-fix burst displaced 20 m | Estimate moves 0.36 m (defaults), 0.02 m (spec noise) |
| Real 15 m relocation | Re-acquired after **4** fixes (`reinit_n` = 4); same for all presets |
| 20 m range, 1.5 s latency, 0.2 rad/s turn | World dispersion **0.37 m** compensated, 5.97 m uncompensated |
| Duplicates and failed requests | Change nothing: identical updates and identical final state and covariance |
| No fix for 4 s, boat at 1 m/s turning | World estimate moves **exactly 0 m**; `b = R(ψ)ᵀ(p̂ − p)` at every tick, to 0 error |
| Speed cap | Velocity estimate ≤ v_max, also for a 2 m/s target (0.500 / 0.300 against caps of 0.5 / 0.3) |
| Online = offline | Fix records identical. Re-running from the per-fix log is bit-identical, RTS included |
| Legacy | Bit-identical to the v1.1 `robot_interface` arithmetic on 5956 random odom events. From each recorded bag's own inputs it reproduces the recorded vector to a median of 0.00–0.09 mm on all 20 pinger bags |

**Re-acquisition change (found by A1).** With the field noise model (bearing 12°, range 20 %) a 15 m relocation gives
NIS 13–16, inside the hard gate, so it was only down-weighted fix by fix and took 35 fixes. Re-acquisition now looks
at every fix beyond the **soft** gate and re-initialises when the last `reinit_n` of them agree within
`reinit_radius_m`. Random outliers do not cluster; a relocation does.

**Tuning** (sweep over `sigma_bearing_deg`, `range_k`, `sigma_a`, `v_hard`).
- Each candidate was scored on 20 bags with the evaluation below.
- The optimum is flat: `sigma_bearing_deg` 10–14°, **`range_k` 0.2** (along-range error ≈ 20 % of range, not the
  spec's 0.2 %), `sigma_a` 0.02–0.03. `v_hard` has no effect.
- **Chosen `slow_rov` defaults: `sigma_bearing_deg` 12, `range_k` 0.2, `sigma_a` 0.02.** Everything else as in the
  plan. Hash `4181149c1767f002` (the schema gained `max_pose_gap_s` after the review).
- `static_target`: `model: static`, `v_max: 0`, `init_radius_m: 5`. Hash `23092b80be249895`.

**A2 — semi-synthetic known truth** (`pinger_filter_replay.py evaluate ~/ros2_ws/data`;
`phase1_a2_slow_rov.json`, `phase1_a2_static_target.json`).
- Each of the 20 pinger bags is replayed with its own boat motion, odom/IMU/poll timing and duplicate/zero pattern.
- Its acoustic values are replaced by a synthetic pinger seen from the pose at `t_rx − 1.5 s`, plus a **real residual**
  of similar range drawn from the 3770 measured residuals.
- Truths: still, and a crawl back and forth over 15 m at 0.2 m/s.
- Legacy is fed a modelled WL-filtered input: 1 s more lag and half the residual.
- Values are medians over runs (m).

| Truth | Output | CEP50 | CEP95 | RMS | Error along travel (p50) | Drag between new fixes (slope, max move) |
|---|---|---|---|---|---|---|
| still | raw fixes | 3.84 | 19.1 | 9.2 | | |
| still | legacy | 3.39 | 10.2 | 5.4 | +0.73 | 0.13, up to 23 m |
| still | **filtered (slow_rov)** | **0.88** | **2.10** | 1.21 | −0.00 | **0.000, 0.000 m** |
| still | smoothed (RTS) | 0.60 | 1.41 | 0.81 | 0.01 | |
| still | filtered (static_target) | 0.34 | 0.79 | 0.47 | | 0.000, 0.000 m |
| crawl 0.2 m/s | legacy | 3.45 | 11.5 | 6.0 | +0.79 | 0.26, up to 21 m |
| crawl 0.2 m/s | **filtered (slow_rov)** | **0.96** | **2.30** | 1.21 | 0.17 | **0.000, 0.000 m** |
| crawl 0.2 m/s | smoothed (RTS) | 0.53 | 1.28 | 0.71 | −0.05 | |
| crawl 0.2 m/s | filtered (static_target) | 3.46 | 7.35 | 4.3 | | |

**A2 — real fixes.**
- 34 runs: 21 bags + 13 CSV exports. The pool outline is applied to the 26 runs launched in the pool.
- The legacy track is computed at odom rate; filtered is the estimate after each fix.

| Output | > 1 m outside pool (mean / median of runs) | Speed p50 / p95 (m/s) | Step p95 (m) |
|---|---|---|---|
| raw fixes | 22.6 % / 17.3 % | 6.8 / 50 | 23 |
| legacy | 12.4 % / 10.2 % | 0.21 / 11.9 | 1.7 |
| **filtered (slow_rov)** | 13.0 % / 7.3 % | 0.45 / 2.9 | 1.6 |
| smoothed (RTS) | 6.4 % / 0.7 % | 0.17 / 0.37 | 0.4 |
| filtered (static_target) | 11.2 % / 2.0 % | 0.07 / 1.2 | 0.6 |

**NIS and decisions.**
- Mean NIS on accepted fixes: **1.79** (median over runs, range 1.02–2.73), inside the 1–3 target.
- On all used fixes (accepted + down-weighted): median 2.1.
- Decisions (median share): 62 % accepted, 19 % down-weighted, 18 % set aside.

**Reading the numbers.**
- On a known truth with real motion, timing and residuals, the filter is **3.6–3.9× better than legacy at CEP50 and
  about 5× at CEP95**. It never moves between fixes, by construction.
- The RTS smoother adds another ~35 %, offline.
- On real fixes the mean outside-pool rate is dominated by two runs that are mostly impossible fixes: 10-28 12:52,
  where the fixes form a 14 m ring around the boat (`phase1_tracks_10_28_1252.png`), and 10-27 10:22. No filter can
  recover a position from those; the filter follows the cloud.
- The filtered output's **position** steps at each update. Its p95 implied speed is 2.9 m/s, against 11.9 for legacy;
  the velocity *state* is capped at `v_max`. A smoother live output means more lag. `static_target` is the right
  preset for a pinger that does not move.

**Figures.**
- `phase1_tracks_10_28_1147.png`, `phase1_tracks_10_28_1252.png`, `phase1_tracks_harbour_1512.png`: legacy vs
  filtered vs smoothed, with fixes coloured by decision and the 10 most down-weighted fixes numbered.
- `phase1_nis_10_28_1147.png`: NIS and decisions over time.

**Reproduce:**

```bash
/usr/bin/python3 blueboat_control/src/_custom_libraries/pinger_filter_replay.py evaluate ~/ros2_ws/data \
    --out /tmp/a2 --outline docs/img/pinger_findings/makurazaki_pool.yaml [--preset static_target]
```

## 12. After the independent review (2026-11-24)

A fresh reviewer found no blocker. It found 4 important and 4 minor issues, all fixed the same day. These numbers are
recomputed with the final code.

**Important**
- **Per-fix log not exactly re-runnable mid-session.**
  - The estimator now emits filter snapshots (`decision: state`) at every run start, after every parameter change and
    every 10 s.
  - `robot_interface` starts a run's file at one, and the replay restores it.
  - Proven on a live ROS graph: re-running the filed log reproduces all 168 live estimate rows bit for bit
    (`test/pinger_integration.py`).
- **Every `configure` restarted the filter.** It now restarts only when the parameter hash changes, and publishes an
  estimate and a snapshot right after.
- **`pinger_source:=legacy` was still gated by the filter's status.**
  - The estimate now carries `source_status`, and `master_control` gates on it.
  - A zero or non-finite vector is never chased (`mc_pinger_scenarios` `pinger_follow_source`).
- **The replay estimator ran on Gazebo's clock.** It now runs on the bag's timeline (`clock_source:=replay`). At replay
  rate 3 its estimates match the offline filter at the same bag time: median 4 mm, p90 0.25 m.

**Minor**
- `/uw_gps/fix` receive times are corrected for the cycle's later requests.
- The live node keeps no forward-pass history (memory).
- Non-finite and malformed inputs are counted as invalid; every callback is guarded.
- `heartbeat_hz` ≥ 0.5.

**Suspicions also fixed**
- A clock stepping back restarts the pose buffer.
- No pose is interpolated across an odometry gap longer than `max_pose_gap_s` (2 s).
- A manual target or the arrow keys drop the follow-mode hold point.
- A saved parameter file builds on the preset it names.
