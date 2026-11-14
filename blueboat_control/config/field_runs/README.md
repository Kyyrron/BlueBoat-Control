# Field-run configurations (2026-11-13)

One ROS parameter file per field day and controller: the gains flown that day and the odometry
timing of that day. Used with `Sim_launch.py tuning_file:=<file>` to re-fly a field run in simulation
with the controller as it was (the plant, the real MPC model and 20 Hz thrust come from
`sim_config:=field_equivalent`, the default).

What a file cannot restore: the code. Every run before 2026-11-13 flew the MPC with the heading fault at
+-180 deg (one full-differential thrust pulse per crossing); all twelve runs of 2026-09-07 crossed it.

| Field run (folder under `~/ros2_ws/data`) | Split | Controller | File | Route to give `trajectory:=` | Odometry |
|---|---|---|---|---|---|
| `Sept_data/ControllersComparison/2026_09_07-10_39_16-PidSin2Real` | FIT | PID | `2026_09_07_PID.yaml` | sine weave | 3.0 Hz |
| `…/2026_09_07-10_51_09-LosSin` | FIT | LoS | `2026_09_07_LoS.yaml` | sine weave | 3.0 Hz |
| `…/2026_09_07-11_01_14-MPCSin` | FIT | MPC | `2026_09_07_MPC.yaml` | sine weave | 3.0 Hz |
| `…/2026_09_07-11_07_44-SquarePID` | FIT | PID | `2026_09_07_PID.yaml` | expanding square | 3.0 Hz |
| `…/2026_09_07-11_15_27-SquareLoS` | FIT | LoS | `2026_09_07_LoS.yaml` | expanding square | 3.0 Hz |
| `…/2026_09_07-11_25_10-SquareMPC` | FIT | MPC | `2026_09_07_MPC.yaml` | expanding square | 3.0 Hz |
| `…/2026_09_07-11_33_01-PIDNonRegular` | HELD-OUT | PID | `2026_09_07_PID.yaml` | tough angles | 3.0 Hz |
| `…/2026_09_07-11_39_16-LoSNonRegular` | HELD-OUT | LoS | `2026_09_07_LoS.yaml` | tough angles | 3.0 Hz |
| `…/2026_09_07-11_56_59-MPCNonRegular` | HELD-OUT | MPC | `2026_09_07_MPC.yaml` | tough angles | 3.0 Hz |
| `…/2026_09_07-12_04_24-LawnmowerPID` | HELD-OUT | PID | `2026_09_07_PID.yaml` | lawnmower | 3.0 Hz |
| `…/2026_09_07-12_14_16-LawnmowerLoS` | HELD-OUT | LoS | `2026_09_07_LoS.yaml` | lawnmower | 3.0 Hz |
| `…/2026_09_07-12_26_53-LawnmowerMPC` | HELD-OUT | MPC | `2026_09_07_MPC.yaml` | lawnmower | 3.0 Hz |
| 2026-10-27 MPC runs (8-shape 10_07_17, 10_26_18; mpcRandom 10_44_08; MPCLawnmower 10_48_53; HarbourMPC 16_04_45, 16_10_14) | HELD-OUT | MPC | `2026_10_27_MPC_pre_retune.yaml` (MPC was not retuned) | the run's `-path.yaml` where it survives | 4 Hz (10_44_08: `odom_profile:=field_13hz`) |
| 2026-10-27 PID / LoS runs (8-shape 10_19_17, 10_22_10, 10_27_32; LoSLawnmower 11_55_53) | HELD-OUT | PID / LoS | `…_pre_retune.yaml` **or** `…_post_retune.yaml`: UNCERTAIN | the run's `-path.yaml` | 4 Hz |
| 2026-10-28 runs | HELD-OUT | as named | `2026_10_28_<CTRL>.yaml` | the run's `-path.yaml` | 4–16 Hz by run: see `timing/odom_timing.json` in the session evidence (`odom_profile:=field_4hz` for the runs at 4 Hz) |

The four routes of 2026-09-07 are the designer files `c_plage_sin.yaml` (sine weave), `c_plage_square.yaml`
(expanding square), `c_plage_non_regular.yaml` (tough angles) and `c_plage_lawnmower.yaml` (lawnmower; rewritten
on 2026-10-12 — `.deployed/plage_lawnmower.yaml` may be the version flown, to check) under
`~/.config/blueboat_mcs/trajectories/`, authored at 0.5 m/s. No `-path.yaml` snapshot was written then. To re-fly
one in a plain `Sim_launch.py`, translate it so that its first point is the origin and give its first yaw as
`spawn_yaw` (what the session did for the FIT runs:
`~/ros2_ws/sim_runs/sim_acceptance/04_control/runs/routes/`).
