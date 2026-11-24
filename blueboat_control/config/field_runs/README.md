# Field-run configurations

One ROS parameter file per field day and controller (PID: proportional-integral-derivative,
LoS: line of sight, MPC: model predictive control). Each file holds the controller gains
flown that day (only those that differ from today's defaults) and that day's odometry timing
(`odom_profile`). Use it to re-fly a field run in simulation with the controller as it was.

The rest comes from `sim_config:=field_equivalent` (the default of `Sim_launch.py`): the
identified BlueBoat plant, the real MPC model and 20 Hz thrust.

## Use

```bash
ros2 launch blueboat_control Sim_launch.py controller_type:=PID trajectory:=<route> \
  tuning_file:=$(ros2 pkg prefix blueboat_control)/share/blueboat_control/config/field_runs/2026_09_07_PID.yaml
```

`controller_type` must match the file. The launch's own values (controller, `path_source`,
data folder) always win over the file. An explicit `odom_profile:=` overrides the file's timing.
After adding or editing a file here, rebuild (`colcon build` from `~/ros2_ws`) so the installed
copy is updated.

## Files

| File | Day, place | Controller | Odometry |
|---|---|---|---|
| `2026_09_07_PID.yaml`, `_LoS`, `_MPC` | 2026-09-07, beach basin | PID / LoS / MPC | `field_3hz` |
| `2026_10_27_MPC_pre_retune.yaml` | 2026-10-27 | MPC (not retuned that day) | `field_4hz` |
| `2026_10_27_PID_pre_retune.yaml`, `_post_retune` | 2026-10-27 | PID | `field_4hz` |
| `2026_10_27_LoS_pre_retune.yaml`, `_post_retune` | 2026-10-27 | LoS | `field_4hz` |
| `2026_10_28_PID.yaml`, `_LoS`, `_MPC` | 2026-10-28, pool | PID / LoS / MPC | `field_13hz` |

Each file's header says where its values come from and how certain they are.

## Which file for which run

**2026-09-07** (runs in `~/ros2_ws/data/Robot_data/Sept_data/ControllersComparison/`): the file
of the run's controller. The four routes are the designer files under
`~/.config/blueboat_mcs/trajectories/`, authored at 0.5 m/s:

| Run names | Route file |
|---|---|
| `PidSin2Real`, `LosSin`, `MPCSin` | `c_plage_sin.yaml` (sine weave) |
| `SquarePID`, `SquareLoS`, `SquareMPC` | `c_plage_square.yaml` (expanding square) |
| `PIDNonRegular`, `LoSNonRegular`, `MPCNonRegular` | `c_plage_non_regular.yaml` (tough angles) |
| `LawnmowerPID`, `LawnmowerLoS`, `LawnmowerMPC` | `c_plage_lawnmower.yaml` (lawnmower; it was rewritten later, so `.deployed/plage_lawnmower.yaml` may be the version flown) |

These runs saved no copy of their path. To re-fly one with a plain `Sim_launch.py`, shift the
route so its first point is the origin, and pass its first heading as `spawn_yaw` (radians, east-north-up convention: 0 = East).

**2026-10-27 and 2026-10-28**: use the run's own `-path.yaml` (in its run folder) as the route.
- MPC on 2026-10-27: `2026_10_27_MPC_pre_retune.yaml`. Run `10_44_08` needs `odom_profile:=field_13hz`.
- PID / LoS on 2026-10-27: it is **not known** whether a run flew the pre- or post-retune gains.
  Try both files.
- 2026-10-28: the odometry rate varied by run (4 to 16 Hz); add `odom_profile:=field_4hz` for the
  runs at 4 Hz.

## Limit

A file restores gains and timing, not the code. Every run before 2026-11-13 flew an MPC that had
a heading fault at ±180° (one full differential-thrust pulse per crossing); all twelve runs of
2026-09-07 crossed it. Today's code does not reproduce that.
