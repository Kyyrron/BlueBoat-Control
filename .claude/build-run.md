# BlueBoat-Control — build, launch, run, and the gates

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 3. Build, launch, run

```bash
colcon build && source /opt/ros/jazzy/setup.bash && source install/setup.bash   # from ~/ros2_ws
ros2 launch blueboat_control Sim_launch.py controller_type:='MPC' trajectory:='kin_square'
ros2 launch blueboat_control BlueBoat_launch.py enable_motors:=True controller_type:='PID' note:='testing_gains'
```

What the launch files do not make obvious:

- **`BlueBoat_launch.py`** always starts `mavros`, `robot_interface`, `uwgps_log`,
  `param_set`; `master_control` only with a non-empty `controller_type`; `path_generation`
  unless `use_pinger` (always in a session). `use_pinger` reaches `robot_interface` as
  **`use_UWgps`**, which also selects the CSV layout (§6).
- **`session_mode`** (default False; MCS always passes true): connect, go into override,
  **idle** — no following, no CSV, no `.npy` — until `start`; `stop` returns to idle in
  override. Each run gets its own CSV + sidecar + report and `.npy`; between runs
  `configure` may switch controller, follow mode and note. `path_generation` watches the
  station's session file (`trajectories/.session/run.yaml`), rewritten before every start.
- **Arm before override** (session only, for the superproject link guardian, which needs the
  autopilot armed): MANUAL → `default` (normal 73/74 mapping) → wait ~3.5 s disarmed (Rover
  sets the ±1000 thruster range only in its disarmed 1 Hz loop) → arm (2 s retries) →
  `override` (`arm_sequence_step`). Never blocking: after `arm_timeout` it proceeds; `rearm`
  retries. Parameters `arm_on_connect` (True, session only), `arm_timeout` (15 s),
  `arm_default_timeout` (25 s). The E-STOP still disarms.
- **Pinger** (2026-11-24): `BlueBoat_launch.py` always starts `pinger_estimator` (the one producer of
  `/blueboat/pinger_coordinates` and `/blueboat/pinger/*`). `pinger_mode` (`off|follow|log`, '' = from
  `use_pinger`), `has_path` (false: a run with no path, pinger mode off or log; no `path_generation` outside a session),
  `pinger_source` (`filter|legacy`), `pinger_preset`, `pinger_filter_yaml`, `allow_live_retune`
  (interface §2.7). `Replay_launch.py pinger_estimator:=true` re-estimates the pinger on the replayed
  inputs (the recorded pinger outputs then move to `/replay/*`).
- **`path_source`** (default `mission`): `replanner` makes runs replanner-owned (§4), in both
  launch files and per run through `configure`; with `use_pinger` it is a FATAL at launch.
  (2026-11-18, R-3) The simulator's `full_mission_launch.py` takes it too and passes it the same
  way, so a built world hosts a replanner-owned run (`path_source:=replanner`).
- **`record_bag`** (default True, session only): one ros2bag per run (§6);
  `robot_interface` also receives `trajectory` to snapshot a `from_yaml` run's path.
- **`Replay_launch.py`**: `bag` (run folder or bag), `world_dir` (empty = stock `ocean`),
  `rate`, `heave`, `heave_gate`, `start_paused`, `gui`, `placement` (`gps`/`origin`). Gazebo +
  clock and `set_pose` bridges + the boat with `replay:=true` (§8) + `bag_replay`; no
  controller, no sea state, no sonar, no `blueboat_sss_sim` import.
- **`Sim_launch.py` runs one of two simulations** (`sim_config`, 2026-11-13):
  `field_equivalent` (default) = identified plant + the real boat's column (`gain_profile`
  field) + thrust applied at 20 Hz + `/blueboat/odom` at the field timing; `legacy` =
  BlueROV2 plant + sim-tuned column + 10 Hz thrust + Gazebo's 20 Hz odometry, i.e. every
  simulation before that date. Single pieces: `plant`, `odom_profile`, `odom_seed`,
  `tuning_file` (a day's gains, `config/field_runs/`). Each piece defaults to field-equivalent
  on its own, so the simulator's `full_mission_launch.py` gets that set without forwarding
  anything; (2026-11-18) since 2026-11-15 it also forwards `sim_config` and its overrides (R-6, done by
  simulator session 03), and since 2026-11-18 `path_source` (R-3, simulator session 13). `field_equivalent` and the identified plant are frozen with the simulator
  (2026-11-18): no retuning of the plant.
- **`Sim_launch.py`** defaults `controller_type` to `'MPC'`, never starts `robot_interface`;
  `spawn_yaw` is radians ENU (position stays (0, 0)). For a `from_yaml:` trajectory it sizes
  `path_publisher`'s `total_time` as duration × 1.1 + 30 s (`_path_window`).

### Gates

No lint or type-check target. Scripts with exit codes, pytest-compatible:

```bash
/usr/bin/python3 test/test_path_update.py         # ROS-free path-update core (102 checks)
python3 test/test_yaw_wrap.py                     # heading at +-180 deg, MPC/PID/LoS (32 checks; needs what mc_harness needs)
/usr/bin/python3 test/test_path_update_log.py     # ROS-free pathlog + filing (24 checks)
python3 test/test_bag_replay_track.py             # 18 ROS-free checks; 29 with ROS + the recorded bag
/usr/bin/python3 test/test_pinger_filter.py       # pinger filter A1 + legacy bit-identity + review fixes (148 checks; recorded-bag checks need ~/ros2_ws/data)
/usr/bin/python3 test/test_yaml_trajectory_end.py # a non-loop path clamps (pose + heading), a loop wraps (21 checks)
python3 test/mc_harness.py --pinger-modes         # pinger modes A4: configure table, log w/o path = 0 /path_request, stale hold
python3 test/pinger_integration.py --mode log     # ROS graph: estimator -> robot_interface CSV, per-fix log, sidecar, smoothing (also --mode follow)
python3 test/gazebo_path_end.py --controllers PID,LoS,MPC   # Gazebo, headless, isolated: non-loop end held >= 60 s, loop wraps
QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py   # skips without a poslog under ~/ros2_ws/data
python3 .claude/tools/interface_inventory.py --check .claude/tools/interface_baseline.json  # 0 = unchanged
# deterministic tick-level harness (ROS sourced + install + venv with acados for MPC):
python3 test/mc_harness.py --compare HEAD --param gain_profile:=sim   # N11: HEAD vs working tree, LEGACY column, bit-identical
python3 test/mc_harness.py --one --scenario kin_square --controller MPC --plant identified --out /tmp/x.json  # offline run on the identified plant
python3 test/mc_harness.py --time-source ros      # time_source wall vs ros, bit-identical (one ns clock)
python3 test/mc_harness.py --replanner            # offline replanner scenarios, all controllers
python3 test/gazebo_path_update.py --replanner --out-dir /tmp/gz   # Gazebo, headless
```

`mc_harness --compare` runs `circle`, `kin_square`, `fsin`, `manual`, `arrow`, `session`
and `pinger` (PID/LoS only) for PID/LoS/MPC plus a real-parameter `kin_square` (23 runs),
requiring `/thruster_input`, `/monitoring_data`, `/controller_target` and τ bit-identical
tick by tick; `--scenarios replanner,replanner_session` adds the path-update runs. Since
2026-11-13 the gate is run with `--param gain_profile:=sim` (a simulation's default column is
now the real boat's; a revision that predates the parameter ignores it), and a run that
differs prints its first differing tick beside the first tick at which the measured yaw
wraps. The
interface guard (stdlib, also a `PostToolUse` hook) compares every publisher, subscriber,
service, client, declared parameter and the `blueboat_interfaces` field lists to a baseline;
it is **not committed** (`.claude/tools/` is gitignored), so a fresh clone has no contract
gate. A deliberate contract change: notify consumers, then `--update`.

**Interpreter — name it.** `~/ros2_ws/.venv` carries `acados_template`, `casadi`, `pandas`:
`master_control` and `simulation_interface` die without it. `/usr/bin/python3` has numpy,
scipy, matplotlib, PyYAML, `rclpy`, not those three. An interactive shell may resolve bare
`python3` to `SSS-Dataset-Aug-Studio/.venv`.

**Dependencies** (`requirements.txt`; apt: `xacro`, `simple_launch`, `mavros`,
`urdf_parser_py`, acados). `import ur_mpc` is at `master_control`'s module scope, so
acados is needed for **every** controller. acados also needs `libacados.so` and the Tera
renderer at `<acados>/bin/t_renderer` (without it codegen blocks on an invisible `input()`;
`master_control` preflights it and exits with a FATAL). Export `ACADOS_SOURCE_DIR`.

---
