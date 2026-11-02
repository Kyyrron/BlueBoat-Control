# CLAUDE.md — BlueBoat-Control

Working guidance for this submodule. Read §1 before editing anything.

Deeper references: `blueboat_control/src/TRAJECTORY_SYSTEM.md` (where the reference comes
from: trajectory library, τ, governor), `FIELD_TUNING.md` (every tuning knob, real/sim
defaults, symptom→knob index), `log_reviewer/README.md`. Nested `CLAUDE.md` files load in
their directory: `blueboat_control/src/` (measurements and derivations behind §4–§5),
`blueboat_description/` (§8), `log_reviewer/` (§9). The **live path update** (the
BlueBoat-Replanner entry point) is documented in `docs/`: `PATH_UPDATE_DESIGN.md` (why),
`PATH_UPDATE_INTERFACE.md` (the client contract), `REFERENCES.md`, `REQUESTS.md` (what other
modules need), `PATH_UPDATE_REAL_WATER_TEST.md`. Open items: `.claude/TODO.md`.

Frozen at `v1.0` and **add-only** (superproject §3): additions, or refactors shown identical
by the gates of §3.

---

## 0. What this module is

The platform control stack of a BlueRobotics BlueBoat USV: MAVROS/ArduPilot bridge, thruster
driver, trajectory generation and three interchangeable controllers (MPC, PID, LoS). The
sonar, perception and planning stack consumes its topics and runs unmodified against
simulation and hardware. ROS 2 **Jazzy**, Python nodes throughout.

| Package | Build type | Role |
|---|---|---|
| `blueboat_control` | ament_cmake (+ `ament_python_install_package`) | All nodes, controllers, trajectories, launch files |
| `blueboat_description` | ament_cmake | URDF/xacro, meshes, Gazebo world and spawn launch (§8) |
| `blueboat_interfaces` | ament_cmake (rosidl) | `srv/RequestPath.srv`, `srv/UpdatePath.srv`, `msg/OmniscanProfile.msg`, `msg/ProcessedSSSPing.msg` — one `rosidl_generate_interfaces` call; the two `.msg` serve the sonar project, no node here uses them |

`log_reviewer/` is not a package: a standalone PySide6 app with a `COLCON_IGNORE` (§9).

---

## 1. NON-NEGOTIABLES

Do not change any of these without checking their downstream consequences.

**N1 — Never change the name or message type of any topic, service or parameter.** This
module is a drop-in interface for the perception/planning stack, the Mission Control Station
and the GCS. Refactors change file internals only; a signature change is a cross-repo
decision.

**N2 — Simulation and real water expose the identical ROS interface** (one sanctioned
exception: run-session plumbing, §2.3). Divergence invalidates the sim-to-real comparison.

**N3 — MAVROS twist is body-frame and must not be rotated.** `/mavros/local_position/odom`
has `child_frame_id: base_link`: `twist` is surge/sway/yaw-rate, `pose` world-frame (the ENU
velocity is `/mavros/local_position/velocity_local`). `robot_interface.odom_callback`
translates the pose position to the launch point (axes ENU, yaw absolute ENU) and passes the
twist through. Two consumers depend on it: `master_control` reads `current_twist[0]` as surge
(PID/LoS inner speed loop), and the pinger dead reckoning subtracts `vel + ω × p` from
body-frame pinger coordinates. Measured against mavros 2.14.0 (a synthetic FCU at 1 m/s north
reads (0.707, 0.707) at heading NE in the odom twist); rotating by −`yaw0` would be wrong.

**N4 — `enable_motors` gates thruster output; a closed gate HOLDS NEUTRAL.** The gate is in
`robot_interface.manualMove`: no thrust-bearing PWM reaches the motors unless
`enable_motors:=True`. Every other `/mavros/rc/override` write carries neutral 1500/1500 or
channel release (`full_stop` → `manualMove([0,0], force=True)`; `mode_callback` neutral then
release on leaving override). The closed gate streams neutral while `mode == 'override'`:
going silent let the ESCs follow RC channels 1/3 from any transmitter (override puts
SERVO1/3 on passthrough at init), and 0 N is an exact table knot (1500 µs, and
`3000 − 1500` for the reversed side). Any new thrust-bearing bypass violates this (CM-16).

**N4b — `'stop'` latches.** `full_stop()` sets `estopped`, cleared only by `'enable'`
(`release_estop`). While latched the timer re-publishes `controller_ready=False` at the
readiness rate and re-zeroes through the **unforced** `manualMove([0,0])` every tick, so the
next `/thruster_input` cannot undo the stop. `timer_callback` never passes `force=True`.

**N5 — Restore the default servo mapping before shutdown; never set a SERVO function to 0.**
`override` remaps SERVO1/3 to RC passthrough (the Xbox controller stops working); `0`
(Disabled) causes a PreArm "no motor" failure (`param_set.SERVO_DISABLED` is never applied).
Both nodes attempt the restore themselves in independent `try/finally` blocks
(`param_set.restore_default_blocking`, ~3 s; `robot_interface.shutdown`: motors, RC release,
`default`, CSV close, report) — best effort, since a launch teardown SIGINTs mavros too. An
operator `default` before shutdown stays the reliable route (CM-15).

**N6 — Thrust is streamed as RC override, never as per-tick acknowledged MAVLink commands.**
`OverrideRCIn` at ~20 Hz is latest-wins and feeds ArduPilot's `RC_OVERRIDE_TIME`; per-tick
`/mavros/cmd/command` calls stall the command plugin for seconds on one lost ACK.
`set_servo()` is a documented legacy fallback called from nowhere.

**N7 — Controller iteration happens offline** (rosbags, the harness of §3), never in a field
session.

**N8 — The reference advances from the boat's measured progress, never from wall-clock
time.** The governor (`path_progress_errors` + `advance_governor`, §4) enforces it.
`current_time` only timestamps `/monitoring_data` and the `.npy` (from the run's start in a
session); never route it back into path generation.

**N9 — Monitoring output is world-frame in every controller branch, and means the same
quantity.** `/monitoring_data` and the `.npy` carry world-frame `x_d, y_d, psi_d` for
manual, pinger, LoS, PID, MPC, arrows and the replanner hold; each capture site is marked
`# --- world-frame monitoring target ---`. Path following logs `_reference_pose(path)` (the
window's **first** pose), never `cf.compute_target`'s result (the second pose, which the
PID/LoS laws need for velocities). `/controller_target` stays body-frame and pinger-only —
a different signal, not to be unified.

**N10 — A path update never touches safety or operator state.** `/blueboat/update_path`
changes only the reference of a replanner-owned run. It never re-enables motors, releases an
E-STOP, starts a run or releases a manual target / arrow control, and is refused while any of
them is engaged (`not_ready`, `no_run`, `operator_override`). A refusal changes nothing.

**N11 — Fixed missions stay bit-identical.** `path_source=mission` (the default) behaves
exactly as before the live path update. Proof: `test/mc_harness.py --compare <rev>` (§3).
Run it after any change to `master_control.py` or what it imports.

---

## 2. Interface — the authoritative contract

Names are written **absolutely** in the source; `master_control` has `namespace='blueboat'`,
which ROS 2 does not apply to absolute names — rewriting them relative would rename half the
interface (N1). Relative names: `path_publisher` (`set_path`, `path_request`, root namespace)
and the `ROV` helper in `blueboat_control/__init__.py` (`odom`, `joint_states`,
`robot_description`, `cmd_<thruster>`, `cmd_<joint>`, display-only `blueboat_<thruster>_wrench`
and `blueboat_base`, all under `blueboat`). Cross-module behaviour of these topics (QoS of
`controller_ready`, `/thruster_input` never silent, the 3-value body-frame pinger vector) is
in the superproject `CLAUDE.md` §4.1.

### 2.1 Nodes

All executables are installed **flat** into `lib/blueboat_control` (`CMakeLists.txt`), so
sources import each other as bare modules (`import custom_functions`, `import PID`).

| Executable | Source under `blueboat_control/` | Node | Purpose |
|---|---|---|---|
| `master_control.py` | `src/` | `master_control` (ns `blueboat`) | MPC / PID / LoS; serves `/blueboat/update_path` |
| `simulation_interface.py` | `src/` | `pid_sim` (ns `blueboat`) | Gazebo thrust bridge via `ROV`; sim readiness; position CSV (no-pinger layout) |
| `robot_interface.py` | `src/robot_interaction/` | `blueboat_controller` | MAVROS bridge, thrust→PWM, odom republish, CSV, run bags |
| `param_set.py` | `src/robot_interaction/` | `blueboat_parameter_control` | SERVO function + GCS sysid remapping; watchdogged |
| `uwgps_log.py` | `src/robot_interaction/` | `underwater_gps_logger` | Water Linked UGPS HTTP poller |
| `path_generation.py` | `src/_custom_libraries/` | `path_generation` | `/path_request` server; trajectory library |
| `path_publisher.py` | `src/_custom_libraries/` | `path_publisher` | Whole-path preview for RViz |
| `bag_replay.py` | `src/` | `bag_replay` | Kinematic Gazebo replay of one run's bag (§2.3) |
| `MPC/ur_mpc_control.py`, `MPC/uvr_mpc_control.py` | `src/MPC/` | `mpc_control` (ns `blueboat`) | Standalone MPC nodes (2 / 3 thrusters) — installed, launched by nothing |

**ROS-free by construction** (numpy or stdlib only; keep them so — anything needing `rclpy`
goes in `custom_functions.py`): `frame_math.py`, `robot_log_schema.py`, `thrust_limits.py`,
`path_stamp.py`, `poslog_report.py`, `run_bag.py`, `path_update.py`, `path_update_log.py`,
`replay_attitude.py`, `yaml_trajectory.py` (all `src/_custom_libraries/`), plus `MPC/ur_mpc.py`
(acados, no ROS).

`poslog_report.py` (also a CLI) files a finished run: creates `Robot_data/<csv stem>/` and
**moves** the CSV, its `-origin.yaml`, a PNG report and, from a session, the `-path.yaml`
snapshot and the bag in. Moving is allowed, rewriting is not (CM-7): an existing folder is
never touched. matplotlib is imported **lazily** (a flight node must never fail on a missing
plotting library). It decides the plotting frame for itself and the log reviewer (a simulated
run is local ENU metres, never lat/lon). Speeds above `MAX_PLAUSIBLE_SPEED_MS` (5 m/s) are
pose discontinuities: excluded from every statistic and counted, never clipped.

**File organisation.** `master_control.py`, `robot_interface.py` and `path_generation.py`
open with a FILE MAP and banner-numbered sections; `master_control.timer_callback` delegates
to `_mission_reference_step` / `_replanner_reference_step`, `_compute_command`
(`_path_following_command`, `_pinger_command`) and `_record_monitoring`. Method order may
change; what file a thing lives in may not (N1). The `fsin` table must stay at
`path_generation`'s module scope (its functions rebind the globals with `global`).

### 2.2 Internal topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface` (real) · Gazebo bridge (sim, §8) · `bag_replay` | `master_control`, `simulation_interface` — **local ENU** on both: origin = launch point (real) / Gazebo origin (sim), yaw **absolute ENU** |
| `/blueboat/pinger_coordinates` | `Float32MultiArray` | `robot_interface` | `master_control` |
| `/blueboat/controller_ready` | `Bool` | `robot_interface` · `simulation_interface` | `master_control` |
| `/thruster_input` | `Float32MultiArray` `[right, left]` N | `master_control` | `robot_interface`, `simulation_interface` |
| `/controller_target` | `Float32MultiArray` | `master_control` — **pinger branch only** | `robot_interface` (stored, never read) |
| `/monitoring_data` | `Float32MultiArray` | `master_control` (`simulation_interface` creates a publisher, never publishes) | `robot_interface`, `simulation_interface` (CSV target columns) |
| `/blueboat/param_str` | `String` | `robot_interface` | `param_set` |
| `/blueboat/param_ready` | `Bool` | `param_set` | `robot_interface` |
| `/blueboat/param_mode` | `String` | `param_set` | `robot_interface` |
| `/uw_gps_data` | `Float32MultiArray` | `uwgps_log` | `robot_interface` |
| `/blueboat/run_state` | `String` (JSON) | `master_control` — session only | `robot_interface`, MCS |

### 2.3 External-facing topics

| Topic | Type | Dir | This side | Other party |
|---|---|---|---|---|
| `/blueboat/input_str` | `String` | in | `robot_interface` | Operator CLI, MCS (§2.6) |
| `/blueboat/manual_target` | `Float32MultiArray` `[x, y]` world | in | `master_control` | MCS, GCS (`[0, 0]` = resume sentinel) |
| `/monitoring_data` | `Float32MultiArray` | out | `master_control` | MCS map display |
| `/pose_arrow` | `visualization_msgs/Marker` | out | `master_control` | RViz (simulation only) |
| `/set_path` | `nav_msgs/Path` | out | `path_publisher` | RViz / GCS |
| `/blueboat/arrow_thrust` | `Float32MultiArray` | in | `master_control` | MCS ARROW CONTROL: `[right, left]` N at 20 Hz, `[]` = release (every mode) |
| `/blueboat/run_command` | `String` (JSON) | in | `master_control` — session only | MCS: `configure` / `start` / `stop` |
| `/blueboat/run_state` | `String` (JSON) | out | `master_control` — session only | MCS (ack + 1 Hz heartbeat) |
| `/blueboat/run_log` | `String` (JSON) | out | `robot_interface` — session only | MCS (the run's CSV, 1 Hz heartbeat) |
| `/blueboat/update_path` | srv `blueboat_interfaces/srv/UpdatePath` | in | `master_control` — every launch | Replanner, tests: live path update (§4) |
| `/blueboat/path_progress` | `String` (JSON `blueboat_path_progress/1`) | out | `master_control` — every launch, 10 Hz + on change | Replanner, MCS: tau, state, ids, `min_change_point` |
| `/blueboat/path_update_event` | `String` (JSON `blueboat_path_update_event/1`), reliable depth 50 | out | `master_control` | `robot_interface` / `simulation_interface` (the run's `-pathlog.csv`), the bag |

**Run sessions** (`session_mode:=true`, every station real-robot launch). The four
"session only" rows exist only then and only on the real graph; `Sim_launch.py` and a CLI
launch without the argument expose the v1.0 interface (operator plumbing, not a sensor
interface — CM-2). Contracts:

```
run_command  {"cmd":"configure","controller_type":"PID|LoS|MPC","use_pinger":b,"note":s,"mission":s,"seq":n,
              "path_source":"mission|replanner"}                     # path_source optional, additive
             {"cmd":"start","seq":n}   {"cmd":"stop","seq":n}
run_state    {"seq":n,"ok":b,"detail":s,"state":"idle|loading|running|error","controller_type":s,
              "use_pinger":b,"note":s,"mission":s,"run":n,"npy":path,"path_source":s}
run_log      {"run":n|null,"logging":b,"csv":path,"layout":"no_pinger|pinger",
              "bag_enabled":b,"bag_recording":b,"bag":path}          # bag_*: additive (v1.1)
```

`seq` echoes the last command handled; `configure` is refused during a run (and MPC with the
pinger, and `replanner` with the pinger), `start` unless `state == idle`. All four topics use
the default profile (depth 10, reliable, volatile) at both ends (CM-4); both states are
heartbeated at 1 Hz because one-shots race DDS discovery.

**Bag replay** (`Replay_launch.py` → `bag_replay`): a playback of one real run, never a
simulation (CM-2).

| Topic | Type | Contract |
|---|---|---|
| `/replay/state` | `String` (JSON), out, TRANSIENT_LOCAL depth 1, 2 Hz | `{bag, t, duration, rate, paused, finished, frame: empty\|anchored, world, world_offset, pose_source, gazebo}` + additive `{placement, attitude_source, attitude_rate_hz, attitude_interp, heave: replayed\|pinned_noisy\|off, boat_pose}` |
| `/sim/sea_state/boat_pose` | `geometry_msgs/TransformStamped`, out, 100 Hz, reliable depth 50 (only with `publish_boat_pose`, set by every `Replay_launch.py`) | the replayed pose (frame `world`, child `blueboat`, sim-time stamp, z on the simulator's datum `boat_pose_waterline_z` −0.111) — what `sss_sim_node` renders every ping from |
| `/replay/command` | `String` (JSON), in | `{"cmd":"pause"\|"resume"}`, `{"cmd":"rate","rate":x}` |
| `/blueboat/odom` | `Odometry`, out | rebuilt from the recorded `/mavros/local_position/odom`: **full** attitude, body twist, world frame |
| `/monitoring_data`, `/set_path` | as live, out | targets shifted into the world frame; the path from the run's `-path.yaml` |
| live names | as recorded, out | every other `/mavros/*` but `rc/override`, `/thruster_input`, `/controller_target`, `/blueboat/pinger_coordinates`, `/uw_gps_data` |
| `/replay/<name>` | as recorded, out | everything else (commands, session plumbing, the raw odom) — never on a live name |

Every `header.stamp` is rewritten to replay time; the bag's **receive** times drive the
schedule (`BagClock`, pausable). Frames: the empty world is the run's launch frame; a built
world is placed by the median of (recorded fix in the anchor's EN − boat in the launch frame)
over the bag's fixes, or on the world origin with `placement:=origin`. Attitude
(`attitude_source:=auto`): the autopilot's ATTITUDE from `/uas1/mavlink_source`, timed by
`time_boot_ms` (WiFi delay removed), else the v1.1 source; `attitude_interp:=hermite` (cubic,
body rates as tangents; `slerp` = v1.1 bit for bit). Heave: detrended EKF z, replayed only if
its std ≤ `heave_max_std_m` (0.10) and ≤ 5 % clipped (`heave_gate`), else the waterline is
pinned. The replay never starts a sonar; the operator (or the GCS) starts
`sss_sim_launch.py world_dir:=<world> with_mavros_shim:=false auto_ping:=false`. Maths in
`replay_attitude.py` (ROS-free); evidence in `blueboat_control/src/CLAUDE.md`; tests
`test/test_bag_replay_track.py`.

### 2.4 MAVROS boundary

**`robot_interface`** subscribes `/mavros/state` (default QoS), `/mavros/imu/data`,
`/mavros/local_position/odom`, `/mavros/global_position/global` (the last three BEST_EFFORT
depth 10); publishes `/mavros/rc/override`; clients `/mavros/cmd/arming`,
`/mavros/set_mode`, `/mavros/cmd/command`. **`param_set`** owns `/mavros/param/pull` and
`/mavros/param/{get,set}_parameters` (rcl_interfaces on the mavros node), never blocks on
service availability in its constructor (checked per request; `robot_interface` re-sends).
FCU endpoint: `BlueBoat_launch.py`'s `fcu_url` (default `udp://:14550@192.168.2.2:14550`;
collides with a running QGroundControl — superproject §4.4).

### 2.5 Services

`/blueboat/update_path` (`UpdatePath`, served by `master_control`): contract in
`docs/PATH_UPDATE_INTERFACE.md`, summary in §4.

`/path_request` (`RequestPath`): request `Float32MultiArray path_request` = path-parameter
values; response `nav_msgs/Path`, one pose per value, `frame_id: "world"`. Server
`path_generation`; clients `master_control`, `path_publisher`, both standalone MPC nodes.
Deliberately parameter-agnostic — keep it so.

**Exactly one server may run, enforced twice** (with two, every request is answered by
whichever replies first and the reference alternated between trajectories — measured):
* `path_generation` refuses to be the second server: before creating the service it
  enumerates the graph (no self-exclusion — the duplicate has its very name) and exits with
  a FATAL (`allow_duplicate_server:=true` opts out; `server_discovery_wait` 2.0 s).
* each pose is **stamped with its path parameter** (`path_stamp.encode`) and
  `master_control.accept_path` rejects a response whose pose count or `poses[0]` stamp does
  not match its request (`RequestPath` untouched). A server predating the stamp is detected
  once ("rebuild"), then a geometric plausibility check applies.

`master_control` also logs an error at startup if it sees more than one server.

### 2.6 Operator CLI

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: <value>"
```

`enable` · `disable` · `stop` · `override` · `default` · `arm` · `disarm` · `rearm` (session
only: `default` mapping → arm → `override`; refused during a run or while latched) · `manual`
(autopilot mode → MANUAL; the station sends it after `default` when the guardian was turned
OFF) · `move <left> <right> <seconds>` (stored `[right, left]`; only without a
`controller_type`). `arm` works only in `default` (Rover refuses to arm on passthrough). An
unrecognised first token falls through to `move_callback`, which needs exactly four fields
(`x 1.0 1.0 5` moves; `1.0 1.0 5` does not); an empty message is ignored.

```bash
ros2 topic pub --once /blueboat/run_command std_msgs/msg/String '{data: "{\"cmd\": \"start\", \"seq\": 1}"}'
```

---

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
- **`path_source`** (default `mission`): `replanner` makes runs replanner-owned (§4), in both
  launch files and per run through `configure`; with `use_pinger` it is a FATAL at launch.
- **`record_bag`** (default True, session only): one ros2bag per run (§6);
  `robot_interface` also receives `trajectory` to snapshot a `from_yaml` run's path.
- **`Replay_launch.py`**: `bag` (run folder or bag), `world_dir` (empty = stock `ocean`),
  `rate`, `heave`, `heave_gate`, `start_paused`, `gui`, `placement` (`gps`/`origin`). Gazebo +
  clock and `set_pose` bridges + the boat with `replay:=true` (§8) + `bag_replay`; no
  controller, no sea state, no sonar, no `blueboat_sss_sim` import.
- **`Sim_launch.py`** defaults `controller_type` to `'MPC'`, never starts `robot_interface`;
  `spawn_yaw` is radians ENU (position stays (0, 0)). For a `from_yaml:` trajectory it sizes
  `path_publisher`'s `total_time` as duration × 1.1 + 30 s (`_path_window`).

### Gates

No lint or type-check target. Scripts with exit codes, pytest-compatible:

```bash
/usr/bin/python3 test/test_path_update.py         # ROS-free path-update core (102 checks)
/usr/bin/python3 test/test_path_update_log.py     # ROS-free pathlog + filing (24 checks)
python3 test/test_bag_replay_track.py             # 18 ROS-free checks; 29 with ROS + the recorded bag
QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py   # skips without a poslog under ~/ros2_ws/data
python3 .claude/tools/interface_inventory.py --check .claude/tools/interface_baseline.json  # 0 = unchanged
# deterministic tick-level harness (ROS sourced + install + venv with acados for MPC):
python3 test/mc_harness.py --compare HEAD         # N11: HEAD vs working tree, bit-identical
python3 test/mc_harness.py --replanner            # offline replanner scenarios, all controllers
python3 test/gazebo_path_update.py --replanner --out-dir /tmp/gz   # Gazebo, headless
```

`mc_harness --compare` runs `circle`, `kin_square`, `fsin`, `manual`, `arrow`, `session`
and `pinger` (PID/LoS only) for PID/LoS/MPC plus a real-parameter `kin_square` (23 runs),
requiring `/thruster_input`, `/monitoring_data`, `/controller_target` and τ bit-identical
tick by tick; `--scenarios replanner,replanner_session` adds the path-update runs. The
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

## 4. Control architecture

Branch priority: **arrow keys** → **manual target** → **path following** → **replanner
awaiting-hold** → **pinger** → nothing (zero thrust). MPC is unsupported in pinger mode. In
a session path and pinger following run only between `start` and `stop`. Evidence for
everything here: `blueboat_control/src/CLAUDE.md`; knobs: `FIELD_TUNING.md`.

- **The controller is built in `__init__`**, never on the first tick (an acados build there
  froze the single-threaded executor). `Controller node initiated` = construction;
  `Controller ready` = readiness. A session `configure` rebuilds through the same code
  (`_apply_controller_setup` + `_make_controller`): PID/LoS inline, **MPC on a worker
  thread** swapped in by `_poll_controller_build` (state `loading`, `start` refused), so the
  loop never stops.
- **Arrow keys override everything**: out through `publish_thrust` (this node stays the only
  `/thruster_input` producer), τ frozen, the logged target held. Older than `arrow_timeout`
  (0.5 s) → zero, still engaged. Released by `[]`, a manual target or a run `start`.
- **acados solver cache**: `$ROS_HOME/blueboat_control/mpc` (`ur_mpc.acados_build_dir`),
  never under `cf.data_root()`. Any change to model coefficients, horizon, weights,
  `thrust_limit` or `mpc_qp_iter_max` regenerates it once (~1 min, logs `generated and
  compiled`) — expected; do not delete the cache.
- **A failed acados solve is not a command (C6).** Non-zero status → reset, cold re-seed,
  one retry, else zeros; reported through the ROS logger (never `print`); no early return,
  so `/thruster_input` and the `.npy` keep flowing. `mpc_qp_iter_max` 0 derives
  `max(50, 4·nu·N)`.
- **Live path update.** A run is **fixed-mission** (`path_source=mission`) or
  **replanner-owned**. A replanner run holds its own plan (`path_update.PathPlan`, evaluated
  by `YamlTrajectory.from_points`), never calls `/path_request`, starts holding at the boat
  (keep-location law, `solve_LoS(keep_location=True)`), takes every path through
  `/blueboat/update_path` and composes each window locally (`_replanner_reference_step`) —
  governor and controllers unchanged. Acceptance, in order: state gate (`not_ready`,
  `no_run`, `mission_owned`, `operator_override`), request validity, change-point range,
  lead `c ≥ τ + max(path_time + 2·dt, gov_Lmax/U_d)`, continuity at c (0.20 m, 10°; heading
  skipped where the reference is stationary). Run boundaries clear the plan. Thresholds:
  `path_update_*` parameters. Handling ≤ 2.2 ms worst case (20 000 samples). N10, N11.
- **Reference generation** (N8): the governor's along/cross factors are clipped to `[0, 1]`
  and multiplied, so τ is monotonic and never faster than authored speed — nothing may scale
  `tau_dot` above unity. `fac_cross` is **off** (`gov_Emax = 0`): throttling on an error the
  inner loops cannot close is positive feedback. The request
  `linspace(tau, tau + path_time, path_steps)` is asynchronous (window 1–2 ticks stale), and
  τ advances only against a **fresh** window (`path_stale_timeout`). `path_time` /
  `path_steps` are **derived**, never declared: PID/LoS `control_dt` / 2, MPC `mpc_time` /
  `mpc_horizon + 1` (6.0 s / 31 sim, 2.5 s / 16 real).
- **Which law runs where:** manual target → `solve_LoS` (every `controller_type`), then
  `manual_keep_location` once reached; path following → MPC `ur_mpc.MPCController.solve`,
  PID `PIDLoS.compute` (with `u_ff`, `psi_path`), LoS `los_guidance`; pinger → PID
  `PIDLoS.compute(state, target)` body-frame with `psi_path=None`, LoS `solve_LoS`.
  `solve_LoS` is a crude proportional pursuit law, not the path LoS, and works as-is.
  `PIDLoS` is always built with `lookahead = pid_lookahead = 2.5 m`.
- **Station-keeping hold** (zero authored speed): gate `w = 1 − U_d/hold_speed` plus
  `hold_radius`. `w` is exactly zero on every library shape (slowest: `sin` at 0.280 m/s),
  so raising `hold_speed` above 0.28 would alter path following.
- **Tuning knobs** are all declared unconditionally in `_declare_tuning_parameters` and read
  once at construction; gain triples are double arrays. Several defaults are **split
  simulation/real** (PID gains, point gains, LoS speed scale, MPC horizon/time/R and plant
  model, `manual_hold_kx` — N/m). Also: `path_request_timeout` / `path_stale_timeout`
  (1.0 s), `thruster_input_timeout` (0.5 s, both interface nodes), `param_sequence_timeout_s`
  (20 s, `param_set`).

---

## 5. Thrust path — sharp edges

Evidence and tables: `blueboat_control/src/CLAUDE.md` §5.

- `/thruster_input` is **`[right, left]`** on every code path (allocation matrix, URDF
  thruster placement, `ROV.read_model`'s alphabetical joints, `simulation_interface`,
  `solve_LoS`, `manualMove`, the CLI's `move <left> <right> <s>`).
- `left_pwm = 3000 − pwm`: the left thruster is reversed for an asymmetric propeller —
  intentional.
- `manualMove`'s `compensation_gain` conditional (1.2 / 0.75) is overridden by a hard-coded
  `1.0`, and keys on `input[1]` while scaling `input[0]`. Dead — do not tidy without
  deciding what it should do.
- Thrust→PWM is a `PchipInterpolator` on a measured bollard-pull table
  (`cf.generate_interpolator`): about −27.6 N to +55.2 N; PWM clamps to `[1100, 1900]`.
- **Saturation is uniform, and one number**: every exit (`ThrustAllocator.allocate`,
  `publish_thrust`, `manualMove`, `simulation_interface`) scales the pair by one factor to
  `thrust_limit` (20.0 N; `thrust_limits.scale_to_limit`). Never per-side clip (it rewrites
  the wrench: 27 N of yaw collapsed to 2 N). `publish_thrust` returns the saturated vector,
  so monitoring, `.npy` and CSV agree with the wire.
- **0–2 N does not turn the propellers** (T200 ESC deadband). Only `solve_LoS`'s real-boat
  pinger surge stays under 2 N beyond 2 m, so only it is floored:
  `min_thrust · g · max(0, cos(bearing))` (both factors only reduce it; the yaw
  differential is untouched; `min_thrust = 0.0` restores the law). The boat parks at
  ~0.81 m by design.
- **A reached manual target is held**: `manual_keep_location` re-evaluates every tick (on
  station inside `manual_hold_radius`, pursuit beyond `manual_reacquire_radius`,
  proportional capped floored surge between); yaw untouched; `manual_hold_radius <= 0`
  restores the old latch bit-identically.
- **Loss of reference zeroes thrust at both ends**: `master_control` publishes `[0, 0]` on
  every early return; both interface nodes zero a command older than
  `thruster_input_timeout` (0.5 s). The watchdog does not disarm (`full_stop` stays the
  operator's), releases when commands resume, and on the real boat goes through the
  unforced `manualMove` (behind the N4 gate). The `controller_type == ''` `move` timeout
  owns that case.
- **`param_set`**: `override` maps SERVO1/3 to passthrough (51/53) and `SYSID_MYGCS` /
  `MAV_GCS_SYSID` (resolved at run time) to mavros (1); `default` restores 74/73 and 255.
  Every write is read back before `param_ready`. It never blocks forever and always reports
  (three rules in its module docstring). Because `param_mode` is heartbeated, a repeat is
  **not** an acknowledgement: MCS safe-shutdown requires a transition into `default`, and
  `robot_interface.mode_callback` / `param_callback` are edge-triggered.
- **Readiness handshakes survive discovery races**: both interface nodes re-publish
  `/blueboat/controller_ready` every 1.0 s; `robot_interface` re-requests the mode every
  second until `param_set` confirms (handled idempotently). Keep the pairing.

---

## 6. Data

| Artifact | Path | Nature |
|---|---|---|
| Position/pinger CSV | `<root>/data/Robot_data/{date}-{note}-poslog.csv` | **Raw field record** (`robot_interface` real, `simulation_interface` Gazebo) |
| Controller log | `<root>/data/{ctrl}_data/{date}-{ctrl}_{sim}_data.npy` | Per run |
| Run bag | `<root>/data/Robot_data/<csv stem>/{date}-{note}-bag/` (mcap) | **Raw field record**, sessions only: every topic except `/side_scan_sonar/*`, `/sss_processor/*`, `/sss_ai/*` |
| Path snapshot | `<root>/data/Robot_data/<csv stem>/{date}-{note}-path.yaml` | Copy of a path run's `from_yaml` mission; not for replanner runs |
| Path-update log | `<root>/data/Robot_data/<csv stem>/{date}-{note}-pathlog.csv` | **Raw field record**: one row per `/blueboat/path_update_event`, 3 header rows (`path_update_log.LEGEND`); only if the run received a request; never for the pinger layout |

**Per run in a session**: nothing is claimed until `start`; `master_control` reserves the
`.npy` at `start`, `robot_interface` opens the CSV when `run_state` reports the run (layout
from its follow mode). At `stop` the `.npy` is written and the CSV filed by
`poslog_report.py` **in a subprocess** (own session; a render never holds the GIL against the
20 Hz RC stream). Every run's sidecar carries the same session origin.

**The run bag**: a separate `ros2 bag record` process (`run_bag.py`) on the laptop, never in
`robot_interface`'s interpreter. Pre-armed **paused** while idle (hidden
`Robot_data/.run-bag-<pid>-<k>/`), resumed via `~/resume` at run start (first message within
1 ms), stopped at `stop` on a worker thread (SIGINT 10 s → SIGTERM → SIGKILL), then moved in
by `poslog_report.py <csv> --bag <staging>` (the destination must not exist — write-once).
A launch teardown SIGINTs it too; the shutdown hook waits (bounded) and files the open run.
A recorder that never recorded is deleted; a partial bag is kept (`ros2 bag reindex`).
Services are not recorded. ⚠ simple_launch gives mavros, `param_set` and `path_generation`
`use_sim_time:=True` on the real launch, so their stamps may be 0: use receive timestamps.

`<root>` = `custom_functions.data_root`: `data_dir` parameter → `$BLUEBOAT_DATA_DIR` → the
sourced workspace (normally `~/ros2_ws/data/`) → the working directory. An unwritable root
fails the launch naming the path; names are claimed with `O_EXCL` (`reserve_run_file`), so
same-second runs get `-2`, `-3` (CM-7). Read back only by `poslog_report.py` and the log
reviewer.

**`.npy` schema** `['t','x','y','psi','x_d','y_d','psi_d','u1','u2']` (`NPY_HEADER`),
targets world-frame (N9). The header is a row of **strings** in the same array, so `np.save`
coerces everything to strings — cast back on load. Written by `save_monitoring` on a 2 s
timer and at shutdown, never from the control tick.

**CSV.** Two layouts by `use_UWgps`: **no pinger, 27 columns** and **pinger, 39 columns**,
sharing the first 19 (`robot_log_schema.py`'s docstring is the column list and revision
record). Rows are filled **by column name**. Two **legend rows** (description, unit) precede
the column names, written at file creation (a killed run keeps them); older logs have none
and are never rewritten — `robot_log_schema.split_header` finds the header row in both,
pandas reads a new log with `header=2`. No description or unit may contain a comma. In the
pinger layout `ant_*` is all-zero and `dep` duplicates `aco_z`. `actuation_state`: `0` motors
disabled, `1` live, `2` enabled but not in override, `3` watchdog forcing zero. The
`<stem>-origin.yaml` sidecar carries `latitude`, `longitude` (the local-ENU frame's origin,
latched at the first odom) and `yaw0_rad` (provenance only; since 2026-08-31 `relative_psi`
is absolute ENU yaw, before it was `yaw − yaw0`).

**Simulation writes the same CSV** (`simulation_interface`, always no-pinger): date columns
are **sim time** (1970-01-01), `lin_acc_*` is differentiated odometry (no gravity),
`actuation_state` only 1 or 3, GPS (0, 0) unless something publishes
`/mavros/global_position/global` (MCS bridge or the simulator's shim).

All field data comes from a single site and is weather-limited: treat it as irreplaceable,
and state any control result derived from it as single-site (`project_synthesis.md` §4, §10).

### Known data caveats

- **Manual-target and pinger thrust columns are not calibrated Newtons.** `solve_LoS` builds
  `[v ± 0.295·yaw_rate/2]` (keep-location) / `[v ± 0.295·yaw_rate]` (pursuit, pinger) from a
  surge in m/s and a yaw rate in rad/s, published as force; the allocator *divides* by the
  moment arm where this law multiplies. Those columns are **relative** commands, comparable
  within one run only. Path following (PID, LoS, MPC) is unaffected. Fixing it rescales all
  steering authority in two branches and needs a dock test.
- **`fsin` runs at 0.5 m/s**, not the 0.1 m/s it was authored at (`_FSIN_V`, `e6dff70`): runs
  either side of that commit are not comparable (`TRAJECTORY_SYSTEM.md` §3).
- **The target columns changed meaning once**: logs before `_reference_pose` carry a PID/LoS
  target ~2.5 cm further along the path (the window's second pose).

---

## 7. Trajectories

`PathGeneration.single_pose(t, shape)` serves `station_keeping`, `circle`, `straight_line`,
`sin`, `fsin`, `square`, `kin_square`, `seabed_scanning` and `from_yaml:<abs path>`;
`SHAPES` / `is_valid_shape()` are the importable half of the contract. Library, revision
record and known defects (`square`'s 4 m discontinuity): `TRAJECTORY_SYSTEM.md`.

- **Pure in `t`** for every shape (`fsin` reads an append-only cumulative 0.01 s table), so
  trajectories can be swapped, replayed or hot-reloaded. Speed is baked into each formula.
- **Frame**: `/blueboat/odom`'s local ENU — a shape starting at `(0, 0)`, yaw 0 starts at the
  launch point heading East. GPS-anchored missions arrive pre-translated by the MCS deploy
  step; in a session a non-anchored mission arrives placed at the boat. No transform here.
- The hard-coded shapes are **reference conditions for existing field data**: changing one
  silently invalidates comparison with earlier runs.
- Shapes that end hold their last pose (`sin`, `kin_square` at t = 500, `seabed_scanning` at
  ≈ 77.7 s) and the station-keeping hold takes over; `straight_line`, `square`, `circle`
  never clamp. `fsin`'s `radius` (1.5 m) scales the weave, not its speed.
- An unknown `trajectory:=` is a FATAL at construction; `single_pose` raises `ValueError`.
- `from_yaml` (`blueboat_trajectory/1`: dense `[t, x, y, yaw]`, linear, short-way yaw,
  clamped or `loop: true`) rides inside `trajectory`; the `yaml_path` parameter wins when set
  but no launch file passes it. The file is **watched** (reload on mtime change) and a
  station-keeping pose at the origin is served **until it appears** — load-bearing for
  deferred GPS deployment and the session file.

---

## 8. `blueboat_description`

URDF/xacro, meshes, Gazebo world, spawn chain `world_launch.py` → `upload_rov_launch.py` →
`state_publisher_launch.py`; details in `blueboat_description/CLAUDE.md`. Gazebo plugins
still carry Fortress names (Harmonic's deprecated-name shim — superproject §4.6). In
simulation `/blueboat/odom` comes from `upload_rov_launch.py`'s bridge (20 Hz,
`odom_frame: world`). The **simulation** MPC model is fitted to `hydrodynamics.xacro`
(secant damping); the **real-boat** model shares only the rigid body with it. A `replay`
xacro argument (default false, default description byte-identical) builds the force-free
boat a bag replay moves by `set_pose`.

---

## 9. `log_reviewer/` — the mission log reviewer app

Standalone PySide6 app (`COLCON_IGNORE`); guidance in `log_reviewer/CLAUDE.md`. Two rules
that bind beyond it: it opens `~/ros2_ws/data/Robot_data/` **strictly read-only** (CM-7;
exports go to `Processed_Robot_data/`), and it takes every number and the plotting frame
from `poslog_report`, so the archived PNG and the app cannot disagree.
