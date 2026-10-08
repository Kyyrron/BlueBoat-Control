# CLAUDE.md — BlueBoat-Control

Working guidance for this submodule. Read §1 (Non-negotiables) before editing anything.

Two documents sit beneath this one and are the deeper reference for the control stack:
`blueboat_control/src/TRAJECTORY_SYSTEM.md` (where the reference target comes from — the
trajectory library, τ and the governor) and `FIELD_TUNING.md` (what each controller does with
that target in practice: every tuning knob, real/sim defaults, symptom→knob index).
`log_reviewer/README.md` covers the desktop log-reading app (§9). Nested `CLAUDE.md` files
load when working in their directory: `blueboat_control/src/CLAUDE.md` (controller and thrust-path
evidence behind §4–§5), `blueboat_description/CLAUDE.md` (§8), `log_reviewer/CLAUDE.md` (§9). The module carries no open
backlog; §6 records the data caveats that survive it.

---

## 0. What this module is

The **platform control stack** for a BlueRobotics BlueBoat USV: MAVROS/ArduPilot bridge,
thruster driver, trajectory generation, and three interchangeable controllers (MPC, PID,
LoS). It sits beneath a separate side-scan-sonar perception and survey-planning project,
which consumes this module's topics and runs unmodified against both simulation and hardware.

ROS 2 **Jazzy**, Python nodes throughout. `blueboat_description`'s Gazebo plugins still carry
Fortress names (loaded through Harmonic's deprecated-name shim — superproject §4.6).

| Package | Build type | Role |
|---|---|---|
| `blueboat_control` | ament_cmake (+ `ament_python_install_package`) | All nodes, controllers, trajectories, launch files |
| `blueboat_description` | ament_cmake | URDF/xacro, meshes, Gazebo world and spawn launch (§8) |
| `blueboat_interfaces` | ament_cmake (rosidl) | `srv/RequestPath.srv`, `msg/OmniscanProfile.msg`, `msg/ProcessedSSSPing.msg` |

A fourth top-level directory, **`log_reviewer/`**, is not a package at all — a standalone
PySide6 desktop app for reading recorded missions, carrying a `COLCON_IGNORE` so a workspace
build skips it. See §9.

All three `blueboat_interfaces` definitions are registered in one `rosidl_generate_interfaces`
call. `OmniscanProfile.pwr_results` is `uint16[]`; the two `.msg` files serve the sonar
project and no node in this module publishes or subscribes them.

---

## 1. NON-NEGOTIABLES

These rules must not be changed without explicitly checking their
downstream consequences.

**N1 — Never change the name or message type of any topic, service, or parameter.**
This module is a drop-in interface for the perception/planning stack, the Mission Control
Station, and the GCS visualiser, all separate codebases. Refactors change file internals
only; a node's external inputs and outputs stay byte-identical. Changing a signature is a
cross-repo decision, not a local one.

**N2 — Simulation and real water expose the identical ROS interface.**
Downstream stacks run unmodified in both; divergence invalidates the sim-to-real comparison
the thesis rests on.

**N3 — MAVROS twist is body-frame and must not be rotated.**
`/mavros/local_position/odom` has `child_frame_id: base_link` and MAVROS has already rotated
its `twist` into that frame, so it is surge/sway/yaw-rate while `pose` is world-frame; the ENU
velocity is a separate topic, `/mavros/local_position/velocity_local`. `robot_interface`
translates the **pose position** to the launch point (subtracting `x0, y0` only — axes stay
ENU, yaw stays **absolute ENU**; the published frame is local ENU, see the §2.2 odom row) and
passes **twist** through untouched (`robot_interface.py:492`). Two consumers depend on it staying
body-frame: `master_control` reads `current_twist[0]` as surge for the inner speed loop of both
`PID` and `LoS` (`master_control.py:401`), and the pinger dead-reckoning subtracts
`self.vel + ω × p` from body-frame pinger coordinates (`robot_interface.py:534`).

Measured against mavros 2.14.0, not inferred. Driven by a synthetic FCU holding 1 m/s due
**north**, so the two readings separate: at heading north the odom twist reads (1.000, 0.000),
at heading NE it reads (0.707, 0.707) — the body-frame values — while
`local_position/velocity_local` reads (0.000, 1.000) at both. End to end through
`robot_interface` at `yaw0 = 45°`, `/blueboat/odom` carries (0.707, 0.707). Rotating it by
−`yaw0` would turn a correct 0.707 into 1.000.

**N4 — `enable_motors` gates thruster output; a closed gate HOLDS NEUTRAL.**
The gate is in `manualMove` (`robot_interface.manualMove`, anchor on the symbol —
the line moves): **no thrust-bearing PWM reaches the motors unless
`enable_motors:=True`.** Every write to `/mavros/rc/override` outside the gate
carries neutral 1500/1500 or channel release, never thrust: `full_stop()` calls
`manualMove([0,0], force=True)`, `mode_callback()` sends neutral then release when
leaving override, and — since 2026-09-04 — the closed gate itself streams neutral.
Any *new* bypass that carries thrust is a rule violation.

**The closed gate no longer goes silent, and that is the point.** It used to just
`return`, which is not inert: `robot_interface` requests `override`
unconditionally at init, so by then `SERVO1/3_FUNCTION` are on RCIN1/RCIN3
passthrough and the ESCs follow RC channels 1 and 3 from *any* transmitter, with
nothing feeding `RC_OVERRIDE_TIME` to keep those channels ours. Motors could spin
with `enable_motors:=False`. Streaming neutral pins both channels and keeps the
override watchdog fed; 0 N is an exact knot of the calibration table (→ 1500 µs,
and `3000 − 1500 = 1500` for the reversed side), so it is true neutral, not a
rounded one. Only done while `self.mode == 'override'` — outside it the autopilot
is not listening to us and streaming would fight `mode_callback`'s release.
The superproject's CM-16 is worded to match: *only neutral PWM may be written
while disabled*.

**N4b — `'stop'` latches.** `full_stop()` sets `self.estopped`, cleared only by
the `'enable'` token (`release_estop`). While latched the timer publishes
`controller_ready=False` at the readiness rate and re-zeroes through the
**unforced** `manualMove([0,0])` every tick. Without the latch a stop is undone
50 ms later by the next `manualMove(self.thruster_input)`, because
`master_control` keeps publishing — and the station's E-STOP no longer works by
dropping out of override, so the latch is what makes it an actual stop.
`timer_callback` must never carry `force=True`; keep it that way.

**N5 — Restore the default servo mapping before shutdown.**
`override` remaps `SERVO1/3_FUNCTION` to RC passthrough; leaving the boat in that state
disables the Xbox controller. Never set a SERVO function to `0` (Disabled) — that produces an
ArduPilot PreArm "no motor" failure. `param_set` defines `SERVO_DISABLED = 0` but never
applies it.
Since 2026-09-04 **both nodes attempt this themselves** on the way out, in
independent `try/finally` blocks (previously neither had one: a bare
`rclpy.spin` meant `KeyboardInterrupt` skipped `destroy_node()` entirely, so the
boat was left in passthrough and the position CSV was never closed).
`param_set.restore_default_blocking` re-applies the mapping bounded to ~3 s;
`robot_interface.shutdown` stops the motors, releases the RC channels, requests
`default`, closes the CSV and writes the post-mission report. Both are
**best-effort** — a launch teardown SIGINTs the whole process group, so mavros is
usually dying at the same moment — and neither depends on the other. An operator
`default` before shutdown is still the reliable route.

**N6 — Thrust is streamed as RC override, never as per-tick acknowledged MAVLink commands.**
`OverrideRCIn` on `/mavros/rc/override` at ~20 Hz is latest-wins, hides packet loss, and
feeds ArduPilot's `RC_OVERRIDE_TIME` watchdog; acknowledged per-tick service calls to
`/mavros/cmd/command` stall the command plugin for seconds when a single ACK is lost.
`set_servo()` survives as a documented legacy fallback and is called from nowhere.

**N7 — Controller iteration happens offline against recorded rosbags.**
A control or AI change must not require a field session to test. Field time is weather-gated
and is the project's scarcest resource.

**N8 — The path reference advances from the boat's measured progress, never from wall-clock
time.** A time-driven reference is an open-loop player rather than a follower, and no gain
tuning can compensate for it. The governor at `master_control.py:584-643`
(`path_progress_errors` + `advance_governor`) is what enforces this.

`current_time = time.time() - self.initial_time` (`master_control.py:391`) still exists, but
it only timestamps `/monitoring_data` and the `.npy` log — it does not touch the reference.
Do not "fix" it by routing it back into path generation; that is precisely the design this
replaced.

**N9 — Monitoring output is world-frame in every controller branch.**
`/monitoring_data` and the `.npy` log carry world-frame `x_d, y_d, psi_d` for manual, pinger,
LoS, PID and MPC alike; mixed frames corrupt the station map display. Each branch sets
`world_target` and monitoring reads that. `/controller_target` deliberately keeps its original
body-frame content for the pinger case — the two are different signals and must not be
unified, which is also why the publish is not hoisted into the other branches. A consumer that
wants the target during path following or manual control reads `/monitoring_data[4:6]`, which
is world-frame in every branch.

It must also mean the **same quantity** in every branch: all three path-following branches set
`world_target` from `_reference_pose(self.controller_path)`, the first pose of the window.
Do not set it from `cf.compute_target`'s result — that returns the *second* pose, which is
right for the control law (the velocities come from differencing the two) and wrong for the
log, and using it made `x_d`/`y_d` the pose at `tau + path_time` on `PID`/`LoS` and the pose
at `tau` on `MPC`.

---

## 2. Interface — the authoritative contract

The control nodes write topic and service names **absolutely** (leading `/`) in the source.
`master_control` is constructed with `namespace='blueboat'`, but ROS 2 does not namespace
absolute names, so they resolve exactly as written. Rewriting them as relative names would
silently rename half the interface (N1).

Two components use **relative** names instead, resolved through their node's namespace, and
land on the same wire names: `path_publisher` (`set_path`, `path_request` — root namespace)
and the `ROV` helper in `blueboat_control/__init__.py` (`odom`, `joint_states`,
`robot_description`, `cmd_<thruster>` and `cmd_<joint>`, plus the display-only
`blueboat_<thruster>_wrench` and `blueboat_base` — all under `blueboat`, the namespace of every
node that constructs it).

### 2.1 Nodes

Every executable below is installed **flat** into `lib/blueboat_control` by `CMakeLists.txt`,
which is why the sources import each other as bare modules (`import custom_functions`,
`import PID`, `import ur_mpc`) regardless of the directory they live in.

| Executable | Source path under `blueboat_control/` | Node name | Purpose |
|---|---|---|---|
| `master_control.py` | `src/` | `master_control` (ns `blueboat`) | The controller: MPC / PID / LoS |
| `simulation_interface.py` | `src/` | `pid_sim` (ns `blueboat`) | Gazebo thrust bridge via `ROV`; sim-side readiness; the position CSV (no-pinger layout) |
| `robot_interface.py` | `src/robot_interaction/` | `blueboat_controller` | MAVROS bridge, thrust→PWM, odom republish, CSV logging |
| `param_set.py` | `src/robot_interaction/` | `blueboat_parameter_control` | SERVO function + GCS sysid remapping; watchdogged so it can never latch busy |
| `uwgps_log.py` | `src/robot_interaction/` | `underwater_gps_logger` | Water Linked UGPS HTTP poller |
| `path_generation.py` | `src/_custom_libraries/` | `path_generation` | `/path_request` service; trajectory library |
| `path_publisher.py` | `src/_custom_libraries/` | `path_publisher` | Whole-path preview for RViz; outside the control loop |
| `MPC/ur_mpc_control.py` | `src/MPC/` | `mpc_control` (ns `blueboat`) | Standalone MPC node — installed, launched by nothing |
| `MPC/uvr_mpc_control.py` | `src/MPC/` | `mpc_control` (ns `blueboat`) | Standalone 3-thruster MPC node — installed, launched by nothing |

`custom_functions.py` (shared helpers incl. `data_root`, `compute_target`) imports ROS;
`frame_math.py`, `robot_log_schema.py`, `thrust_limits.py`, `path_stamp.py` and
`poslog_report.py` (all `src/_custom_libraries/`) are **ROS-free by construction** — numpy
only, or no imports at all — so the geometry and the CSV format are debuggable from a plain
Python prompt with no sourced workspace. Keep them that way; anything needing `rclpy` belongs
in `custom_functions.py` instead.

`poslog_report.py` (ROS-free; also a CLI) files a finished run: it creates
`Robot_data/<csv stem>/` and **moves** the CSV, its `-origin.yaml` sidecar and a PNG report in.
Moving a field record is allowed, rewriting one is not (CM-7): an existing folder is never
touched, re-running is a no-op. Load-bearing details:
- **matplotlib is imported lazily, inside the renderer only** — `robot_interface` calls it
  from its shutdown hook, `package.xml` declares neither library, and a flight node must never
  fail because a plotting library is absent; the folder and the move happen with stdlib only.
- **It decides the frame a track is plotted in** (`is_simulation`, `track_series`,
  `origin_label`) for itself and the log reviewer: a simulated run is local ENU metres, never
  latitude/longitude.
- Speed is central-differenced from `relative_x/y`; samples above `MAX_PLAUSIBLE_SPEED_MS`
  (5 m/s) are pose discontinuities, **excluded from every statistic and counted**, never
  clipped.

The last two nodes claim the same node name and are started by neither launch file. They are
byte-identical to each other on the interface: both subscribe `/blueboat/odom`, publish
`/monitoring_data` and `/pose_arrow`, and hold a `/path_request` client, duplicating
`master_control`'s side of those four rows. Both also construct the `ROV` helper.

### 2.1.1 File organisation of the three long nodes

`master_control.py`, `robot_interface.py` and `path_generation.py` each open with a FILE MAP
comment and banner-numbered sections (wiring → knobs → main loop → maths → plumbing/logging).
Method *order* is free to change; method *contents* are not, and neither is what file a thing
lives in — see the constraint below and N1.

**The `fsin` table must stay at `path_generation`'s module scope.** `_fsin_extend` and
`_fsin_state` mutate the `_fsin_yaw/_x/_y` globals through the `global` keyword, which only
reaches names defined in this module's own namespace — moving the globals elsewhere breaks
that binding.

### 2.2 Internal topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface` (real-robot run) · Gazebo bridge (simulation, §8) | `master_control`, `simulation_interface` — frame is **local ENU** on both: origin = launch point (real) / Gazebo world origin (sim), axes East/North, yaw **absolute ENU** (0 = East, CCW+). Real yaw is NOT re-zeroed (fixed 2026-08-31; documents describing a launch-relative yaw predate the fix) |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | `robot_interface` | `master_control` |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface` · `simulation_interface` | `master_control` |
| `/thruster_input` | `std_msgs/Float32MultiArray` | `master_control` | `robot_interface`, `simulation_interface` |
| `/controller_target` | `std_msgs/Float32MultiArray` | `master_control` — **pinger branch only** | `robot_interface` (stored, never read) |
| `/monitoring_data` | `std_msgs/Float32MultiArray` | `master_control` (`simulation_interface` creates the publisher but never publishes — its monitoring block is commented out) | `robot_interface`, `simulation_interface` (both for the target columns of the position CSV) |
| `/blueboat/param_str` | `std_msgs/String` | `robot_interface` | `param_set` |
| `/blueboat/param_ready` | `std_msgs/Bool` | `param_set` | `robot_interface` |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set` | `robot_interface` |
| `/uw_gps_data` | `std_msgs/Float32MultiArray` | `uwgps_log` | `robot_interface` |

The behaviour of these topics that crosses modules — `controller_ready`'s two publishers with
different QoS (never make the subscriber latched), `/thruster_input` never silent (every early
return publishes `[0, 0]`), `/blueboat/pinger_coordinates` carrying **three** body-frame values
(`fixed_pinger` is hard-coded `False`), and `/controller_target` published **only in the pinger
branch**, by decision (world-frame targets ride `/monitoring_data[4:6]`, N9) — is stated in the
superproject `.claude/CLAUDE.md` §4.1, which loads in every session here.

### 2.3 External-facing topics

| Topic | Type | Direction | This side | Other party |
|---|---|---|---|---|
| `/blueboat/input_str` | `std_msgs/String` | in | `robot_interface` | Operator CLI, Mission Control Station |
| `/blueboat/manual_target` | `std_msgs/Float32MultiArray` | in | `master_control` | GCS visualisation app (`[x, y]`, world frame) |
| `/monitoring_data` | `std_msgs/Float32MultiArray` | out | `master_control` | Mission Control Station map display |
| `/pose_arrow` | `visualization_msgs/Marker` | out | `master_control` | RViz / Gazebo debug (simulation only) |
| `/set_path` | `nav_msgs/Path` | out | `path_publisher` | RViz / GCS |

### 2.4 MAVROS boundary

Split across two nodes.

**`robot_interface`** subscribes `/mavros/state` (`State`, default reliable QoS),
`/mavros/imu/data` (`Imu`), `/mavros/local_position/odom` (`Odometry`) and
`/mavros/global_position/global` (`NavSatFix`) — those three on **BEST_EFFORT, depth 10**.
It publishes `/mavros/rc/override` (`OverrideRCIn`) and holds clients for
`/mavros/cmd/arming` (`CommandBool`), `/mavros/set_mode` (`SetMode`) and
`/mavros/cmd/command` (`CommandLong`).

**`param_set`** owns the parameter services: `/mavros/param/pull` (`mavros_msgs/ParamPull`),
plus `/mavros/param/get_parameters` and `/mavros/param/set_parameters` (`rcl_interfaces`
`GetParameters` / `SetParameters` against the mavros node's own ROS parameters). It
deliberately does **not** block on service availability in its constructor; it checks lazily
and lets `robot_interface` retry.

FCU endpoint: `BlueBoat_launch.py`'s `fcu_url` (default `udp://:14550@192.168.2.2:14550`;
**port 14550 collides with a running QGroundControl** — superproject §4.4).

### 2.5 Service

`/path_request` — `blueboat_interfaces/srv/RequestPath`.
Request: `std_msgs/Float32MultiArray path_request`, an array of **path-parameter values**.
Response: `nav_msgs/Path path`, one pose per requested value, `frame_id: "world"`.
Server: `path_generation`. Clients: `master_control`, `path_publisher`, `ur_mpc_control`,
`uvr_mpc_control`.

This contract is deliberately parameter-agnostic — the caller decides what the numbers mean.
That property is what allows the reference-generation strategy to change without touching
`path_generation`, and it must be preserved.

**Exactly one server may run, and it is enforced.** ROS 2 lets two nodes offer one service
name; every request is then answered by whichever replies first, and the bare response cannot
say which. Measured: with two missions up, `master_control`'s reference alternated tick by
tick between two different trajectories and the boat could follow neither. Two guards, either
sufficient alone:

* `path_generation` **refuses to be the second server**: before creating the service it
  enumerates the graph and exits non-zero with a FATAL naming the other node
  (`allow_duplicate_server:=true` opts out; `server_discovery_wait` 2.0 s). The enumeration
  deliberately does **not** exclude itself by name — the duplicate is another node called
  `path_generation` in the same namespace.
* Each returned pose is **stamped with the path parameter it was evaluated at**
  (`path_stamp.encode`), and `master_control.accept_path` rejects any response whose pose count
  or `poses[0]` stamp does not match its request. `RequestPath` is untouched (N1/CM-1). A
  server built before this stamps the wall clock: detected once, reported as "rebuild", then a
  geometric plausibility check for the rest of the run.

`master_control` also logs an error at startup if it sees more than one server.

### 2.6 Operator CLI

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: <value>"
```

`enable` · `stop` · `override` · `default` · `arm` · `disarm` ·
`move <left> <right> <seconds>`

Any **unrecognised** first token falls through to `move_callback`
(`robot_interface.py:418`), which is handed the *whole* split string and still requires exactly
four fields — so `x 1.0 1.0 5` is accepted as a move without the `move` keyword, while
`1.0 1.0 5` (three fields) is not. Anything that is not exactly four fields is rejected with a
log line and no action.

---

## 3. Build, launch, run

```bash
# Build — from the workspace root (parent of src/)
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch blueboat_control Sim_launch.py controller_type:='MPC' trajectory:='kin_square'
ros2 launch blueboat_control BlueBoat_launch.py enable_motors:=True controller_type:='PID' note:='testing_gains'
```

Launch arguments and defaults: the launch files. What they do not make obvious:

- **`BlueBoat_launch.py`** always starts `mavros`, `robot_interface`, `uwgps_log`,
  `param_set`; `master_control` only when `controller_type` is non-empty, `path_generation`
  only when `use_pinger` is **False**. `use_pinger` reaches `robot_interface` as the parameter
  **`use_UWgps`**, which also selects the CSV layout (§6). `fcu_url` see §2.4.
- **`Sim_launch.py`** defaults `controller_type` to **`'MPC'`** and always launches a
  controller; it never starts `robot_interface`. `spawn_yaw` is **radians ENU** (spawn
  position stays (0, 0)); the Mission Control Station passes a random one for GPS-anchored
  simulated missions. It is an `OpaqueFunction` (post-v1.0, 2026-09-28): a
  `from_yaml:<file>` trajectory sizes `path_publisher`'s `total_time` as duration × 1.1 + 30 s
  (`_path_window`), reading the duration from the designer file for a `.deployed/` path
  (the deployed copy appears only after MCS's GPS fit converges).

**Testing.** No lint, type-check or ROS-side test target. Two gates:

```bash
python3 .claude/tools/interface_inventory.py --check .claude/tools/interface_baseline.json  # 0 = unchanged, 2 = moved
QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py                                # 121 checks, ~25 s
```

The interface guard (stdlib only, ~0.15 s, also a `PostToolUse` hook) statically extracts
every publisher, subscriber, service, client and declared parameter plus the
`blueboat_interfaces` field lists and compares them to the baseline; only a name, type or QoS
change produces a diff. It is **not committed** (`.gitignore` excludes `.claude/tools/` and
`.claude/settings.json`), so a fresh clone has no contract gate. A *deliberate* contract change
is a cross-repo decision (N1): notify the consumers, then re-baseline with `--update`. The log
reviewer gate is in a fresh clone but needs a real poslog under `~/ros2_ws/data/Robot_data/`
(skips, exit 0, without one).

**Interpreter — name it, never trust a bare `python3`.**
- `~/ros2_ws/.venv` carries `acados_template`, `casadi`, `pandas` and is what the **ROS nodes**
  need: `master_control` and `simulation_interface` die with `ModuleNotFoundError` without it
  (the executables' `#!/usr/bin/env python3` means activating the venv selects it).
- `/usr/bin/python3` has numpy, scipy, matplotlib, sympy, PyYAML, `rclpy` — **not** `casadi`,
  `acados_template` or `pandas`.
- On this machine an interactive shell may resolve bare `python3` to
  `SSS-Dataset-Aug-Studio/.venv/bin/python3` (no matplotlib, no pandas).

**Dependencies** (`requirements.txt`; apt: `xacro`, `simple_launch`, `mavros`,
`urdf_parser_py`, acados). `acados_template` and `casadi` are needed to **start
`master_control` at all**, not just for MPC: `import ur_mpc` sits at module scope. **acados
needs two things pip does not install** (`README.md`): `libacados.so` and the Tera renderer at
`<acados>/bin/t_renderer` — without the renderer, code generation stops on an invisible
`input()` prompt under `ros2 launch` (the 2026-08-31 "MPC never starts" hang; `master_control`
now preflights it and exits with a FATAL). Export `ACADOS_SOURCE_DIR`: unset, acados guesses
and can pick a different checkout (this machine has two).

---

## 4. Control architecture

Three controllers share one control callback. Branch priority: **manual target** → **path
following** → **pinger** → nothing. `MPC` is unsupported in pinger mode. Evidence and
derivations for everything below: `blueboat_control/src/CLAUDE.md`; every tuning knob with its
real/sim default and the symptom→knob table: `FIELD_TUNING.md`; reference generation:
`blueboat_control/src/TRAJECTORY_SYSTEM.md`.

- **The controller object is built in `__init__`, not on the first tick** — MPC's acados
  build in `timer_callback` froze the single-threaded executor. `Controller node initiated`
  marks construction; `Controller ready` marks the readiness handshake.
- **The acados solver is cached in `$ROS_HOME/blueboat_control/mpc`**
  (`ur_mpc.acados_build_dir()`), deliberately **not** under `cf.data_root()` (write-once
  field record). Any change to the MPC model coefficients, horizon, weights, `thrust_limit` or
  `mpc_qp_iter_max` regenerates it (~1 min, logs `generated and compiled`) — expected; do not
  delete the cache to "make it take".
- **A failed acados solve is not a command (C6).** A non-zero status → solver reset, cold
  re-seed, **one retry**, else **zeros**, logged through the ROS logger (never `print()`),
  without an early return so `/thruster_input` and the `.npy` keep flowing.
  `mpc_qp_iter_max` 0 derives the qpOASES working-set budget as `max(50, 4·nu·N)` — acados'
  default 50 sits below the simulation horizon's 60 variables.
- **Reference generation** advances `tau` from the boat's measured progress (N8): the
  governor's factors are clipped to `[0, 1]` and multiplied, so `tau` is monotonic and never
  faster than authored speed — nothing may scale `tau_dot` above unity. `fac_cross` is **off by
  default** (`gov_Emax = 0`): throttling on an error the inner loops cannot close is positive
  feedback. The request is `linspace(tau, tau + path_time, path_steps)`, issued asynchronously
  (window one or two ticks stale, loop never blocks); `path_time` / `path_steps` are
  **derived** (PID/LoS 0.05 s / 2, MPC 2.5 s / 15), never declared, so window and horizon
  cannot disagree.
- **Which law runs where:** manual target → `solve_LoS` for every `controller_type`, then
  `manual_keep_location` once reached; path following → MPC `ur_mpc.MPCController.solve`, PID
  `PIDLoS.compute` (with `u_ff`, `psi_path`), LoS `los_guidance`; pinger → PID
  `PIDLoS.compute(state, target)` body-frame with `psi_path=None`, LoS `solve_LoS`.
  `solve_LoS` is a crude proportional pursuit law, not the path LoS, and works as-is.
  `PIDLoS` is always built with `lookahead = pid_lookahead = 2.5 m`.
- **Station-keeping hold** (zero authored speed): gate `w = 1 - U_d/hold_speed` plus
  `hold_radius`. `w` is exactly zero for every library trajectory (slowest: `sin` at
  0.280 m/s), so raising `hold_speed` above 0.28 would start altering path following.
- **Tuning knobs** are all `declare_parameter`'d unconditionally in
  `_declare_tuning_parameters` (`ros2 param list /blueboat/master_control` shows the set),
  read once at construction; gain triples are double arrays (no tuple type in ROS 2). Several
  defaults are **split simulation/real** (PID gains, MPC horizon/time/R and its plant model,
  point gains, `manual_hold_kx` — in N/m, not m/s). Not in `FIELD_TUNING.md`:
  `path_request_timeout` / `path_stale_timeout` (1.0 s each — without the first, one lost
  response wedged the node), `thruster_input_timeout` (0.5 s, both interface nodes),
  `param_sequence_timeout_s` (20 s, `param_set`).

---

## 5. Thrust path — sharp edges

Evidence (tables, measurements, proofs) for each item: `blueboat_control/src/CLAUDE.md` §5.

- `/thruster_input` carries **`[right, left]`** — consistent across every code path (allocation
  matrix, URDF thruster placement, `ROV.read_model`'s alphabetical joint order,
  `simulation_interface`, `solve_LoS`, `manualMove`, the CLI's `move <left> <right> <s>`
  stored as `[right, left]`); the full trace is in `blueboat_control/src/CLAUDE.md` §5.
- `left_pwm = 3000 - pwm`. The left thruster is reversed to compensate an asymmetric
  propeller; this is intentional, not a typo.
- `manualMove` contains a `compensation_gain` conditional (1.2 / 0.75) that is immediately
  overridden by a hard-coded **`1.0`**, leaving the branch dead. The conditional also keys on
  `input[1]` (left) while the gain is applied to `input[0]` (right). Do not tidy this without
  deciding what it should do.
- Thrust→PWM is a `PchipInterpolator` fitted to a measured bollard-pull table
  (`custom_functions.generate_interpolator`), so its useful range is asymmetric: about
  −27.6 N to +55.2 N.
- **Saturation is uniform everywhere, and it is one number.** PWM clamps to `[1100, 1900]`;
  thrust is bounded at every exit (`ThrustAllocator.allocate`, `master_control.publish_thrust`,
  `robot_interface.manualMove`, `simulation_interface`) by **scaling the pair by one factor**
  to `thrust_limit` (20.0 N everywhere; `thrust_limits.scale_to_limit`). Never per-side clip:
  it rewrites the wrench (a commanded 27 N yaw collapsed to 2 N — the harder the turn, the
  straighter the boat went). `publish_thrust` returns the saturated vector, so monitoring,
  `.npy` and CSV agree with the wire.
- **The 0–2 N band does not turn the propellers** (inside the T200 ESC's neutral deadband;
  the bollard-pull table has none). Only `solve_LoS`'s real-boat pinger surge stays under 2 N
  beyond 2 m, so only it is floored: `min_thrust * g * max(0, cos(bearing))`. Both factors
  only reduce the floor and are load-bearing (no push away from an abeam target; no step at
  `hold_radius`). The yaw differential is bit-identical everywhere; `min_thrust = 0.0`
  restores the original law exactly. The boat parks at ~0.81 m, by design.
- **A reached manual target is held, not abandoned.** `manual_keep_location` re-evaluates
  every tick (on station inside `manual_hold_radius`, pursuit again beyond
  `manual_reacquire_radius`, proportional capped floored surge between); **the yaw channel is
  untouched**; gains meet the pursuit law at the handover. `manual_hold_radius <= 0` restores
  the old latch bit-identically. The floor is on common-mode surge, never per side.
- **Loss of reference zeroes the thrust at both ends.** `master_control` publishes an explicit
  `[0, 0]` on each of its three early returns instead of falling silent, and both interface
  nodes stop applying a command that has gone stale: `thruster_input_timeout` (0.5 s in each,
  a declared parameter) against the producer's 20 Hz tick, so ten missed ticks — well outside
  DDS jitter and well inside ArduPilot's own `RC_OVERRIDE_TIME`. The watchdog covers what a
  publish cannot: a crashed or hung controller. It **zeroes thrust and does not disarm**, and
  releases itself as soon as commands resume; `full_stop()` (which does disarm) stays bound to
  the operator `stop` command, because these stalls are transient by design. On the real boat
  the zeroing goes through `manualMove([0, 0])` **without** `force`, so it is behind the
  `enable_motors` gate and is not a third `/mavros/rc/override` write path (N4). The
  `controller_type == ''` manual-move timeout is unchanged and still owns that case.
- `param_set`: `override` maps `SERVO1/3_FUNCTION` to RC passthrough (51/53) and sets
  `SYSID_MYGCS` / `MAV_GCS_SYSID` to the MAVROS sysid (1); `default` restores 74/73 and
  sysid 255. Which of the two sysid parameter names exists is resolved once at runtime by
  querying both. Every write is read back and verified before `param_ready` goes true.
- Readiness handshakes survive DDS discovery races. `robot_interface` re-publishes
  `/blueboat/controller_ready` on a 1 s timer, and `robot_interface` **re-requests** the
  mode every second until `param_set` confirms it, which `param_set` handles
  idempotently. Editing either side means keeping that pairing intact.
- **`param_set` never blocks forever, and always reports.** Three rules, none may be removed:
  (1) no state is cleared only by a callback — a 0.5 s watchdog abandons a sequence held past
  `param_sequence_timeout_s`; (2) every abandoned sequence is fenced off by a generation
  counter (`_seq`, checked first in each done-callback); (3) `param_mode` is always published
  (`''` = alive, no mode locked) and heartbeated at 1 Hz. Failures retry within
  `param_retry_limit` / `param_retry_delay_s`. **Downstream consequence, do not break it:** a
  repeated `param_mode` is *not* evidence a new command was acted on — the Mission Control
  Station's safe-shutdown requires a **transition** into `default`
  (`BlueBoat-MCS/.claude/CLAUDE.md` N1); `robot_interface.mode_callback` and `param_callback`
  are edge-triggered for the same reason.

---

## 6. Data

| Artifact | Path | Nature |
|---|---|---|
| Position/pinger CSV | `<root>/data/Robot_data/{date}-{note}-poslog.csv` | **Raw field record — never overwrite or regenerate.** Written by `robot_interface` in a real-robot run and by `simulation_interface` in Gazebo |
| Controller monitoring | `<root>/data/{ctrl}_data/{date}-{ctrl}_{sim}_data.npy` | Per-run result |

`<root>` = `custom_functions.data_root`, first match wins: `data_dir` parameter →
`$BLUEBOAT_DATA_DIR` → the sourced workspace (parent of the first `$COLCON_PREFIX_PATH` entry,
normally `~/ros2_ws/data/`) → the working directory — so a run started by the Mission Control
Station never writes into the station's repository. An unwritable root fails the launch naming
the path (`ensure_data_dir`, no silent fallback); names are claimed with `O_EXCL`
(`reserve_run_file`), so same-second runs get `-2`, `-3`, … (CM-7). Read back only by
`poslog_report.py` and the log reviewer.

`.npy` schema: `['t','x','y','psi','x_d','y_d','psi_d','u1','u2']`, target columns world-frame
per N9. The header is appended as a row of **strings** to the same list as the float rows, so
`np.save` coerces the whole array to strings — analysis scripts must cast back on load.

**Simulation writes the same file** (`simulation_interface`, always the no-pinger layout),
so a Gazebo run and a field run share one reader (N2 / CM-2). Differences: the date columns
carry **sim time** (a simulated log reads 1970-01-01), `lin_acc_*` is differentiated odometry
(no gravity), `actuation_state` only takes 1 and 3, and GPS stays (0, 0) unless something
publishes `/mavros/global_position/global` (MCS bridge or the simulator's shim).

Two CSV layouts, chosen by `use_UWgps` — **no pinger, 27 columns** and **pinger, 39 columns** —
sharing the same first 19 columns; the column lists and the 2026-08-31 schema revision are in
`robot_log_schema.py`'s docstring. In the pinger layout `ant_*` is all-zero (no launch file
passes `uwgps_log --antenna`) and `dep` duplicates `aco_z`. Rows are filled **by column name**,
so order can change without desynchronising the data.

**Legend rows (since 2026-10-08).** A new CSV opens with a **description** row (1–4 words)
and a **unit** row above the column names, data from row 4 — both from
`robot_log_schema.LEGEND`, written by both nodes **at file creation** with the header, never
added at run end (CM-7; a killed run keeps them). Logs recorded before have no legend and are
**never rewritten** to gain one. Every reader accepts both: `robot_log_schema.split_header`
finds the column names as the first row holding `relative_x` (row 1 old, row 3 new) —
`poslog_report.read_poslog`, and through it the log reviewer, use it; pandas reads a new log
with `header=2`. Log-reviewer exports always carry the legend (built from the schema for an
old source). No description or unit may contain a comma.

`actuation_state` says whether the logged command could reach the water: `0` motors disabled,
`1` live (enabled and in override), `2` enabled but not in override, `3` watchdog forcing
zero. Without it a run with the motor gate off is byte-indistinguishable from a live one, and
a watchdog trip is indistinguishable from a genuine zero command — the watchdog writes
`[0, 0]` into the same field.

A `<stem>-origin.yaml` sidecar is written beside the CSV carrying `latitude`, `longitude` and
`yaw0_rad`. Every world-frame column in the log lives in the local-ENU frame latched at the
first odom callback (origin previously recorded nowhere, which made a finished run impossible
to georeference afterwards). `yaw0_rad` is the boat's ENU heading at that instant —
**provenance only, not part of the frame**: since the 2026-08-31 local-ENU fix,
`relative_psi` is absolute ENU yaw; in logs recorded before it, `relative_psi` was
`yaw − yaw0_rad` (and world positions were ENU regardless), so the sidecar is what
disambiguates old data from new. Older logs also differ in layout (pre-2026-08-31: pandas
index column, `quat_*` instead of `roll, pitch`, milliseconds in `MicroSecond`) —
`robot_log_schema.py`'s docstring is the revision record.

Field data is campaign-bound and weather-limited — treat it as irreplaceable.

All of it comes from a single site, which bounds what any control result derived from it
supports: state such a result as single-site, not general. `project_synthesis.md` §10 and §4.1
govern how results from this project are phrased; §4's evidence layers cover the sonar and
policy work and do not extend to control-loop performance, so these artifacts are the only
evidence behind a control claim.

### Known data caveats

Three properties of the recorded artifacts that a reader — or the log reviewer — must know
about before drawing a number off them. None is a bug in the logging: the logs faithfully
record what happened.

**The manual-target and pinger thrust columns are not calibrated Newtons.** `solve_LoS`
builds its command as `[v + 0.295·yaw_rate, v − 0.295·yaw_rate]` from a surge in **m/s** and a
yaw rate in **rad/s**, and publishes that on `/thruster_input`, where every consumer reads it
as force — including `robot_interface`'s bollard-pull interpolator, the CSV's
`right_thr_in`/`left_thr_in` and the reviewer's thrust panel. It compounds: the allocator
*divides* by the moment arm (`1/(2r) = 1.695`) where this law *multiplies* by it (0.295), a
factor of 5.75 between the two for the same nominal moment. So for a manual-target or pinger
run those columns are a **relative** command, comparable within one run and not across laws,
and the law's gains are meaningful only relative to each other. Path following — `PID`, `LoS`
and `MPC` alike — goes through the allocator and is unaffected. Correcting it rescales all
steering authority in those two branches at once and needs a dock test, so it is recorded
here rather than fixed.

**`fsin` runs at 0.5 m/s, not the 0.1 m/s it was authored at.** `_FSIN_V` was raised in
`e6dff70`. The geometry is unchanged — the same 1.5 m turn radius — but it is traversed 5×
faster: one cycle per 60 s instead of 300, yaw-rate amplitude 0.333 rad/s instead of 0.067.
`fsin` runs recorded either side of that commit are **not comparable**; every other shape is
untouched. `TRAJECTORY_SYSTEM.md` §3 carries the revision record.

**The target columns changed meaning once.** All branches now log `poses[0]` of the reference
window through `_reference_pose` (N9); logs recorded before carry a PID/LoS target ~2.5 cm
further along the path (the second pose, `tau + path_time`).


---

## 7. Trajectories

`PathGeneration.single_pose(t, shape)` — a **method**, because the `from_yaml` branch reads the
node's loaded trajectory — provides `station_keeping`, `circle`, `straight_line`, `sin`,
`fsin`, `square`, `kin_square`, `seabed_scanning` and `from_yaml:<abs path>`; the module-level
`SHAPES` / `is_valid_shape()` are the importable half of the contract. Library, revision record
and known defects (`square`'s 4 m discontinuity): `TRAJECTORY_SYSTEM.md`.

- It is **pure in `t`** for every shape, which is what lets a trajectory be swapped, replayed
  or hot-reloaded with no coupling to the controller. `fsin` has no closed form: it is read
  from an append-only cumulative Euler table (0.01 s) at module scope, each extension
  continuing from its stored last value, so a given `t` gives the same pose in any request
  order. **Speed is baked into each formula** (`x = 0.5*t` = 0.5 m/s).
- **Frame:** every shape and YAML row is in `/blueboat/odom`'s local ENU — a shape starting at
  `(0, 0)`, `yaw = 0` starts at the launch point heading **East**, real and sim alike.
  GPS-anchored missions arrive pre-translated by the MCS deploy step; `path_generation`
  applies no transform.
- The hard-coded shapes are the reference conditions for existing field data: changing one
  silently invalidates comparison with earlier runs.
- Shapes that end hold their last pose (`sin`, `kin_square` at t = 500, `seabed_scanning` at
  ≈ 77.7 s), so the station-keeping hold takes over; `straight_line`, `square`, `circle` never
  clamp. `fsin`'s `radius` knob (1.5 m since 2026-09-01) scales the weave, not its speed.
- An unrecognised `trajectory:=` name is a FATAL at construction (and `single_pose` raises
  `ValueError`), so the `if`/`elif` chain has no fall-through.
- `from_yaml` (`blueboat_trajectory/1`: dense `[t, x, y, yaw]`, linear interpolation with
  short-way yaw wrap, clamped or `loop: true`) rides inside the `trajectory` argument — the
  dedicated `yaml_path` parameter wins when set but **no launch file passes it**. It is
  **file-watched** (`_maybe_reload_yaml` on every request, reload on mtime change) and serves
  a station-keeping pose at the origin **until the file appears** — load-bearing: it lets the
  Mission Control Station deploy a GPS-anchored mission only once the odom↔GPS fit exists.

---

## 8. `blueboat_description`

URDF/xacro, meshes, Gazebo world and the spawn chain `world_launch.py` →
`upload_rov_launch.py` → `state_publisher_launch.py`; details in
`blueboat_description/CLAUDE.md`. Two facts that matter outside it: **the Gazebo plant and the
MPC's internal model share the rigid body and nothing else** (`hydrodynamics.xacro` carries
the BlueROV2 table), so an MPC result in simulation is not a solver-against-its-own-model
result; and in simulation `/blueboat/odom` comes from `upload_rov_launch.py`'s Gazebo bridge
(20 Hz, `odom_frame: world`), in a real-robot run from `robot_interface` — same topic, same
type, different origin.

---

## 9. `log_reviewer/` — the mission log reviewer app

A standalone PySide6 desktop app (not a ROS package; `COLCON_IGNORE`) for reading recorded
missions; full guidance in `log_reviewer/CLAUDE.md`. Two rules that bind beyond it: it opens
`~/ros2_ws/data/Robot_data/` **strictly read-only** (CM-7 — exports go to
`Processed_Robot_data/`), and it takes every number and the plotting frame from
`poslog_report` rather than re-deriving them, so the archived PNG and the app can never
disagree.
