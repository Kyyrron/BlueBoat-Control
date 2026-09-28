# CLAUDE.md — BlueBoat-Control

Working guidance for this submodule. Read §1 (Non-negotiables) before editing anything.

Two documents sit beneath this one and are the deeper reference for the control stack:
`blueboat_control/src/TRAJECTORY_SYSTEM.md` (where the reference target comes from — the
trajectory library, τ and the governor) and `FIELD_TUNING.md` (what each controller does with
that target in practice: every tuning knob, real/sim defaults, symptom→knob index).
`log_reviewer/README.md` covers the desktop log-reading app (§9). The module carries no open
backlog; §6 records the data caveats that survive it.

---

## 0. What this module is

The **platform control stack** for a BlueRobotics BlueBoat USV: MAVROS/ArduPilot bridge,
thruster driver, trajectory generation, and three interchangeable controllers (MPC, PID,
LoS). It sits beneath a separate side-scan-sonar perception and survey-planning project,
which consumes this module's topics and runs unmodified against both simulation and hardware.

ROS 2 **Jazzy**, Python nodes throughout. Every Gazebo plugin in `blueboat_description` is
declared with **Ignition Fortress** names (`ignition-gazebo-*-system`,
`ignition::gazebo::systems::*`) rather than Harmonic's `gz-sim-*` / `gz::sim::*`.

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

Non-node library modules, installed the same flat way and imported as bare modules:

| Module | Source path under `blueboat_control/` | Imports ROS? | Contents |
|---|---|---|---|
| `custom_functions.py` | `src/_custom_libraries/` | yes (`rclpy`, msgs) | Shared helpers: `data_root`, `odometry`, `compute_target`, quaternion/frame maths |
| `yaml_trajectory.py` | `src/_custom_libraries/` | no | `blueboat_trajectory/1` loader, evaluated at time `t` |
| `frame_math.py` | `src/_custom_libraries/` | **no** | `inRobotFrame()` — world→body geometry, with its full input/output contract |
| `path_stamp.py` | `src/_custom_libraries/` | **no** | The `/path_request` response tag: encodes a path parameter into a `builtin_interfaces/Time`, and the `matches` / `is_parameter` predicates both ends share |
| `robot_log_schema.py` | `src/_custom_libraries/` | **no** | The position-CSV column layouts (`COLUMNS_PINGER`, `COLUMNS_NO_PINGER`, `columns_for`, `target_columns_for`, the `ACT_*` constants) |
| `thrust_limits.py` | `src/_custom_libraries/` | **no** | `scale_to_limit()` — uniform per-thruster saturation, and the argument for why it is not two clips |
| `poslog_report.py` | `src/_custom_libraries/` | **no** | Post-mission report: reads either poslog layout (detected from the header), renders one PNG, then files the run into `Robot_data/<csv stem>/`. Also the CLI. |
| `PID/PID.py`, `MPC/ur_mpc.py`, `MPC/uvr_mpc.py` | `src/PID/`, `src/MPC/` | no | Controller implementations |

`frame_math.py`, `robot_log_schema.py`, `thrust_limits.py`, `path_stamp.py` and
`poslog_report.py` are **ROS-free by construction**
— numpy only, or no imports at all. That is deliberate: both can be imported, diffed and checked from a plain
Python prompt with no sourced workspace, which is what makes the geometry and the CSV format
debuggable without a running graph. Keep them that way; anything needing `rclpy` belongs in
`custom_functions.py` instead.

`poslog_report.py` needs numpy, and imports **matplotlib lazily, inside the
renderer only**. That is load-bearing, not fastidiousness: `robot_interface` calls
it from its shutdown hook, `package.xml` declares neither library, and a flight
node must never fail — at start or at teardown — because a plotting library is
absent. `finalise_run` therefore always creates the folder and moves the files
(stdlib only) and treats the picture as optional; the CLI can render it later
from a machine that has matplotlib.

**What it does with the files.** It creates `Robot_data/<csv stem>/` and **moves**
the CSV, its `-origin.yaml` sidecar and the new PNG into it. Moving a primary
field record is allowed; rewriting or regenerating one is not (CM-7 / N7), so an
existing destination folder is never touched and re-running is a no-op that says
so. The sidecar name is derived by pattern (`-poslog(-N)?.csv` → `-origin(-N).yaml`)
rather than by a fixed-length slice, which is what the writer does too — the old
slice mangled the name whenever a same-second collision produced
`...-poslog-2.csv`.

**It also decides the FRAME the track is plotted in**, for itself and for the log reviewer
(§9.6): `is_simulation`, `track_series` and `origin_label` live here, and the app imports them
rather than re-deciding, so the archived PNG and the app can never draw one run in two frames.
A simulated run is plotted in local ENU metres and prints no latitude or longitude anywhere.

**A reader note.** There is no speed column: speed is central-differenced from
`relative_x/y`, so samples implying more than `MAX_PLAUSIBLE_SPEED_MS` (5 m/s, well
above a BlueBoat's ~2 m/s) are pose discontinuities rather than motion. They are
**excluded from every statistic and counted in the summary**, never clipped: a run
whose pose teleports is a finding, and one 300 m/s spike otherwise sets the mean
and the y-scale for the whole mission.

The last two nodes claim the same node name and are started by neither launch file. They are
byte-identical to each other on the interface: both subscribe `/blueboat/odom`, publish
`/monitoring_data` and `/pose_arrow`, and hold a `/path_request` client, duplicating
`master_control`'s side of those four rows. Both also construct the `ROV` helper.

### 2.1.1 File organisation of the three long nodes

`master_control.py`, `robot_interface.py` and `path_generation.py` each open with a FILE MAP
comment and are divided by banner comments into numbered sections, ordered for a live field
session rather than by history: wiring first, then the knobs, then the main loop, then the
maths, then plumbing and logging last.

| File | Section order |
|---|---|
| `master_control.py` | 1 wiring · 2 **tuning knobs** (`_declare_tuning_parameters`, every gain) · 3 control loop · 4 guidance · 5 callbacks/helpers |
| `robot_interface.py` | 1 wiring · 2 main loop + watchdog · 3 thrust→PWM calibration · 4 operator commands · 5 pose/pinger · 6 telemetry · 7 MAVROS plumbing · 8 CSV logging |
| `path_generation.py` | module scope (`SHAPES`, `is_valid_shape`, the `fsin` table) · 1 wiring · 2 service entry point · 3 `from_yaml` reload · 4 shape library |

Method *order* is free to change; method *contents* are not, and neither is what file a thing
lives in — see the constraint below and N1.

**The `fsin` table must stay at `path_generation`'s module scope.** `_fsin_extend` and
`_fsin_state` mutate the `_fsin_yaw/_x/_y` globals through the `global` keyword, which only
reaches names defined in this module's own namespace — moving the globals elsewhere breaks
that binding.

### 2.2 Internal topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface` (real-robot run) · Gazebo bridge (simulation, §8) | `master_control`, `simulation_interface` — frame is **local ENU** on both: origin = launch point (real) / Gazebo world origin (sim), axes East/North, yaw **absolute ENU** (0 = East, CCW+). Real yaw is NOT re-zeroed (fixed 2026-08-31: subtracting `yaw0` without rotating the position axes made a hybrid frame that was only consistent when the boat launched facing East — the cause of the East-only trajectory-following field symptom) |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | `robot_interface` | `master_control` |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface` · `simulation_interface` | `master_control` |
| `/thruster_input` | `std_msgs/Float32MultiArray` | `master_control` | `robot_interface`, `simulation_interface` |
| `/controller_target` | `std_msgs/Float32MultiArray` | `master_control` — **pinger branch only** | `robot_interface` (stored, never read) |
| `/monitoring_data` | `std_msgs/Float32MultiArray` | `master_control` (`simulation_interface` creates the publisher but never publishes — its monitoring block is commented out) | `robot_interface`, `simulation_interface` (both for the target columns of the position CSV) |
| `/blueboat/param_str` | `std_msgs/String` | `robot_interface` | `param_set` |
| `/blueboat/param_ready` | `std_msgs/Bool` | `param_set` | `robot_interface` |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set` | `robot_interface` |
| `/uw_gps_data` | `std_msgs/Float32MultiArray` | `uwgps_log` | `robot_interface` |

**`/blueboat/controller_ready` has two publishers with different QoS.** `robot_interface`
uses depth 10 volatile and re-publishes every second; `simulation_interface` uses depth 1
**TRANSIENT_LOCAL** (latched) and publishes once. `master_control` subscribes with depth 10
volatile, which is compatible with both — a transient-local publisher satisfies a volatile
subscriber, not the reverse. Making the subscriber latched would break the real-boat path.

**`/thruster_input` is never silent while `master_control` runs.** Every early return in
`timer_callback` publishes `[0, 0]` rather than skipping the publish, so a consumer can tell
"commanded to stop" from "not being commanded at all" — and the staleness watchdogs in both
interface nodes (§5) then mean the second case also ends in zero thrust.

**`/blueboat/pinger_coordinates` carries three values, not two.** `robot_interface` seeds
`self.pinger_coordinates` from the Water Linked *filtered* (`filaco`) x/y/z
(`robot_interface.py:603`) and dead-reckons that 3-vector at odom rate, so the published array
is body-frame `[x, y, z]`. `master_control` reads `pinger_target[:2]` in the `PID` branch and
hands the whole array to `solve_LoS` in the `LoS` branch, which unpacks exactly three. The
2-element world-frame `corrected_pinger` goes out on the same topic only under
`self.fixed_pinger`, which is hard-coded `False` (`robot_interface.py:88`) and reachable from
no parameter or topic.

**`/controller_target` is published only inside the pinger branch**
(`master_control.py:514-517`), by decision rather than omission. The topic carries the
body-frame pinger vector; the world-frame target is carried by `/monitoring_data[4:6]` in
every branch (N9), which is what the station map display and the no-pinger CSV layout read.
`robot_interface` is the only subscriber in the project and stores the value without reading
it, so during path following and manual-target control the topic is silent and nothing
consumes it.

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

FCU endpoint is the `fcu_url` argument of `BlueBoat_launch.py`, defaulting to
`udp://:14550@192.168.2.2:14550`. **Port 14550 collides with a running QGroundControl**, which
manifests as intermittent launch failures — `mavros_router` logs `link[1000] open failed:
DeviceError:udp:bind: Address already in use`. Close QGC, or pass another port through
`fcu_url:=`.

### 2.5 Service

`/path_request` — `blueboat_interfaces/srv/RequestPath`.
Request: `std_msgs/Float32MultiArray path_request`, an array of **path-parameter values**.
Response: `nav_msgs/Path path`, one pose per requested value, `frame_id: "world"`.
Server: `path_generation`. Clients: `master_control`, `path_publisher`, `ur_mpc_control`,
`uvr_mpc_control`.

This contract is deliberately parameter-agnostic — the caller decides what the numbers mean.
That property is what allows the reference-generation strategy to change without touching
`path_generation`, and it must be preserved.

**Exactly one server may run, and since 2026-09-03 that is enforced rather than assumed.**
ROS 2 does not stop a second node offering the same service name, and when two do, every request
is answered by whichever replies first. The response is a bare `nav_msgs/Path` with no echo of
the request and no server identity, so the caller cannot tell. Measured on recorded logs with two
missions up: `master_control`'s reference alternated **tick by tick** between two entirely
different trajectories — the mission's `from_yaml` path and another launch's built-in `fsin` —
both sampled at its own single `tau`, and the boat could follow neither. The still-running
15:53 mission's log is 100 % corrupted across exactly the window in which further missions were
launched beside it. Two guards, either of which is sufficient alone:

* `path_generation` **refuses to be the second server**: at construction, before creating the
  service, it enumerates the graph and exits non-zero with a FATAL naming the other node.
  `allow_duplicate_server:=true` opts out; `server_discovery_wait` (2.0 s) is how long it gives
  discovery before believing an empty graph. The enumeration deliberately does **not** exclude
  itself by name — the duplicate is another node called `path_generation` in the same namespace,
  so a name test skips precisely the node it must find.
* Each returned pose is **stamped with the path parameter it was evaluated at**, not the clock
  (`path_stamp.encode`), and `master_control.accept_path` rejects any response whose pose count
  or whose `poses[0]` stamp does not match the request it actually issued. `RequestPath` is
  untouched — the tag rides in the `PoseStamped` header the message already had, so N1/CM-1 is
  not engaged. A server built before this stamps the wall clock; that is detected once, reported
  as an error telling the operator to rebuild, and falls back to a geometric plausibility check
  for the rest of the run rather than rejecting every response.

`master_control` also logs an error at startup if it sees more than one server. It cannot refuse
to run — it is the controller — but the operator is told.

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

# Simulation
ros2 launch blueboat_control Sim_launch.py
ros2 launch blueboat_control Sim_launch.py controller_type:='MPC' trajectory:='kin_square'

# Real robot
ros2 launch blueboat_control BlueBoat_launch.py
ros2 launch blueboat_control BlueBoat_launch.py enable_motors:=True controller_type:='PID' note:='testing_gains'
```

**`BlueBoat_launch.py`** — arguments `enable_motors` (False), `note` (''), `controller_type`
(''), `trajectory` ('station_keeping'), `use_pinger` (False), `fcu_url`
('udp://:14550@192.168.2.2:14550', §2.4) and `data_dir` ('', §6). It always starts `mavros`,
`robot_interface`, `uwgps_log` and `param_set`; it starts `master_control` only when
`controller_type` is non-empty, and `path_generation` only when `use_pinger` is **False** —
pinger mode needs no trajectory server. `use_pinger` reaches `robot_interface` under the
different parameter name **`use_UWgps`**, which also selects the CSV layout (§6).

**`Sim_launch.py`** — arguments `robot_file` ('thrusters_ur'), `trajectory`
('station_keeping'), `controller_type` (**default `'MPC'`**), `data_dir` ('', §6),
`note` ('sim', the position log's file-name tag, §6) and
`spawn_yaw` (0.0 — the boat's spawn heading in **radians ENU**, forwarded as
`world_launch.py`'s `yaw` into `upload_rov_launch.py`'s declared gazebo axes and Gazebo's
`-Y`; the spawn position stays (0, 0), since the pre-deployment hold pose is the world
origin). It includes `blueboat_description/world_launch.py` and starts
`simulation_interface`, `path_generation`, `path_publisher` and `master_control`. It never
starts `robot_interface`, accepts none of the real-only arguments, and always launches a
controller — the "empty `controller_type` launches no controller" rule applies to the
real-robot launch only. The Mission Control Station passes a random `spawn_yaw` when
launching a GPS-anchored mission in simulation, to rehearse anchoring at arbitrary headings.

**Testing.** No lint or type-check tooling exists, and there is no ROS-side automated test
(no `pytest`, no `ament_*` test target). One gate exists in the working tree, and it is **not
committed**: `.gitignore` excludes `.claude/tools/` and `.claude/settings.json`, so
`git ls-files .claude/` returns `CLAUDE.md` only. A fresh clone has no gate.

*Interface-contract guard* — `.claude/tools/interface_inventory.py`. Static AST extraction of
every publisher, subscriber, service, client and declared parameter in the repository — node,
resolved wire name, message type and QoS — plus the field list of every `.msg` and `.srv` in
`blueboat_interfaces`, compared against `.claude/tools/interface_baseline.json`
(100 entries + the three interface definitions). A changed message field reports as
`FIELDS <name> changed in blueboat_interfaces`.

```bash
python3 .claude/tools/interface_inventory.py --emit                                        # read the inventory
python3 .claude/tools/interface_inventory.py --check .claude/tools/interface_baseline.json  # 0 = unchanged, 2 = moved
```

stdlib only — no ROS, no sourced workspace, ~0.15 s measured — and it is wired as a
`PostToolUse` hook in `.claude/settings.json`, so an edit that renames or retypes an interface
fails at the moment it is made (N1). The compared key carries no file path or line number, so
moving code between files or renaming a local variable produces no diff; only a name, type or
QoS change does. It also reports parameters a launch file passes to a node that does not
declare them (`## launch cross-check` in `--emit`; currently empty). The baseline **matches the
tree** — `--check` exits 0. A *deliberate* contract change is a cross-repo decision (N1):
notify the consumers, then re-baseline with `--update`.

*The log reviewer's gate* — `QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py`,
89 checks, ~25 s, passing under both interpreters (§9). Unlike the interface-contract guard
above it needs no `.claude/` file, so it IS in a fresh clone — but it needs at least one real
poslog under `~/ros2_ws/data/Robot_data/` and skips cleanly (exit 0) when there is none.

**Interpreter.** Two interpreters, and they differ in what they carry.

`~/ros2_ws/.venv` is where `acados_template`, `casadi` and `pandas` live, and it is what the
**ROS nodes** need: without it `master_control` (acados + casadi) and `simulation_interface`
(casadi, through `blueboat_control.ROV`) fail at import — verified by running
`Sim_launch.py` outside the venv, where both die with `ModuleNotFoundError` before reaching
`rclpy.init`. `robot_interface` no longer needs it: its CSV writer stopped using pandas with
the 2026-08-31 logging rework (§6), so numpy is its only non-ROS dependency. The installed
executables carry `#!/usr/bin/env python3`, so activating the venv is what selects it.

`/usr/bin/python3` carries numpy, scipy, matplotlib, sympy, PyYAML and `rclpy`, but **not**
`casadi`, `acados_template` or `pandas`. `.claude/tools/interface_inventory.py` is stdlib-only
and runs unchanged under either interpreter — or under no sourced environment at all.

Beware the third one: on this machine an interactive shell resolves a bare `python3` to
`SSS-Dataset-Aug-Studio/.venv/bin/python3` (numpy and scipy, **no matplotlib, no pandas**).
Name the interpreter rather than relying on a bare `python3` in `PATH`.

**Dependencies.** `requirements.txt` pins `acados_template` (from git), `bluerobotics-ping`,
`casadi`, `Cython`, `matplotlib`, `numpy`, `pandas`, `pyserial`, `PyYAML`, `requests`,
`scipy`, `sympy`, `transformations`, `lxml` (the only unpinned entry). `casadi` and `sympy` are
load-bearing:
`blueboat_control/__init__.py` builds the thrust-allocation matrix symbolically and
`MPC/ur_mpc.py` builds the OCP with them. From apt: `xacro`, `simple_launch`, `mavros`,
`urdf_parser_py`, and **acados** for the MPC solver. `slider_publisher` and `pose_to_tf` are
required by `blueboat_description`'s spawn launch, not by any control node.

`acados_template` and `casadi` are needed to **start `master_control` at all**, not just for
`controller_type:='MPC'`: `import ur_mpc` sits at module scope and pulls both in, so the `PID`
and `LoS` paths need them even though neither uses them. (The unused `from blueboat_control
import ROV` that also did this was removed; `ur_mpc` alone still carries the requirement.)

**acados needs two things pip does not install** (`README.md` carries the commands): the
built C library (`libacados.so`) and the Tera template renderer binary at `<acados>/bin/t_renderer`.
Without the renderer, acados code generation stops on an interactive `input()` prompt — which,
under `ros2 launch`, is an invisible permanent hang rather than an error. That was the
2026-08-31 "MPC never starts" symptom. `master_control` now
preflights the renderer and runs the construction with stdin closed, so the same environment
produces a FATAL naming the fix and a non-zero exit instead of a freeze. `ACADOS_SOURCE_DIR`
should be exported: unset, acados *guesses* the path and can pick a different checkout than
the one `acados_template` was installed from (this machine has two).

---

## 4. Control architecture

Three controllers share one control callback. Branch priority: **manual target** → **path
following** → **pinger** → nothing. `MPC` is unsupported in pinger mode.

**The controller object is built in `__init__`, not on the first tick.** `_build_controller()`
runs at the end of construction, before `rclpy.spin`. This matters only for `MPC`, which runs
acados code generation and a C compile there: done from `timer_callback` (as it was until
2026-08-31) it froze the single-threaded executor for the whole build, taking odom,
`/path_request` futures and `publish_thrust` down with it. `PID` and `LoS` build a plain
Python object and are unaffected either way. One consequence to know: the log line
`Controller node initiated` now marks *construction*, and no longer implies
`/blueboat/controller_ready` has arrived — `Controller ready` is the line for that.

**The generated acados solver is cached outside the working directory**, in
`$ROS_HOME/blueboat_control/mpc` (default `~/.ros/blueboat_control/mpc`), resolved by
`ur_mpc.acados_build_dir()`. acados defaults its json and `code_export_directory` to
*relative* paths, so before 2026-08-31 the generated C landed wherever the launch was invoked
from — including, when the Mission Control Station started the run, inside the station's own
repository — and a new directory meant a full recompile. `ur_mpc.build_solver()` asks acados
for `generate=False, build=False` so it compares the stored json against the current OCP and
regenerates only on a real change: measured 1.0 s for a first build, 0.0 s to reuse. Delete
that directory to force a rebuild. Deliberately **not** under `cf.data_root()` — that tree is
write-once field record (§6 / CM-7).

**What "a real change" covers, verified 2026-09-04 against `acados_template` 0.5.1.**
`is_code_reuse_possible` compares an md5 of the whole `ocp.to_dict()`, and the serialized
`model.f_expl_expr` inside `acados_ocp.json` carries the plant coefficients `a_u…d_r` as
literals. So a *coefficient-only* retune of `mpc_model` does force a regeneration, and so does
any change to `mpc_horizon`, `mpc_time`, `mpc_Q_diag`, `mpc_R_diag`, `thrust_limit` or
`mpc_qp_iter_max` — all of them live in the compared dict. The practical consequence is that
the first launch after such a change takes about a minute and logs `generated and compiled`
rather than `reused`; that is expected, and nobody should be deleting the cache to "make it
take".

**A failed acados solve is not a command (C6).** acados leaves the primal iterate untouched
on a non-zero status, so reading `solver.get(0, 'u')` after one returns the command from the
last *successful* solve. `ur_mpc.solve` returned it, `master_control` published it, and a
constant asymmetric thruster pair is — physically — a constant-radius circle. Measured
2026-09-04: 654 of 656 ticks carrying one bit-identical command for 32.7 s while the state
changed continuously underneath it, and the boat orbited for the whole run. The latch was
self-sustaining, because a circling boat never regains the small heading error that would let
the QP solve again.

There were two causes, and the second is the bigger one. The condensed QP grew to
`nv = 60` variables when the simulation horizon doubled, against acados' default working-set
budget of 50 — so a saturating solve failed by construction (`mpc_qp_iter_max`, §4 table).
And `SQP_RTI` takes one Newton step per call from the previous iterate while `solve` re-seeds
only stage 0, so **one** bad step poisoned every later solve: measured in Gazebo, the boat was
tracking the circle at 0.05 m and 0.10 rad of error when a single tick commanded `[-20, -20]`
and the QP then failed 3291 consecutive times.

Now: a non-zero status triggers `solver.reset(reset_qp_solver_mem=1)`, a cold re-seed of every
stage at the measured state with zero input, and **one retry**; only if that also fails does
`solve` return **zeros**. It records `last_status`, `fail_count`, `total_failures`,
`recoveries` and `last_solve_time`; the MPC branch of `timer_callback` commands zero thrust
and logs `MPC solve FAILED` through `self.get_logger().error` at a 1 s throttle. Measured
after: 3 isolated failures in 149 s of Gazebo, each self-clearing on the retry, 0.03 % of
ticks at zero thrust against 93.7 % before. The recovery is failure handling, not a control
law — none of it runs on the success path. Two further details are load-bearing. It is **not** an early return — falling through keeps
`publish_thrust` and the monitoring append on the path, so `/thruster_input` never goes silent
(§5) and the `.npy` keeps recording, which is the only reason this was diagnosable. And it
goes through the **ROS logger**: the old diagnostic was a bare `print()`, which never reaches
`/rosout`, and neither `Sim_launch.py` nor the simulator's `full_mission_launch.py` captures
this node's stdout — so the failure left no trace anywhere an operator would look.

**Reference generation.** Path following advances a **path parameter `tau`** governed by the
boat's own progress (N8):

```
fac_along = clip((gov_Lmax - e_along) / (gov_Lmax - gov_Lmin), 0, 1)
fac_cross = clip((gov_Emax - |e_y|)  / (gov_Emax - gov_Emin), 0, 1)   # 1 when gov_Emax = 0
tau_dot   = path_speed_scale * fac_along * fac_cross
tau      += tau_dot * dt
```

`e_along` is the along-track gap from boat to virtual target. When the boat keeps up, the
target advances at the path's authored speed; as the gap grows it slows and finally pauses,
so it cannot outrun the boat. Each factor is clipped to `[0, 1]` and they are multiplied, so
`tau` is monotonic and bounded at the authored speed. Because authored speed is the spatial
rate of the path's own parameterisation, **a speed profile that varies along the path is
followed without extra machinery** — this is how the "desired speed at any point on the path"
requirement is met. Nothing may scale `tau_dot` by more than unity without breaking that.

`fac_cross` answers the other half of "is the boat keeping up" — a boat abreast of its target
but far off to the side is not. **It is disabled by default** (`gov_Emax = 0`), because it is
only safe once the inner loops can close a lateral gap: throttling `tau` on an error the
controller cannot reduce is positive feedback — the target stalls, the boat loses the forward
authority it converges laterally with, and the offset grows. Raise the inner gains first.

Defaults: `path_speed_scale = 1.0`, `gov_Lmin = 0.5 m`, `gov_Lmax = 3.0 m`,
`gov_Emin = 0.5 m`, `gov_Emax = 0` (cross-track gating off), `control_dt = 0.05`
(20 Hz control loop). The request sent to `path_generation` is
`linspace(tau, tau + path_time, path_steps)`, issued **asynchronously** — the result is
collected on a later tick, so the reference window is typically one or two ticks stale and
the loop never blocks on the service.

The window is the only thing `controller_type` changes about the reference:

| `controller_type` | `path_time` | `path_steps` |
|---|---|---|
| `PID`, `LoS` | 0.05 s | 2 |
| `MPC` | 2.5 s | 15 |

**Guidance.** `PIDLoS` implements canonical Fossen lookahead LoS,
`psi_d = gamma_p + atan2(-e_y, Delta)`, with path-speed feedforward and optional
turn-slowdown. An invariant holds it compatible with point-following: when `psi_path is None`
the position error is projected onto the **boat heading** and `gamma_p` is taken from
`ref[2]`; when `psi_path` is supplied it is projected onto the **path tangent**.

`PIDLoS` is always constructed with `lookahead = pid_lookahead = 2.5 m`; the class's own
backward-compatible default of `1.0` is never used. The `Delta = 1/los_gain`
re-parameterisation the class documents is an exact algebraic identity, so the claimed
equivalence to the pre-rework point controller holds only at the matching `Delta`.

**Which law runs where:**

* **Manual target** — `solve_LoS`, for every `controller_type`. `PIDLoS` is not involved.
  Once the target is reached the branch switches to **keep location** rather than stopping:
  `manual_keep_location` holds the point against drift instead of latching to zero thrust
  (§4.1).
* **Path following** — `MPC` → `ur_mpc.MPCController.solve`; `PID` → `PIDLoS.compute` with
  `u_ff` and `psi_path` supplied from the path; `LoS` → `los_guidance`.
* **Pinger** — `PID` → `PIDLoS.compute(state, target)` with `psi_path=None`, `u_ff=0`, robot
  position and yaw zeroed so the whole solve is body-frame; `LoS` → `solve_LoS`.

`solve_LoS` is a separate crude proportional point-following law (body-frame pure pursuit,
logarithmic speed in range). It is not the path LoS and is known to work as-is.

**Zero authored speed — the station-keeping hold.** `station_keeping`, a clamped-out mission
and the awaiting-YAML fallback all give a **stationary** reference, so the window's spatial
rate `U_d` is zero. Neither path controller can hold position on one, for two different
reasons, and both get the same gate: `w = 1 - U_d/hold_speed`, plus `hold_radius`, the range
inside which the boat counts as on station.

* **`LoS`** — its surge command *is* the authored speed, so it is identically zero however far
  off the boat is. Below the gate the law steers at the reference point instead of along a
  tangent that means nothing when the reference does not move, and commands
  `min(los_hold_umax, w * los_hold_kx * gap)` of surge for the range `gap` outside
  `hold_radius`. It never commands reverse — a lookahead law steers the wrong way backwards,
  so the yaw channel turns the boat round instead — and rides inside the same
  `max(0, cos(psi_err))` shaping as the feedforward.
* **`PID`** — `PIDLoS`'s outer `pid_x` loop does act on the along-track error whatever `u_ff`
  is, but it projects onto the **path tangent**, and a stationary reference has none worth
  projecting onto: a pure cross-track error produces no along-track error and therefore no
  surge. So below the gate `master_control` rotates the `psi_path` it hands the class toward
  the **bearing** to the hold point, by `w * g` where `g` fades in over `hold_radius`. The
  class's own along-track error then *is* the range and its own LoS steering points at the
  point — the same object and the same law, given a different tangent — and `slow_on_turn`
  (also the class's own option) keeps it from driving away while it turns round.

`w` is **exactly zero** for every authored trajectory in the library, so path following in
both controllers is bit-identical to what it was — but the margin is not uniform. Authored
speed over each shape's active range, measured off its own parameterisation at the 0.05 s
window: `straight_line` and `square` 0.500, `sin` 0.280–0.564, `circle` 0.320,
`seabed_scanning` 0.318–0.500, `kin_square` 0.300, and **`fsin` 0.500** (`_FSIN_V`, raised
from 0.1 in `e6dff70` — see §6). The slowest shape is `sin` at 0.280, so raising `hold_speed`
above 0.28 would start altering path following.

Every shape holds its last pose past the end of its parameter range (`sin` and `kin_square` at
t = 500, `seabed_scanning` at t = 40 + 12π ≈ 77.7 s), so U_d falls to zero there and the hold
takes over by design.

**Tuning knobs — all `declare_parameter`'d.** `_declare_tuning_parameters` declares every
one with today's value as its default, unconditionally (independent of `controller_type`), so
`ros2 param list /blueboat/master_control` shows the whole set and a gain change costs a
launch argument rather than an edit and a rebuild. Values are read once, at construction.

| group | parameters |
|---|---|
| Control loop | `control_dt` (0.05) |
| Path service health | `path_request_timeout` (1.0 s — re-issue a request whose answer never came; without it one lost response wedged the node for the whole run), `path_stale_timeout` (1.0 s — beyond this the governor stops advancing `tau` against the held window). `path_generation` adds `allow_duplicate_server` (False) and `server_discovery_wait` (2.0 s) |
| Governor | `path_speed_scale`, `gov_Lmin`, `gov_Lmax`, `gov_Emin`, `gov_Emax` |
| LoS guidance | `los_lookahead` (2.5), `los_ku` (**20.0**), `los_kpsi` (10.0), `los_kd` (1.0), `los_speed_scale` (1.0 real / 2.0 sim) |
| Station-keeping hold | `hold_speed` (0.05, the gate) and `hold_radius` (0.5), both shared by `PID` and `LoS`; `los_hold_kx` (1.0) and `los_hold_umax` (0.8), the LoS surge law only |
| PID | **Split simulation/real.** `pid_lookahead` (2.5 both); `outer_gains_x` (`[3.0, 0.01, 0]` real / `[6.0, 0.01, 0]` sim), `outer_gains_psi` (`[3.0, 0.01, 0]` / `[4.0, 0.01, 0]`), `inner_gains_u` (`[1.0, 0, 0]` / `[2.0, 0, 0]`), `inner_gains_r` (`[1.5, 0, 0]` / `[2.5, 0, 0]`) |
| MPC | **Split simulation/real, like the PID and point rows** — `mpc_horizon` (30 sim / 15 real), `mpc_time` (6.0 / 2.5), `mpc_R_diag` (0.10 / 0.015), `mpc_Q_diag` (50,50,30,1,1,1 both). The plant **model** is split too — `self.mpc_model`, not a declared parameter — with the simulation column fitted to `hydrodynamics.xacro` (added mass = the xacro values, damping = secant linearisations of its quadratic drag). Plus `mpc_qp_iter_max` (**0**, meaning "derive as `max(50, 4·nu·N)`" = 240 at N = 30, 120 at N = 15) — the qpOASES **working-set budget**, and not optional: `FULL_CONDENSING_QPOASES` is a dense active-set solver, so `nv = mpc_horizon · 2`, and acados' own default of 50 sits *below* the 60 variables of the simulation horizon. That made a saturating solve fail by construction, which is finding **C6** |
| Point following | `point_k_v` / `point_k_psi` (2.0 / 60.0 in simulation, 0.15 / 100.0 on the real boat), `safety_distance` (−1.0, which disables the arrival check — **pinger branch only** since the manual branch got its own hold) |
| Manual keep-location | `manual_hold_radius` (1.0 m, `<= 0` disables the whole hold), `manual_reacquire_radius` (2.0 m), `manual_hold_kx` (15.0 simulation / 8.0 real, **Newtons per metre**, not the m/s that `los_hold_kx` is), `manual_hold_umax` (defaults to `kx × (reacquire − hold)`, so retuning a radius cannot silently break the handover), `manual_brake_time` (1.0 s) |
| Thrust | `thrust_limit` (20.0 N) — feeds the allocator clamp, the MPC input bounds **and, since 2026-08-31, `publish_thrust`'s own uniform saturation**. `robot_interface` and `simulation_interface` each declare a parameter of the same name and default (§5) |
| Dead zone | `min_thrust` (2.0 N) — the propeller-breakaway floor on `solve_LoS`'s surge term **and on the manual keep-location surge**. `0.0` disables both and restores the pre-2026-08-31 law exactly (§5) |

ROS 2 has no dict or tuple parameter type, so gain triples and the MPC weight diagonals are
declared as double arrays and reassembled in the node. `path_time` and `path_steps` stay
**derived** from `control_dt` / `mpc_time` / `mpc_horizon` and are deliberately not declared,
so the reference window and the solver's horizon cannot disagree.

`FIELD_TUNING.md` carries the field-measured symptom→knob table for most of these.

---

## 5. Thrust path — sharp edges

- `/thruster_input` carries **`[right, left]`**. The convention is consistent across every
  code path: the allocation matrix `B = [[1,1],[0,0],[r,-r]]` with `radius = 0.59/2` puts a
  positive (CCW) yaw moment on column 0; the URDF places `thruster1` at `y = -0.295`
  (starboard) and `thruster2` at `y = +0.295` (port), and the yaw moment of a body-x force at
  `y` is `-y*Fx`, reproducing `+0.295 / -0.295` exactly; `ROV.read_model` sorts thruster
  joints alphabetically, so `forces[0]` drives `thruster1`; `simulation_interface` unpacks
  `r, l = thr_input`; `solve_LoS` builds `[v + 0.295*yaw_rate, v - 0.295*yaw_rate]`;
  `manualMove` treats `input[0]` as right; and the CLI's `move <left> <right> <s>` is stored
  as `[right, left]`.
- `left_pwm = 3000 - pwm`. The left thruster is reversed to compensate an asymmetric
  propeller; this is intentional, not a typo.
- `manualMove` contains a `compensation_gain` conditional (1.2 / 0.75) that is immediately
  overridden by a hard-coded **`1.0`**, leaving the branch dead. The conditional also keys on
  `input[1]` (left) while the gain is applied to `input[0]` (right). Do not tidy this without
  deciding what it should do.
- Thrust→PWM is a `PchipInterpolator` fitted to a measured bollard-pull table
  (`custom_functions.generate_interpolator`), so its useful range is asymmetric: about
  −27.6 N to +55.2 N.
- **Saturation is uniform everywhere, and it is one number.** PWM still clamps to
  `[1100, 1900]`, but thrust is now bounded by a single rule at every point it can leave a
  node: `ThrustAllocator.allocate`, `master_control.publish_thrust`,
  `robot_interface.manualMove` and `simulation_interface` all scale the pair by **one factor**
  rather than clipping each side, and all four read a `thrust_limit` parameter defaulting to
  20.0 N. `thrust_limits.scale_to_limit` is the shared implementation.

  **Why uniform, not per-side.** The two thrusters do not carry independent signals — they
  carry one wrench, split into a common mode and a differential (`f = X/2 ± N/0.59`). Clipping
  each side on its own does not clamp that command, it *rewrites* it:

  | | right, left | surge X | yaw N |
  |---|---|---|---|
  | commanded | `[+45, +18]` | 63.0 N | 27.0 N |
  | old per-side clip | `[+20, +18]` | 38.0 N | **2.0 N** |
  | uniform scale | `[+20, +8]` | 28.0 N | 12.0 N |

  The old clip collapsed the differential from 27 N to 2 N, so the harder the controller asked
  for a turn the straighter the boat went — it diverged from the path instead of converging.
  Uniform scaling keeps the right:left ratio, so the direction of the wrench survives and the
  boat holds its commanded turn-per-metre and simply travels it more slowly.

  Two consequences worth knowing. `publish_thrust` returns the saturated vector and
  `timer_callback` reassigns `u` from it, so `/monitoring_data[7:9]`, the `.npy` `u1/u2` and
  the CSV's `right_thr_in`/`left_thr_in` all now agree on what actually went on the wire.
  And it explains thrust recorded above the ±20 N clamp before this landed:
  **`solve_LoS` went through no allocator at all** and could emit 30–40 N, which the per-side
  clip then reshaped rather than rejected. Simulation applied no limit whatsoever.
- **The 0–2 N band does not turn the propellers, and one law lives in it.** The bollard-pull
  table maps 0 N → PWM 1500, 1 N → 1514, 2 N → 1525, so the whole 0–2 N command range lands
  inside a T200 ESC's ~±25 µs neutral deadband. The table itself has no deadband — it crosses
  zero smoothly — so nothing in software knew. Per-law radius at which per-thruster force
  falls under 2 N:

  | law | channel | radius |
  |---|---|---|
  | `solve_LoS`, pinger, **real boat** (`k_v = 0.15`) | surge | **3.28 m** |
  | `solve_LoS`, pinger, simulation (`k_v = 2.0`) | surge | 0.25 m |
  | `solve_LoS`, manual target, real boat (double log) | surge | 0.30 m |
  | PID point-following | along-track | 1.33 m |
  | LoS station-keeping hold | surge | 0.70 m |
  | PID path-following | cross-track | 0.67 m |
  | LoS path-following | cross-track | 0.30 m |

  Only the first exceeds 2 m, so only it is floored. `solve_LoS` raises its **surge** term to
  `min_thrust * g * max(0, cos(bearing))`, where `g` ramps 0→1 over `hold_radius`. Both
  factors only ever *reduce* the floor, and both reuse a blend this file already applies
  elsewhere — `g` is the PID/LoS station-keeping fade, `max(0, cos)` is `los_guidance`'s own
  feedforward shaping. They are not decoration:

  * **`max(0, cos(bearing))`** — forward surge closes the range by `cos(bearing)` only. An
    ungated floor pushed the boat *away* from a target abeam or behind it while it turned
    round; the unfloored law does not, because its surge is ~0 there. The turn needs no help
    at those bearings — the differential is already 4.6 N per side at 90°, well clear of the
    deadband. Gating cut the modified region from every bearing to **|bearing| ≤ 69.5°**.
  * **`g`** — without it the commanded surge stepped by 1.64 N as the boat crossed 0.5 m
    inbound. With it the command is continuous in `d` (verified: the largest sample-to-sample
    jump scales linearly with sample spacing, slope `min_thrust/hold_radius` = 4 N/m).

  **What is provably not touched.** The `± 0.295*yaw_rate` differential — and therefore the
  yaw moment, which way the boat turns and how hard — is bit-identical across 865 200
  (range, bearing) grid points, and the floor never *lowers* the surge at any of them. The
  modified region is **0.61–3.27 m at |bearing| ≤ 69.5°, 5.7 % of the `d ≤ 12 m` plane**;
  everywhere else the output is bit-identical, and `min_thrust = 0.0` reproduces the original
  law exactly. The manual-target *pursuit* law never engages it (its own radius is 0.30 m,
  inside the fade-in) — but since 2026-09-03 the manual **keep-location** surge carries its
  own copy of the floor, written out separately because that one must not be coupled to
  `hold_radius`. See the keep-location entry below.

  **What it does change, stated plainly.** Inside that region the floor raises `X` while
  holding `N`, so the commanded surge/yaw ratio rises and the turn radius grows — at
  `d = 1 m`, `X/N` goes 2.68 → 7.66. That is a real change to the commanded wrench. It is
  defensible only because the unfloored command in that region *produces no motion at all*:
  a closed-loop check against an explicit 2 N-per-side deadband has the boat starting 0.8–3.2 m
  dead ahead of a pinger and **never moving**, and starting at 5–8 m it closes to 3.07 m and
  stalls there — the predicted boundary. With the floor it converges from every start, with no
  hunting (tail swing < 0.01 m).

  It settles at **~0.81 m**, not at `hold_radius`: the fade-in itself dips back under the 2 N
  breakaway around 0.8 m, so the boat parks there. That is the price of continuity over a hard
  step, and it is well inside the 2 m bar this work was measured against.
- **A reached manual target is held, not abandoned (2026-09-03).** `solve_LoS` used to latch on
  arrival — one second astern, then zero thrust for the rest of the run, cleared only by a new
  target. Any current then carried the boat out of the survey area with the controller
  commanding nothing, and on the real boat the latch never armed at all (`safety_distance` is
  −1.0 there), so the point law hunted instead of settling. `manual_keep_location` replaces the
  latch with a state re-evaluated every tick: inside `manual_hold_radius` the boat is on
  station, beyond `manual_reacquire_radius` the pursuit law takes back over, and in between the
  surge is proportional to the gap, capped, `max(0, cos(bearing))`-shaped and floored to
  `min_thrust`. **The yaw channel is untouched**, so the differential — which way the boat turns
  and how hard — is the law it always was; only the common-mode surge is replaced. The gains
  are chosen so the hold meets the pursuit law at the handover rather than stepping there:
  8.38 N against 8.00 N on the real boat, 15.42 against 15.00 in simulation.

  Measured on the harness plant with an explicit 2 N per-side deadband, starting on station,
  real gains, final distance after 400 s:

  | current | keep-location | old latch | old real-boat pursuit |
  |---|---|---|---|
  | 0 N | 0.00 m | 0.00 m | 0.00 m |
  | 2 N | 1.00 m | 27.2 m | 0.30 m |
  | 8 N | 1.50 m | 108.7 m | 0.69 m |
  | 12 N | 1.75 m | 163.0 m | 1.19 m |

  The pursuit column parks tighter but is not an alternative: approaching from 5 m in calm
  water it overshoots and runs away to 472 m on this model, the C8 failure mode, while the hold
  settles at 0.22 m. The floor is what makes 1.00 m reachable — unfloored the proportional term
  does not clear 2 N until 1.25 m, so the boat cannot reach the station it was told to hold.
  `manual_hold_radius <= 0` disables the hold and restores the previous law bit-identically.

  Two limits worth keeping in mind. The floor applies to the **common-mode surge**, not per
  side, so the inner thruster can still sit under 2 N — flooring per-side would alter the
  differential, the one thing this must not do, and a per-side deadband is not invertible
  without changing the wrench. And the modified region still reaches 3.27 m rather than 2 m;
  pulling that in means raising `point_k_v` (0.15 → 0.246 puts the law's own 2 N crossing at
  exactly 2.00 m), which is a change to the control law itself — it lifts far-field surge at
  50 m from 10.70 N to 12.94 N — and has deliberately **not** been made here.
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
- **`param_set` never blocks forever, and always reports (2026-09-04).** It used to
  set `busy` before its first MAVROS call and clear it *only* in a `call_async`
  done-callback, with no timeout on any of the four calls — so a hung
  `/mavros/param/pull` latched it permanently and every later request was dropped
  with "Parameter sequence in progress" while `robot_interface` re-requested
  forever. Observed in the field as an override that never locks. Three rules now
  hold, and none may be removed:
  1. **No state is cleared only by a callback.** A 0.5 s watchdog abandons any
     sequence held longer than `param_sequence_timeout_s` (declared parameter,
     default 20 s — a cold `ParamPull` walks the whole ArduPilot table over
     MAVLink and is genuinely slow). It is a give-up bound, not an expected
     duration.
  2. **Every abandoned sequence is fenced off by a generation counter** (`_seq`,
     checked first in each done-callback), so a future that completes after its
     sequence was abandoned cannot write state belonging to the one that
     replaced it.
  3. **The node always reports.** `publish_state()` publishes `param_mode` even
     before the first successful apply (as `''` = "alive, no mode locked"), and a
     1 Hz heartbeat repeats it so a late subscriber need not wait for a
     transition. Previously the topic was silent until the first success, which
     is why `robot_interface` logged `current: ''`.
  Failures schedule a bounded internal retry (`param_retry_limit` /
  `param_retry_delay_s`) rather than stopping; the limit bounds self-driven
  retries only, since an external request resets it.
  **Downstream consequence, do not break it:** the heartbeat means a repeated
  `param_mode` value is *not* evidence that a new command was acted on. The
  Mission Control Station's safe-shutdown therefore requires a **transition** into
  `default` (`BlueBoat-MCS/.claude/CLAUDE.md` N1). `robot_interface.mode_callback`
  and `param_callback` are both edge-triggered for the same reason.

---

## 6. Data

| Artifact | Path | Nature |
|---|---|---|
| Position/pinger CSV | `<root>/data/Robot_data/{date}-{note}-poslog.csv` | **Raw field record — never overwrite or regenerate.** Written by `robot_interface` in a real-robot run and by `simulation_interface` in Gazebo |
| Controller monitoring | `<root>/data/{ctrl}_data/{date}-{ctrl}_{sim}_data.npy` | Per-run result |

`<root>` is resolved at node start by `custom_functions.data_root`, first match wins: the
`data_dir` parameter when non-empty → `$BLUEBOAT_DATA_DIR` → the sourced workspace, i.e. the
parent of the first `$COLCON_PREFIX_PATH` entry → the process working directory. In normal use
the third branch answers and both artifacts land under the workspace root (`~/ros2_ws/data/`),
**independently of the directory the launch was invoked from** — which is what keeps a run
started by the Mission Control Station from writing into the station's own repository. `data/`
is in each repository's `.gitignore`, and the workspace root is outside every repository.

`custom_functions.ensure_data_dir` creates the directory and makes an unwritable root a launch
failure naming the path, rather than a silent fallback; each node logs the artifact it opened.
Names are stamped to the second and claimed with `O_EXCL` by
`custom_functions.reserve_run_file`, so two runs starting inside the same second get `-2`,
`-3`, … instead of the later one rewriting the earlier (#6 / CM-7). Nothing in this module
writes these files back — they are read only by `poslog_report.py` (§2.1) and by the log
reviewer app (§9).

`.npy` schema: `['t','x','y','psi','x_d','y_d','psi_d','u1','u2']`, target columns world-frame
per N9. The header is appended as a row of **strings** to the same list as the float rows, so
`np.save` coerces the whole array to strings — analysis scripts must cast back on load.

**Simulation writes the same file.** `simulation_interface` carries the same logging
section: same columns, same 0.33 s rate, the same `-origin.yaml` sidecar and the same
`poslog_report` folder at shutdown, so a Gazebo run and a field run are read by one reader
and compared without a second format (N2 / CM-2). It is always the **no-pinger** layout —
there is no Water Linked UGPS in Gazebo — and three columns mean slightly less there: the
seven date columns carry **sim time** (so a simulated log reads 1970-01-01, and derived
speeds stay right at a real-time factor other than 1), `lin_acc_*` is the odometry twist
differentiated rather than an IMU reading and therefore carries no gravity component, and
`actuation_state` only ever takes 1 and 3 — 0 and 2 are hardware conditions with no
simulated counterpart. The GPS pairs fill in only when something publishes
`/mavros/global_position/global` (the MCS bridge, or the simulator's `mavros_shim_node`);
under a plain `Sim_launch.py` they stay at (0, 0), this project's "no fix". The frame needs
no re-zeroing there — `/blueboat/odom` is already local ENU about the Gazebo world origin —
so the sidecar's `yaw0_rad` records `spawn_yaw` as provenance and nothing more.
`robot_log_schema.py`'s docstring carries the same list.

The CSV still has two layouts, chosen by `use_UWgps` — a pinger is a target with genuinely
specific fields — but **as of 2026-08-31 they share the same first 19 columns structurally**:
time (7) → robot pose, world (3) → target pose, same world frame (2) → robot GPS (2) →
target GPS (2) → `right_thr_in`, `left_thr_in`, `actuation_state` (3). Only the four target
names differ, so the robot/target pair in each frame is adjacent and selects as a block.

* **no pinger, 27 columns** — target block is `target_x` / `target_y` (from
  `/monitoring_data[4:6]`, world-frame in every controller branch, N9) and
  `target_latitude` / `target_longitude`, the same point through the run's origin fix.
* **pinger, 39 columns** — target block is `corrected_pinger_x/y` and
  `pinger_latitude/longitude`, plus the raw UGPS fields (aco xyz, ant xyz, lat/lon/dep,
  filaco xyz) after the shared block. `ant_*` is all-zero unless `uwgps_log` is given its
  `--antenna` flag, which no launch file passes, and `dep` is set from the same value as
  `aco_z` rather than an independent sensor.

`actuation_state` says whether the logged command could reach the water: `0` motors disabled,
`1` live (enabled and in override), `2` enabled but not in override, `3` watchdog forcing
zero. Without it a run with the motor gate off is byte-indistinguishable from a live one, and
a watchdog trip is indistinguishable from a genuine zero command — the watchdog writes
`[0, 0]` into the same field.

Four things changed with the layouts, none of them cosmetic:

* **One writer, both layouts.** The pinger CSV used to be assembled and written inside
  `uw_gps_callback`, i.e. driven by the Water Linked link at 2 Hz, so a UGPS dropout stopped
  recording the **robot** as well. That callback now only caches its packet;
  `log_timer_callback` owns the file at 0.33 s for both layouts.
* **Append, not rewrite.** The header goes out once and each row is appended and flushed. The
  old writer accumulated a DataFrame and rewrote the whole file every row — O(n²), and despite
  its "for safety in case of unexpected shutdowns" comment, strictly *less* safe: a kill
  mid-rewrite truncates a whole file where an appended row is already on disk. The `.npy` is
  unchanged and still saved at most every 0.1 s.
* **`MicroSecond` is microseconds in both.** It was `now.microsecond // 1000`, i.e.
  milliseconds, in the no-pinger layout and true microseconds in the other.
* **No stray rows or columns.** The all-zero seed row and the unnamed pandas index column
  (which read `0` on every row) are both gone. `robot_interface` no longer imports pandas at
  all.

`quat_x/y/z/w` became `roll, pitch` in both layouts — yaw was already `relative_psi`, so the
other two Euler angles carry everything the quaternion did in half the columns
(`cf.quaternion_to_rpy`, which `quaternion_to_yaw` now delegates to).

A `<stem>-origin.yaml` sidecar is written beside the CSV carrying `latitude`, `longitude` and
`yaw0_rad`. Every world-frame column in the log lives in the local-ENU frame latched at the
first odom callback (origin previously recorded nowhere, which made a finished run impossible
to georeference afterwards). `yaw0_rad` is the boat's ENU heading at that instant —
**provenance only, not part of the frame**: since the 2026-08-31 local-ENU fix,
`relative_psi` is absolute ENU yaw; in logs recorded before it, `relative_psi` was
`yaw − yaw0_rad` (and world positions were ENU regardless), so the sidecar is what
disambiguates old data from new.

Rows are still filled **by column name**, so order can change without desynchronising the
data. Field data is campaign-bound and weather-limited — treat it as irreplaceable. The
schema revision is recorded in `robot_log_schema.py`'s docstring; `data/Robot_data/` was empty
when it landed, so no recorded field CSV was invalidated (CM-7).

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

**The target columns changed meaning once, and now mean one thing.** `cf.compute_target`
returns the *second* pose of the reference window, so `/monitoring_data[4:6]` — the CSV's
`target_x`/`target_y` and the `.npy`'s `x_d`/`y_d` — used to report the pose at `tau +
path_time` on the `PID` and `LoS` branches while the `MPC` branch reported the pose at `tau`.
All three now report `poses[0]`, through the single `_reference_pose` helper, so "target"
means the same thing whatever controller ran (N9). The control law is unchanged — it is still
handed the full `compute_target` result, velocities and all. Logs recorded before the change
carry a PID/LoS target about **2.5 cm** further along the path than a current one.


---

## 7. Trajectories

`PathGeneration.single_pose(t, shape)` — a **method**, not a module-level function, because
the `from_yaml` branch reads the node's loaded trajectory — provides: `station_keeping`,
`circle`, `straight_line`, `sin`, `fsin`, `square`, `kin_square`, `seabed_scanning`, and
`from_yaml:<abs path>`. The module-level `SHAPES` tuple and `is_valid_shape()` are the
importable half of the same contract. Every pose comes back with `frame_id: "world"`. The node
also takes a `display_log` parameter for per-request logging.

The function is pure in `t`, which is what lets the trajectory be swapped,
replayed or hot-reloaded with no coupling to the controller. **Speed is baked into each
formula** — `x = 0.5*t` means 0.5 m/s; there is no separate speed setting.

**Trajectory frame:** every shape (and every YAML `points` row) is expressed in the
`/blueboat/odom` world frame, which is **local ENU** (§2.2): origin = the boat's launch
point, `+x = East`, yaw absolute. A shape authored to start at `(0, 0)` with `yaw = 0`
therefore starts at the launch position heading **East**, identically on the real boat and
in simulation. (Before the 2026-08-31 frame fix, the position stream was ENU while the yaw
stream was launch-relative, so paths tracked correctly only when the boat launched facing
East.) GPS-anchored station missions arrive already translated into this frame by the MCS
deploy step; `path_generation` applies no transform of its own.

The hard-coded shapes are the reference conditions for existing field data; changing one
invalidates comparison against earlier runs without raising any error — `TRAJECTORY_SYSTEM.md`
§3 carries a shape revision record. `square` still carries a known defect — an instantaneous
4 m discontinuity, `TRAJECTORY_SYSTEM.md` §3.

Not every shape ends. `straight_line`, `square` and `circle` are defined for all `t` and never
clamp. The ones that do end hold their last pose, the YAML loader's convention: `sin` and
`kin_square` at `t = 500`, `seabed_scanning` at `t = 40 + 12π ≈ 77.7 s`, `fsin` at
`_FSIN_MAX_STEPS` (1e7 steps = 100 000 s). Past those points the reference stops moving, so the
station-keeping hold takes over. `fsin` takes one knob, a `radius` local to its branch of
`single_pose` (**1.5 m** since 2026-09-01, previously 0.1 m): surge is fixed, so the radius
sets the yaw-rate amplitude and the frequency follows it, scaling the weave without changing
its shape or its authored speed. `fsin` has no closed form and is
integrated by Euler at a fixed 0.01 s step, read out of an append-only cumulative table built
at module scope rather than re-integrated per pose. Each extension of that table continues the
accumulation from its stored last value, so a given `t` yields the same pose whatever order
poses are requested in — `single_pose` is pure in `t` for every shape.

An unrecognised `trajectory:=` name is refused at construction: `path_generation` logs a FATAL
naming the shape and the valid set, and exits. `single_pose` independently raises `ValueError`
with the same message, so the `if`/`elif` chain has no fall-through.

The YAML route (`blueboat_trajectory/1` — dense `[t, x, y, yaw]` samples, linear interpolation
with short-way yaw wrap-around, clamped at the final pose or wrapped when `loop: true`) is
selected as `trajectory:=from_yaml:<abs path>`. `path_generation` also declares a dedicated
`yaml_path` parameter that takes precedence when set, but **neither launch file passes it**, so
through `ros2 launch` the path has to ride inside the `trajectory` argument; `yaml_path` is
reachable only from `ros2 run` or a `--ros-args -p`. It is **file-watched**: `_maybe_reload_yaml` runs on every service
request, reloads on mtime change, and returns a station-keeping pose at the origin until the
file appears. That hold-until-present behaviour looks like a no-op but is load-bearing — it
lets the Mission Control Station deploy a GPS-anchored mission only once the run's odom↔GPS
fit is established.

---

## 8. `blueboat_description`

The URDF/xacro model, meshes, Gazebo world, and the spawn/bridge launch chain
`world_launch.py` → `upload_rov_launch.py` → `state_publisher_launch.py`.
`world_launch.py` declares `yaw` (radians, default 0) and forwards it into
`upload_rov_launch.py`'s `declare_gazebo_axes` arguments, reaching the spawn as
`ros_gz_sim create … -Y <yaw>`; its old `spawn_pose` argument was dead (never read
downstream) and is removed.

Hull: `mass = 16.01` kg (`blueboat.xacro:18`), `izz = 5.6403125` (`blueboat.xacro:42`).
`master_control` hands the MPC `robot_mass = 16.01` and `iz = 5.64` — the mass agrees exactly,
the yaw inertia is the URDF value rounded (0.006 % low). The **hydrodynamics do not agree at
all**: `hydrodynamics.xacro` carries the BlueROV2 table (`xDotU -5.5`, `xU -25.15`, plus
quadratic damping the MPC has no term for), while `master_control` passes `a_u = -26.77`,
`a_v = -7.55`, `a_r = -21.77`, `d_u = -29.34`, `d_v = -51.54`, `d_r = -44.65`
(`master_control.py:359-366`). So the Gazebo plant and the MPC's internal model share the rigid
body and nothing else — an MPC result in simulation is not a solver-against-its-own-model
result the way the offline harness's is. Thrusters
sit at `x = -0.488`, `y = ∓0.295`, `z = -0.025`, with `thruster1` on the starboard side (§5).
`thrusters_ur` (2 thrusters) is the default; `thrusters_uvr` (3) exists and is marked not
functional.

The package holds nothing else. The vendored Plankton sensor snippets (`urdf/snippets/`),
`urdf/path_markers.sdf` and the `slider_publisher` configs (`launch/manual.yaml`,
`launch/thrusters.yaml`) were deleted in the 2026-09-14 stabilization pass: nothing in the
superproject included any of them, and `blueboat.xacro`'s only sensor include had been
commented out. ⚠ One live oddity survives that deletion: `upload_rov_launch.py` defaults
`sliders:=True` and looks for `<thr>.yaml` = `thrusters_ur.yaml`, which exists nowhere — and
`Sim_launch.py` passes `sliders: False` to `world_launch.py`, which neither declares nor
forwards it. So `slider_publisher` is requested on every simulation launch and can never find
its config. Fixing it is a launch-behaviour change and has not been made.

`upload_rov_launch.py` is where simulation gets its sensing: it bridges Gazebo's odometry to
`/blueboat/odom` (the `OdometryPublisher` plugin runs at 20 Hz with `odom_frame: world`),
plus `/blueboat/pose_gt`, `joint_states` and `cmd_thruster{1,2}`. So in a real-robot run
`/blueboat/odom` comes from `robot_interface`, and in simulation it comes from the bridge —
same topic, same type, different origin.

---

## 9. `log_reviewer/` — the mission log reviewer app

A standalone **PySide6 desktop app** at the repository root for reading a recorded mission:
open a poslog CSV, trim it on a timeline that re-derives every figure and every number live,
zoom and replay the track over satellite imagery, retitle the panels, and export the trimmed
run with a fresh picture.

**It is not a ROS package.** No `package.xml`, no `setup.py`, no colcon — and it carries a
`COLCON_IGNORE`, because colcon scans every directory under `src/` and would otherwise try.
Nothing in it imports `rclpy`, so it needs no sourced workspace.

```bash
python3 log_reviewer/run.py [<csv>] [--no-fetch-tiles]
QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py     # the gate, 89 checks
```

Runs unchanged under `/usr/bin/python3` (matplotlib 3.6.3, numpy 1.26) and under
`~/ros2_ws/.venv` (3.10.8 / 2.4.4); the gate is run under both. Its dependencies — PySide6,
matplotlib, numpy, PyYAML, requests, and Pillow via matplotlib — were already in
`requirements.txt`. **It must not acquire `pandas`:** the system interpreter does not have it,
and losing that interpreter would mean the app only runs inside the venv.

### 9.1 The rule that shapes the whole app

`~/ros2_ws/data/Robot_data/` is **primary field record** (CM-7 / N7 / §6). The app opens it
**strictly read-only** and never renames, moves, rewrites or deletes anything in it. The
toolbar's rename entry renames the **export**, not the log. Everything written goes under
`~/ros2_ws/data/Processed_Robot_data/`, which is derived data, so re-exporting is expected and
overwriting is the operator's call (the app asks first).

Two functions of `poslog_report` are therefore **never called from the app**: `finalise_run`,
which MOVES a CSV and its sidecar, and `render_report`, whose first statement is
`matplotlib.use("Agg")` and would hijack the Qt backend process-wide. `smoke_test.py` §[6]
hashes every file under `Robot_data/` before and after a full session — content and mtime —
and fails if one byte moved.

### 9.2 Reuse, and the two places it deliberately stops

`reviewer/poslog_bridge.py` puts `blueboat_control/src/_custom_libraries/` on `sys.path` and
imports `poslog_report` as a plain module — it is ROS-free and numpy-only at module scope, so
this is safe from a bare Python prompt. `read_poslog`, `read_origin`, `sidecar_path`,
`speed_series`, `distance_series`, `compute_metrics`, `_valid_gps`, the formatters and the
whole palette are re-exported **unchanged**. A change to how speed is smoothed or how a metric
is computed therefore lands in the app automatically, and the app can never report a different
number than the archived per-run PNG for the same rows. The gate asserts that equality
directly.

The **painting** is the app's own (`reviewer/figures.py`), and a change to `render_report`'s
layout does *not* reach the app. That is deliberate — four things differ:

1. every title and description is operator text, not a literal;
2. the track carries satellite tiles and must survive pan and zoom;
3. **there is no actuation-state strip** (the app was asked not to have one). The
   `actuation_state` column is still read: "Thrust live (state 1)" stays in the summary, so
   dropping the plot does not drop the fact;
4. the summary lays **21 entries into 7 rows × 3 pairs**. `poslog_report._plot_table` uses
   `per_col = 6` against 3 columns — 18 slots for 19 entries — so **its last row, the world
   origin, is silently dropped from every archived PNG**. Do not copy that back. The gate
   counts rendered cells against entries.

The other reworked default is the speed panel. The PNG titles it
`Ground speed (|v|, 1 s smoothed, differenced from pose)`, which is a definition wearing a
title's clothes. The app titles it **Ground speed** and says the rest in the description
underneath, in words. The gate asserts the old parenthetical has not come back.

`poslog_report.py` itself is **not modified by any of this** — it sits on a flight node's
shutdown path.

### 9.3 Satellite tiles — MCS's cache, and three ways to get it wrong

`reviewer/tiles.py` reads the Mission Control Station's tile cache: a **flat** directory of
files at `~/.config/blueboat_mcs/tile_cache/` (relocatable — the app reads MCS's
`config.json`), Esri World Imagery, no index and no expiry. Since every mission is flown from
MCS, the area a log covers is normally already cached and the app draws offline. A miss is
fetched in the background and a hole is simply never drawn — never an error box.

Three traps, each of which fails **silently**:

* **The URL is `{z}/{y}/{x}` and the filename is `{z}_{x}_{y}`.** Esri puts row before column;
  MCS's cache name puts column before row. Swap them and you get a valid tile of somewhere
  else on Earth, drawn confidently under the track. (Verified the right way round: a tile
  fetched by this app is byte-identical to the one MCS cached for the same key.)
* **The files are not PNGs.** Esri serves JPEG and MCS writes the body to a `.png` name
  without looking. Anything that decodes by *extension* — `matplotlib.image.imread`
  special-cases `.png` — raises `SyntaxError: not a PNG file` on every tile in the cache.
  Decode by content, through PIL.
* **Writes must be atomic.** MCS writes tiles with a plain `write_bytes`, so this app writes
  `<name>.part` and `os.replace`s it; the two can share the cache with MCS running.

The three slippy-map functions are **copied** from `BlueBoat-MCS/mcs/core/geo.py`, not
imported — CM-3, no module reaches into a neighbour's package, and BlueBoat-Control carries no
path dependency on BlueBoat-MCS. If MCS ever changes that projection maths, the copies are
wrong and must be re-copied.

Tiles are placed with `imshow(extent=[lon_w, lon_e, lat_s, lat_n])` using each tile's **own**
corners in degrees, so the track panel keeps the PNG's familiar lon/lat axes and the
Mercator-on-equirectangular error stays within one tile — well under a pixel at survey scale.

### 9.4 Layout and the pieces worth knowing

| File | What it owns |
|---|---|
| `run.py` | entry point, argument parsing |
| `reviewer/poslog_bridge.py` | the reuse above, `crop_run`, `summary_rows`, `wall_clock` |
| `reviewer/figures.py` | the four painters, the table, `build_report_figure`, `DEFAULT_TEXTS` |
| `reviewer/track_view.py` | the interactive track: zoom, pan, blitted replay, hover |
| `reviewer/cursor.py` | `Overlay` (per-canvas blitting) and `LinkedCursor` (the hover dot) |
| `reviewer/timeline.py` | the two-handled range slider (Qt ships none) |
| `reviewer/tiles.py` | §9.3 |
| `reviewer/export.py` | the `Processed_Robot_data/` writer |
| `reviewer/app.py` | the window |

* **`crop_run` does not re-base mission time.** A cropped panel keeps the seconds it had in the
  whole run, so you can still see where you are. Every derived series is recomputed *from the
  crop* rather than sliced out of the full-run result: the crop is the dataset on screen, and a
  mean over a window has to be that window's mean.
* **No pyplot anywhere.** Figures are `matplotlib.figure.Figure` handed to a canvas (Qt on
  screen, Agg for the export), so the app never touches the global backend.
* **`show_headline=False` on screen.** The title and description *are* the entries above each
  canvas; drawing them inside it as well printed every heading twice. Only the export draws
  them.
* **Replay and the linked cursor are blitted, through ONE overlay per canvas.** Their artists
  are `animated=True`, so they are skipped by normal draws; the clean background is captured
  once per real draw and restored per frame. A frame therefore costs the same whether or not
  64 satellite tiles are underneath it — which is what makes a cursor that follows the mouse
  affordable at all.
  **Two blitters on one canvas erase each other**: each restores a background captured without
  the other's artists, so whichever blits second wins. The track's four replay markers and its
  two hover dots therefore share the canvas's single `Overlay`. If a third overlay-drawn thing
  is ever added to a canvas, it joins that `Overlay` — it does not make its own.
* **The linked cursor is one instant on four panels.** Hovering any plot broadcasts a mission
  time; every panel marks the nearest logged row, and the status bar reads the row out. Two
  details are load-bearing: the cursor plots the **same masked arrays the panels plot**, so a
  dot vanishes exactly where its curve does (no target, or a sample excluded as a pose jump)
  rather than claiming a value that was never drawn; and `LinkedCursor` positions its dot **by
  row index**, not by x, which is why the identical class serves both the time-series panels
  (x = time) and the track (x = longitude). Hovering the track finds the nearest sample in
  **metres** — an un-scaled lon/lat distance would snap east-west sooner than north-south —
  and marks nothing beyond a tolerance, so open water is not attributed to a row.
* **`ax.clear()` throws cursor artists away.** Every re-plot must call `cursor.set_data(...)`
  again; an artist left on a cleared axes draws nothing while still looking alive.
* **The scale bar is redrawn on every limit change.** The PNG computes it once from
  `get_xlim()`, which is correct for a static picture and stale the moment anyone zooms.
* **The export CSV is copied field by field**, never re-formatted from the parsed floats — a
  round trip through `float()` and `"%f"` would quietly rewrite field data at lower precision.
* **Nothing numpy reaches PyYAML.** `export.plain()` walks the manifest and converts every
  numpy scalar before `safe_dump`. This is not defensive tidying: `SafeDumper` represents
  Python scalars only and raises `RepresenterError: cannot represent an object` on an
  `np.float64` — and it does so at the LAST step of the export, after the CSV and the picture
  are already written, leaving a folder that looks finished but has no manifest. numpy leaks in
  invisibly, because `ax.get_xlim()` returns numpy scalars and `round()` on one returns another,
  so the value passes every `isinstance(..., float)` eye-test on the way. The bug only fired
  once someone panned or zoomed before exporting, which is why it survived the first gate;
  §[3] now exports with numpy limits on purpose. Inside `plain`, **bool is tested after
  `.item()` and before int** — `bool` subclasses `int`, and `np.bool_` subclasses neither, so
  an unordered check writes `true` out as `1`.
* **`export()` resolves its root at call time** (`root=None` → `DEFAULT_ROOT`), never as a
  default argument. A default binds once at import, so a redirected root would leave the
  caller's already-exists check guarding one folder while the write went to another — and the
  overwrite prompt would then be protecting the wrong directory. `app._export` passes
  `root=E.DEFAULT_ROOT` explicitly for the same reason.
* **The export's error path ends in a dialog, not a traceback.** `_export` catches broadly and
  names the type, the message and the folder that may hold partial files. An export crosses
  three libraries and four files; whatever one of them raises, the operator is looking at the
  window, not at the terminal, and the app has to survive for them to retry.

### 9.5 What an export contains

`~/ros2_ws/data/Processed_Robot_data/<name>/` — `<name>` from the toolbar entry, sanitised
only for path separators:

| File | Content |
|---|---|
| `<name>.csv` | the rows inside the timeline, every column, verbatim, with the source header |
| `<name>.png` | the report **as framed in the app**: edited text, current zoom, current tile setting, no actuation strip |
| `<name>-origin.yaml` | a **copy** of the run's origin sidecar, so the world-frame columns stay georeferenceable |
| `export.yaml` | `schema: blueboat_processed_log/1` — source csv/stem/sidecar/layout/target and row count, the crop (`t_start_s`, `t_end_s`, first/last source row, rows, wall-clock span), the track framing, and every edited string |

Note for re-opening an export: `poslog_report.sidecar_path` derives the sidecar name from the
`-poslog.csv` suffix, which a renamed export no longer has, so `poslog_bridge.load` falls back
to `<stem>-origin.yaml`. Without that fallback a re-opened export silently loses its origin.

### 9.6 Simulation is drawn in world coordinates, never in GPS

A run recorded in Gazebo is plotted in **local ENU metres** — east/north of the world origin —
in the archived `poslog_report` PNG and in the app alike, and **no latitude or longitude is printed
anywhere for it**: not on the track axes, not in the summary's origin cell. A simulated
position is not a surveyed one, and drawing it in degrees invites reading it as one.

**This holds even when the run carries GPS.** A GPS-anchored Gazebo mission has the Mission
Control Station synthesising `/mavros/global_position/global` from sim odom about an arbitrary
anchor, so `gps_latitude`/`gps_longitude` are populated and look exactly like field data. They
are a re-encoding of the world coordinates, not a measurement, so the frame decision **ignores
whether fixes exist** and asks what produced the run.

**The detector is the CLOCK, not the GPS** (`poslog_report.is_simulation`). `Sim_launch.py`
sets `use_sim_time=True`, so `simulation_interface` stamps every row from a clock that starts
at zero and a simulated log reads **1970** (§6) — `Year < 2000`. No real run can, and no CSV
column had to be added, which matters because the poslog layout is frozen field-record schema
(N7 / CM-7). A sidecar may also declare it outright (`frame: simulation`, or `simulation:
true`) and that wins, so a future writer can be explicit without fighting the heuristic.

Three consequences worth knowing:

* **A real run that lost its fix falls to the same world frame**, which beats the empty "no GPS
  fix in this run" panel it used to get. The default blurb distinguishes the two cases — one
  says the run is simulated, the other says it recorded no fix — so the picture never blames
  the wrong thing.
* **The frame is PINNED from the whole run** (`run["frame_mode"]`, set in `poslog_bridge.load`
  and inherited by `crop_run`'s dict copy). Without it, trimming the timeline into a stretch
  that happens to hold no fix would flip the panel from degrees to metres mid-session, which
  reads as a bug rather than as a frame change.
* **Satellite tiles are off in the world frame** and the checkbox is *disabled with a reason*
  rather than hidden — tiles are georeferenced in degrees, and a greyed box with a tooltip
  stops a simulated log looking like a broken tile cache. The scale bar goes too: in metres the
  ticks are the scale. The north arrow stays, and the aspect is simply 1.

**Field reports are untouched by all of this** — verified, not assumed: a real log rendered
through the current `poslog_report` is **byte-identical** to the same log rendered through the
committed one. The gate's `[1b]` section carries the simulated-frame checks, including that
`poslog_report` and the app agree on the frame for the same file.
