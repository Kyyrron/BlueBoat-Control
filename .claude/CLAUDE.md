# CLAUDE.md — BlueBoat-Control

Working guidance for this submodule. Read §1 (Non-negotiables) before editing anything.
Open questions, unresolved decisions and verification work live in `TODO.md`, not here.

Two long-form analyses sit inside the package and are the deeper reference for the control
stack: `blueboat_control/src/TRAJECTORY_SYSTEM.md` (where the reference target comes from)
and `blueboat_control/src/CONTROLLERS.md` (what each controller does with it, with measured
closed-loop comparisons). Their defect registers are tracked in `TODO.md`.

A third file, `blueboat_control/src/summary_controllers.md`, predates both and is **not
maintained** — it still documents `los_ku = 8.0`, which the tree left behind. `CONTROLLERS.md`
supersedes it.

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
re-expresses **pose** into a boot-relative frame (subtracting `x0, y0, yaw0`) and passes
**twist** through untouched (`robot_interface.py:492`). Two consumers depend on it staying
body-frame: `master_control` reads `current_twist[0]` as surge for the inner speed loop of both
`PID` and `LoS` (`master_control.py:401`), and the pinger dead-reckoning subtracts
`self.vel + ω × p` from body-frame pinger coordinates (`robot_interface.py:534`).

Measured against mavros 2.14.0, not inferred: `TRAJECTORY_SYSTEM.md` F8 carries the numbers.

**N4 — `enable_motors` gates thruster output.**
The gate is the early return in `manualMove` (`robot_interface.py:363`): no thrust-bearing
PWM reaches the motors unless `enable_motors:=True`. Exactly two paths write to
`/mavros/rc/override` around it, both deliberately, and neither carries thrust:
`full_stop()` calls `manualMove([0,0], force=True)`, and `mode_callback()` calls
`send_rc_override()` with neutral PWM then release when leaving override mode. Any *new*
bypass is a rule violation.

**N5 — Restore the default servo mapping before shutdown.**
`override` remaps `SERVO1/3_FUNCTION` to RC passthrough; leaving the boat in that state
disables the Xbox controller. Never set a SERVO function to `0` (Disabled) — that produces an
ArduPilot PreArm "no motor" failure. `param_set` defines `SERVO_DISABLED = 0` but never
applies it.

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
| `simulation_interface.py` | `src/` | `pid_sim` (ns `blueboat`) | Gazebo thrust bridge via `ROV`; sim-side readiness |
| `robot_interface.py` | `src/robot_interaction/` | `blueboat_controller` | MAVROS bridge, thrust→PWM, odom republish, CSV logging |
| `param_set.py` | `src/robot_interaction/` | `blueboat_parameter_control` | SERVO function + GCS sysid remapping |
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
| `robot_log_schema.py` | `src/_custom_libraries/` | **no** | The position-CSV column layouts (`COLUMNS_PINGER`, `COLUMNS_NO_PINGER`, `columns_for`) |
| `PID/PID.py`, `MPC/ur_mpc.py`, `MPC/uvr_mpc.py` | `src/PID/`, `src/MPC/` | no | Controller implementations |

`frame_math.py` and `robot_log_schema.py` are **ROS-free by construction** — numpy only, or
no imports at all. That is deliberate: both can be imported, diffed and checked from a plain
Python prompt with no sourced workspace, which is what makes the geometry and the CSV format
debuggable without a running graph. Keep them that way; anything needing `rclpy` belongs in
`custom_functions.py` instead.

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

**The `fsin` table must stay at `path_generation`'s module scope.** `_fsin_extend`,
`_fsin_state` and the `_fsin_yaw/_x/_y` globals cannot move to another module even with a
re-export: `docs/controllers/check_trajectory_library.py` resets the table by assigning
`path_generation._fsin_yaw = np.zeros(1)`, and if the globals lived elsewhere that reset
would silently become a no-op — the F1 purity check would then pass *vacuously*, which is
worse than failing.

**Three offline checks locate code by file, not by symbol table**, and crash rather than fail
cleanly if it moves house: `check_pid_equivalence.py` needs `dbl('pid_lookahead', …)` inside
`src/master_control.py`; `check_watchdog.py` needs `thruster_input_stale`, `timer_callback`,
`'thruster_input_timeout', 0.5` and `self.last_thr_rx = time.time()` inside
`robot_interface.py`, plus `timer_callback` inside `master_control.py`; `check_los_hold.py`
needs `los_guidance`, `timer_callback` and the four `dbl('hold_*', …)` defaults inside
`master_control.py`. All three are order-independent, so reordering within a file is safe.

### 2.2 Internal topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface` (real boat) · Gazebo bridge (simulation, §8) | `master_control`, `simulation_interface` |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | `robot_interface` | `master_control` |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface` · `simulation_interface` | `master_control` |
| `/thruster_input` | `std_msgs/Float32MultiArray` | `master_control` | `robot_interface`, `simulation_interface` |
| `/controller_target` | `std_msgs/Float32MultiArray` | `master_control` — **pinger branch only** | `robot_interface` (stored, never read) |
| `/monitoring_data` | `std_msgs/Float32MultiArray` | `master_control` (`simulation_interface` creates the publisher but never publishes — its monitoring block is commented out) | `robot_interface` |
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
('station_keeping'), `controller_type` (**default `'MPC'`**) and `data_dir` ('', §6). It includes
`blueboat_description/world_launch.py` and starts `simulation_interface`, `path_generation`,
`path_publisher` and `master_control`. It never starts `robot_interface`, accepts none of the
real-only arguments, and always launches a controller — the "empty `controller_type` launches
no controller" rule applies to the real-robot launch only.

**Testing.** No lint or type-check tooling exists, and there is no ROS-side automated test
(no `pytest`, no `ament_*` test target). Two gates exist in the working tree, and **neither is
committed**: `.gitignore` excludes `.claude/tools/`, `.claude/settings.json` and
`.claude/specs/`, and the six harness scripts below (`check_*.py`, `replay.py`) are untracked.
`git ls-files .claude/` returns `CLAUDE.md` and `TODO.md` only. A fresh clone has neither gate,
so anything that says "the diff in the commit is the record" is aspirational, not current.

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

*Closed-loop controller harness* at `blueboat_control/src/docs/controllers/` —
`sim.py` (plant + controllers), `run_sims.py` (scenarios, cached), `gen_figures.py` (plots),
`analyze.py` (summary tables). It needs only numpy, scipy and matplotlib: no ROS, no acados,
and it runs end to end under `/usr/bin/python3` on this machine.
It imports the **real** `PID.PIDLoS` class and reimplements `los_guidance`, `solve_LoS`, the
governor, `single_pose` and `compute_target` verbatim, so controller changes can be evaluated
without a workspace. Nothing in the language enforces that "verbatim", so
`check_trajectory_library.py` asserts it for `single_pose` and `check_los_hold.py` for the two
control laws. It is the evidence behind every number in `CONTROLLERS.md`. Its `PID`,
`LoS` and `Point-LoS` results are the real code and reproduce bit-for-bit; its `MPC` result is
SciPy SLSQP against the MPC's own internal model, so MPC comparisons are its weakest evidence
and move with the SciPy/BLAS build. `sim.py` and `gen_figures.py` resolve their paths from
`__file__`, so both run from any working directory and `gen_figures.py` writes beside itself.
`run_sims.py` caches one `.pkl` per scenario into `docs/controllers/cache/` (gitignored) and
`analyze.py` is importable — its report is behind `main()`. The cache is keyed on the scenario
name alone,
with no hash of the code that produced it, so it does **not** invalidate when a controller or
the plant changes: delete `cache/` after touching either, or `analyze.py` reports numbers from
whatever code last filled it.

```bash
cd blueboat_control/src/docs/controllers && python3 run_sims.py && python3 analyze.py
python3 gen_figures.py            # rewrites the nine checked-in fig*.png in place
```

*Offline replay* — `replay.py` scores a **recording** rather than a simulated run: a rosbag2
directory (`/blueboat/odom`, `/monitoring_data`, `/thruster_input`), a controller `.npy` log or
a position `.csv`. It reports `analyze.py`'s own metrics for what the boat did against the
target it was given, and optionally replays a chosen controller over the logged states to show
what it would have commanded. Recordings are opened read-only (#6). `rosbag2_py` is imported
lazily, so only the bag reader needs a sourced workspace; `tau` is not in any recording, so the
progress column reads `n/a`.

```bash
python3 replay.py <bag-dir|log.npy|poslog.csv> [--controller PID|LoS|MPC]
```

*Five checks*, plain scripts with exit codes, no test framework. Untracked (see above), so
they exist only in this working tree:

```bash
python3 check_pid_equivalence.py  # PIDLoS: Delta = 1/los_gain identity, the documented
                                  # point-following defaults, and that master_control's
                                  # pid_lookahead still implies the los_gain the equivalence
                                  # was claimed for. Reads master_control.py statically (it
                                  # cannot be imported without acados).
python3 check_replay.py           # replay cross-validation: a simulation written out as a
                                  # bag and as an .npy must read back and reproduce its own
                                  # numbers. Skips the bag half without rosbag2_py.
python3 check_watchdog.py         # loss-of-reference watchdog: the staleness predicate
                                  # against a fake clock, and that both interface nodes and
                                  # master_control's early returns still implement it.
                                  # stdlib only - no numpy, no ROS.
python3 check_los_hold.py         # zero-speed hold, both controllers: bit-identical logs
                                  # with the hold on and disabled on every moving path, a
                                  # bounded error at rest, and that the harness copies and
                                  # master_control are still the same two laws.
python3 check_trajectory_library.py  # every built-in shape against embedded reference poses
                                  # (the field-data comparability guard), the t>500 clamp,
                                  # fsin bit-identical to the original Euler loop and pure in
                                  # t, an unknown shape diagnosable, and that sim.py's copy of
                                  # single_pose has not drifted from path_generation. Imports
                                  # path_generation, so it needs a sourced workspace; skips
                                  # cleanly, exit 0, without one.
```

**All five pass on this machine** (exit 0 each, verified with the system `python3`).

An earlier reading recorded `check_trajectory_library.py` as exiting 1 on
`sin: 4 reference poses bit-identical -- moved at t=[500.0]` — a one-ULP
`scipy.spatial.transform.Rotation` quaternion difference against the embedded reference table,
not a shape change. It does not reproduce here. The check demands bit-identity of the
quaternion columns, so it stays sensitive to the scipy/BLAS build it runs on and may exit 1
again on a different interpreter; treat that specific failure as an environment difference,
not a moved shape, and confirm x/y/yaw before believing it.

The pre-rework `los_gain` is **not recoverable from this repository** — `PID.py` exists only
from the initial commit and already carries the reworked signature. `check_pid_equivalence.py`
therefore asserts 0.4 as a live coupling to `pid_lookahead = 2.5`, not as recovered history.

**Interpreter.** Two interpreters, and they differ in what they carry.

`~/ros2_ws/.venv` is where `acados_template`, `casadi` and `pandas` live, and it is what the
**ROS nodes** need: without it `master_control` (acados + casadi), `robot_interface` (pandas)
and `simulation_interface` (casadi, through `blueboat_control.ROV`) all fail at import. The
installed executables carry `#!/usr/bin/env python3`, so activating the venv is what selects
it.

`/usr/bin/python3` carries numpy, scipy, matplotlib, sympy, PyYAML and `rclpy`, but **not**
`casadi`, `acados_template` or `pandas`. Everything under `docs/controllers/` therefore runs
there unchanged — `run_sims.py`, `analyze.py`, `gen_figures.py`, `replay.py` and all five
checks, verified by running them. `check_watchdog.py` and `interface_inventory.py` are the only
two that are stdlib-only.

Beware the third one: on this machine an interactive shell resolves a bare `python3` to
`SSS-Dataset-Aug-Studio/.venv/bin/python3` (numpy and scipy, **no matplotlib, no pandas**), so
`python3 gen_figures.py` fails there while `/usr/bin/python3 gen_figures.py` succeeds. Name the
interpreter.

**Dependencies.** `requirements.txt` pins `acados_template` (from git), `bluerobotics-ping`,
`casadi`, `Cython`, `matplotlib`, `numpy`, `pandas`, `pyserial`, `PyYAML`, `requests`,
`scipy`, `sympy`, `transformations`, `lxml` (the only unpinned entry). `casadi` and `sympy` are
load-bearing:
`blueboat_control/__init__.py` builds the thrust-allocation matrix symbolically and
`MPC/ur_mpc.py` builds the OCP with them. From apt: `xacro`, `simple_launch`, `mavros`,
`urdf_parser_py`, and **acados** for the MPC solver. `slider_publisher` and `pose_to_tf` are
required by `blueboat_description`'s spawn launch, not by any control node.

`acados_template` and `casadi` are needed to **start `master_control` at all**, not just for
`controller_type:='MPC'`: `import ur_mpc` and `from blueboat_control import ROV` sit at module
scope, so the `PID` and `LoS` paths import both even though neither uses them. `TODO.md` holds
it.

---

## 4. Control architecture

Three controllers share one control callback. Branch priority: **manual target** → **path
following** → **pinger** → nothing. `MPC` is unsupported in pinger mode.

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
only safe once the inner loops can close a lateral gap; `TODO.md` F5 holds the measurements
and what gates it.

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
`seabed_scanning` 0.318–0.500, `kin_square` 0.300, and **`fsin` 0.080–0.100** — nominal surge
0.1 m/s, so barely 1.6× the gate. Raising `hold_speed` above 0.08 would start altering `fsin`
path following; above 0.28 it would reach `sin`.

`check_los_hold.py` asserts the inertness rather than assuming it, but for **four shapes only**
(`straight_line`, `circle`, `kin_square`, `sin`) — `sim.py`'s plant carries copies of five
shapes and none of `fsin`, `square` or `seabed_scanning`, so those three are argued from the
speeds above, not from a run.

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
| Governor | `path_speed_scale`, `gov_Lmin`, `gov_Lmax`, `gov_Emin`, `gov_Emax` |
| LoS guidance | `los_lookahead` (2.5), `los_ku` (**20.0**), `los_kpsi` (10.0), `los_kd` (1.0), `los_speed_scale` (1.0) |
| Station-keeping hold | `hold_speed` (0.05, the gate) and `hold_radius` (0.5), both shared by `PID` and `LoS`; `los_hold_kx` (1.0) and `los_hold_umax` (0.8), the LoS surge law only |
| PID | `pid_lookahead` (2.5), `outer_gains_x`, `outer_gains_psi` (both `[3.0, 0.01, 0.0]`), `inner_gains_u` (`[1.0, 0, 0]`), `inner_gains_r` (`[1.5, 0, 0]`) |
| MPC | `mpc_horizon` (15), `mpc_time` (2.5), `mpc_Q_diag`, `mpc_R_diag` |
| Point following | `point_k_v` / `point_k_psi` (2.0 / 16.0 in simulation, 0.15 / 10.0 on the real boat), `safety_distance` (−1.0, which disables the arrival check) |
| Thrust | `thrust_limit` (20.0 N) — feeds both the allocator clamp and the MPC input bounds |

ROS 2 has no dict or tuple parameter type, so gain triples and the MPC weight diagonals are
declared as double arrays and reassembled in the node. `path_time` and `path_steps` stay
**derived** from `control_dt` / `mpc_time` / `mpc_horizon` and are deliberately not declared,
so the reference window and the solver's horizon cannot disagree.

`CONTROLLERS.md` §6–§7 carries measured sweeps for most of these.

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
- PWM clamps to `[1100, 1900]`. Thrust is clamped twice, by two *independent* numbers:
  `ThrustAllocator.allocate` scales uniformly under saturation to preserve direction, bounded by
  the `thrust_limit` parameter (20.0 N); `manualMove` then re-clips each side to a **hard-coded**
  `±20.` (`robot_interface.py:377-380`). Raising `thrust_limit` alone therefore buys nothing on
  the real boat — the second clip has no parameter.
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
- Readiness handshakes survive DDS discovery races, but by two different mechanisms.
  `robot_interface` re-publishes `/blueboat/controller_ready` on a 1 s timer. `param_set`
  does **not** run a timer: `publish_state()` fires only when a request arrives or a
  set/verify sequence finishes — what repeats is `robot_interface` **re-requesting** the mode
  every second, which `param_set` handles idempotently. Editing either side means keeping
  that pairing intact.

---

## 6. Data

| Artifact | Path | Nature |
|---|---|---|
| Position/pinger CSV | `<root>/data/Robot_data/{date}-{note}-poslog.csv` | **Raw field record — never overwrite or regenerate** |
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
`-3`, … instead of the later one rewriting the earlier (#6 / CM-7). The only reader in the
project is this module's own `docs/controllers/replay.py`, which opens both layouts read-only;
nothing writes them back.

`.npy` schema: `['t','x','y','psi','x_d','y_d','psi_d','u1','u2']`, target columns world-frame
per N9. The header is appended as a row of **strings** to the same list as the float rows, so
`np.save` coerces the whole array to strings — analysis scripts must cast back on load.

The CSV has two layouts, chosen by `use_UWgps`:

* **pinger** — `corrected_pinger_x/y` (world frame), `pinger_latitude/longitude`, and the 19
  raw UGPS fields (date ×7, aco xyz, ant xyz, lat/lon/dep, filaco xyz);
* **no pinger** — `target_x` / `target_y` taken from `/monitoring_data[4:6]`, so they are
  world-frame for every controller. There is no `target_psi` column.

Both fill rows **by column name**, so column order can be changed without desynchronising the
data. The CSV is rewritten in full on every write (crash safety); the `.npy` is saved at most
every 0.1 s to avoid corruption from a too-frequent callback. Field data is campaign-bound and
weather-limited — treat it as irreplaceable.

All of it comes from a single site, which bounds what any control result derived from it
supports: state such a result as single-site, not general. `project_synthesis.md` §10 and §4.1
govern how results from this project are phrased; §4's evidence layers cover the sonar and
policy work and do not extend to control-loop performance, so these artifacts are the only
evidence behind a control claim.

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

The hard-coded shapes are the reference conditions for existing field data; changing one
invalidates comparison against earlier runs without raising any error, which is why
`check_trajectory_library.py` pins every one of them against embedded reference poses and
`TRAJECTORY_SYSTEM.md` §3 carries a shape revision record. `square` still carries a known
defect — an instantaneous 4 m discontinuity, `TRAJECTORY_SYSTEM.md` §9 F6.

Not every shape ends. `straight_line`, `square` and `circle` are defined for all `t` and never
clamp. The ones that do end hold their last pose, the YAML loader's convention: `sin` and
`kin_square` at `t = 500`, `seabed_scanning` at `t = 40 + 12π ≈ 77.7 s`, `fsin` at
`_FSIN_MAX_STEPS` (1e7 steps = 100 000 s). Past those points the reference stops moving, so the
station-keeping hold takes over. `fsin` has no closed form and is
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

`upload_rov_launch.py` is where simulation gets its sensing: it bridges Gazebo's odometry to
`/blueboat/odom` (the `OdometryPublisher` plugin runs at 20 Hz with `odom_frame: world`),
plus `/blueboat/pose_gt`, `joint_states` and `cmd_thruster{1,2}`. So on the real boat
`/blueboat/odom` comes from `robot_interface`, and in simulation it comes from the bridge —
same topic, same type, different origin.
