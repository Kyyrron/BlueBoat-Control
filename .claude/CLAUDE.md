# CLAUDE.md — BlueBoat-Control

Working guidance for this submodule. Read §1 (Non-negotiables) before editing anything.
Open questions, unresolved decisions and verification work live in `TODO.md`, not here.

Two long-form analyses sit inside the package and are the deeper reference for the control
stack: `blueboat_control/src/TRAJECTORY_SYSTEM.md` (where the reference target comes from)
and `blueboat_control/src/CONTROLLERS.md` (what each controller does with it, with measured
closed-loop comparisons). Their defect registers are tracked in `TODO.md`.

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
`/mavros/local_position/odom` has `child_frame_id: base_link`, so `twist` is already
body-frame (surge/sway/yaw-rate) while `pose` is world-frame. `robot_interface` re-expresses
**pose** into a boot-relative frame (subtracting `x0, y0, yaw0`) and passes **twist** through
untouched (`robot_interface.py:519-521`).

A block that would rotate the linear velocity by `-yaw0` sits **commented out** at
`robot_interface.py:523-544`, with an in-file comment arguing it is required — that without
it pose and twist live in two frames differing by a constant rotation, producing "a fixed
diagonal drift and mirroring heading-swept paths". `master_control` reads `current_twist[0]`
as body-frame surge (`master_control.py:416`), feeding the inner speed loop of both `PID` and
`LoS`. The rule above and that comment assert opposite things, and the code currently follows
the rule. Do not enable or delete the block on either authority alone; `TODO.md` holds the
decision.

**N4 — `enable_motors` gates thruster output.**
The gate is the early return in `manualMove` (`robot_interface.py:316`): no thrust-bearing
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
tuning can compensate for it. The governor at `master_control.py:274-284` is what enforces
this.

`current_time = time.time() - self.initial_time` (`master_control.py:407`) still exists, but
it only timestamps `/monitoring_data` and the `.npy` log — it does not touch the reference.
Do not "fix" it by routing it back into path generation; that is precisely the design this
replaced.

**N9 — Monitoring output is world-frame in every controller branch.**
`/monitoring_data` and the `.npy` log carry world-frame `x_d, y_d, psi_d` for manual, pinger,
LoS, PID and MPC alike; mixed frames corrupt the station map display. Each branch sets
`world_target` and monitoring reads that. `/controller_target` deliberately keeps its original
body-frame content for the pinger case — the two are different signals and must not be
unified.

---

## 2. Interface — the authoritative contract

The control nodes write topic and service names **absolutely** (leading `/`) in the source.
`master_control` is constructed with `namespace='blueboat'`, but ROS 2 does not namespace
absolute names, so they resolve exactly as written. Rewriting them as relative names would
silently rename half the interface (N1).

Two components use **relative** names instead, resolved through their node's namespace, and
land on the same wire names: `path_publisher` (`set_path`, `path_request` — root namespace)
and the `ROV` helper in `blueboat_control/__init__.py` (`odom`, `joint_states`,
`robot_description`, `cmd_<thruster>` — under `blueboat`).

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

The last two claim the same node name and are started by neither launch file.

### 2.2 Internal topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface` (real boat) · Gazebo bridge (simulation, §8) | `master_control`, `simulation_interface` |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | `robot_interface` | `master_control` |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface` · `simulation_interface` | `master_control` |
| `/thruster_input` | `std_msgs/Float32MultiArray` | `master_control` | `robot_interface`, `simulation_interface` |
| `/controller_target` | `std_msgs/Float32MultiArray` | `master_control` — **pinger branch only** | `robot_interface` |
| `/monitoring_data` | `std_msgs/Float32MultiArray` | `master_control`, `simulation_interface` | `robot_interface` |
| `/blueboat/param_str` | `std_msgs/String` | `robot_interface` | `param_set` |
| `/blueboat/param_ready` | `std_msgs/Bool` | `param_set` | `robot_interface` |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set` | `robot_interface` |
| `/uw_gps_data` | `std_msgs/Float32MultiArray` | `uwgps_log` | `robot_interface` |

**`/blueboat/controller_ready` has two publishers with different QoS.** `robot_interface`
uses depth 10 volatile and re-publishes every second; `simulation_interface` uses depth 1
**TRANSIENT_LOCAL** (latched) and publishes once. `master_control` subscribes with depth 10
volatile, which is compatible with both — a transient-local publisher satisfies a volatile
subscriber, not the reverse. Making the subscriber latched would break the real-boat path.

**`/controller_target` is published only inside the pinger branch**
(`master_control.py:507-510`). During path following and manual-target control the topic is
silent, even though `world_target` is computed in every branch.

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

FCU endpoint is hard-coded in `BlueBoat_launch.py`: `udp://:14550@192.168.2.2:14550`.
**Port 14550 collides with a running QGroundControl**, which manifests as intermittent launch
failures.

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

Any **unrecognised** string falls through to `move_callback` (`robot_interface.py:439`), so
`1.0 1.0 5` is accepted as a move without the `move` keyword. A malformed command (not
exactly four fields) is rejected with a log line and no action.

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
(''), `trajectory` ('station_keeping'), `use_pinger` (False). It always starts `mavros`,
`robot_interface`, `uwgps_log` and `param_set`; it starts `master_control` only when
`controller_type` is non-empty, and `path_generation` only when `use_pinger` is **False** —
pinger mode needs no trajectory server. `use_pinger` reaches `robot_interface` under the
different parameter name **`use_UWgps`**, which also selects the CSV layout (§6).

**`Sim_launch.py`** — arguments `robot_file` ('thrusters_ur'), `trajectory`
('station_keeping'), `controller_type` (**default `'MPC'`**). It includes
`blueboat_description/world_launch.py` and starts `simulation_interface`, `path_generation`,
`path_publisher` and `master_control`. It never starts `robot_interface`, accepts none of the
real-only arguments, and always launches a controller — the "empty `controller_type` launches
no controller" rule applies to the real-robot launch only.

**Testing.** No lint, type-check or ROS-side automated test exists. What does exist is a
checked-in closed-loop simulation harness at `blueboat_control/src/docs/controllers/` —
`sim.py` (plant + controllers), `run_sims.py` (scenarios, cached), `gen_figures.py` (plots),
`analyze.py` (summary tables). It needs only numpy, scipy and matplotlib: no ROS, no acados.
It imports the **real** `PID.PIDLoS` class and reimplements `los_guidance`, `solve_LoS`, the
governor, `single_pose` and `compute_target` verbatim, so controller changes can be evaluated
without a workspace. It is the evidence behind every number in `CONTROLLERS.md`.

```bash
cd blueboat_control/src/docs/controllers && python run_sims.py && python gen_figures.py
```

**Dependencies.** `requirements.txt` pins `acados_template` (from git), `bluerobotics-ping`,
`casadi`, `Cython`, `matplotlib`, `numpy`, `pandas`, `pyserial`, `PyYAML`, `requests`,
`scipy`, `sympy`, `transformations`, `lxml`. `casadi` and `sympy` are load-bearing:
`blueboat_control/__init__.py` builds the thrust-allocation matrix symbolically and
`MPC/ur_mpc.py` builds the OCP with them. From apt: `xacro`, `simple_launch`, `mavros`,
`urdf_parser_py`, and **acados** for the MPC solver. `slider_publisher` and `pose_to_tf` are
required by `blueboat_description`'s spawn launch, not by any control node.

---

## 4. Control architecture

Three controllers share one control callback. Branch priority: **manual target** → **path
following** → **pinger** → nothing. `MPC` is unsupported in pinger mode.

**Reference generation.** Path following advances a **path parameter `tau`** governed by the
boat's own progress (N8):

```
tau_dot = path_speed_scale * clip((gov_Lmax - e_along) / (gov_Lmax - gov_Lmin), 0, 1)
tau    += tau_dot * dt
```

`e_along` is the along-track gap from boat to virtual target. When the boat keeps up, the
target advances at the path's authored speed; as the gap grows it slows and finally pauses,
so it cannot outrun the boat. `clip(..., 0, 1)` makes `tau` monotonic and bounds it at the
authored speed. Because authored speed is the spatial rate of the path's own
parameterisation, **a speed profile that varies along the path is followed without extra
machinery** — this is how the "desired speed at any point on the path" requirement is met.

Defaults: `path_speed_scale = 1.0`, `gov_Lmin = 0.5 m`, `gov_Lmax = 3.0 m`, `dt = 0.05`
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

**Tuning knobs.** Governor: `path_speed_scale`, `gov_Lmin`, `gov_Lmax`. LoS guidance:
`los_lookahead`, `los_ku`, `los_kpsi`, `los_kd`, `los_speed_scale`. PID: `pid_lookahead`,
`outer_gains`, `inner_gains`. MPC: `mpc_horizon`, `mpc_time`, `Q_weight`, `R_weight`,
`input_bounds`. Point-following: `k_v` / `k_psi` (2.0 / 16.0 in simulation, 0.15 / 10.0 on
the real boat) and `safety_distance` (−1.0, which disables the arrival check).

**None of these are `declare_parameter`'d.** Every one is a hard-coded attribute in
`master_control.__init__`, so changing any of them requires an edit and a rebuild.
`CONTROLLERS.md` §6–§7 carries measured sweeps for most of them.

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
- PWM clamps to `[1100, 1900]`; thrust clamps to `±20 N`; allocation scales uniformly under
  saturation to preserve direction.
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
| Position/pinger CSV | `../../../../data/Robot_data/{date}-{note}-poslog.csv` | **Raw field record — never overwrite or regenerate** |
| Controller monitoring | `data/{ctrl}_data/{date}-{ctrl}_{sim}_data.npy` | Per-run result |

Both paths are **relative to the process working directory**, so they depend on where the
launch was invoked; changing either is a breaking change for downstream analysis scripts.
Both directories are created with `os.makedirs(..., exist_ok=True)` at node start. `data/` is
in `.gitignore`; the CSV's four-level path puts it outside the repository entirely.

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

---

## 7. Trajectories

`path_generation.single_pose(t, shape)` provides: `station_keeping`, `circle`,
`straight_line`, `sin`, `fsin`, `square`, `kin_square`, `seabed_scanning`, and
`from_yaml:<abs path>`. Every pose comes back with `frame_id: "world"`. The node also takes a
`display_log` parameter for per-request logging.

The function is stateless and pure in `t`, which is what lets the trajectory be swapped,
replayed or hot-reloaded with no coupling to the controller. **Speed is baked into each
formula** — `x = 0.5*t` means 0.5 m/s; there is no separate speed setting.

The hard-coded shapes are the reference conditions for existing field data; changing one
invalidates comparison against earlier runs without raising any error. Several of them carry
known defects (`fsin` re-integration cost, `square` discontinuity, `sin` / `kin_square`
backward wrap) — see `TODO.md`.

The YAML route (`blueboat_trajectory/1` — dense `[t, x, y, yaw]` samples, linear interpolation
with short-way yaw wrap-around, clamped at the final pose or wrapped when `loop: true`) is
selected either as `trajectory:=from_yaml:<abs path>` or as `trajectory:=from_yaml` with
`yaml_path:=<path>`. It is **file-watched**: `_maybe_reload_yaml` runs on every service
request, reloads on mtime change, and returns a station-keeping pose at the origin until the
file appears. That hold-until-present behaviour looks like a no-op but is load-bearing — it
lets the Mission Control Station deploy a GPS-anchored mission only once the run's odom↔GPS
fit is established.

---

## 8. `blueboat_description`

The URDF/xacro model, meshes, Gazebo world, and the spawn/bridge launch chain
`world_launch.py` → `upload_rov_launch.py` → `state_publisher_launch.py`.

Hull: `mass = 16.01` kg, `izz = 5.6403125` — the same values `master_control` hands the MPC
solver, so the simulated hull and the MPC's internal model agree by construction. Thrusters
sit at `x = -0.488`, `y = ∓0.295`, `z = -0.025`, with `thruster1` on the starboard side (§5).
`thrusters_ur` (2 thrusters) is the default; `thrusters_uvr` (3) exists and is marked not
functional.

`upload_rov_launch.py` is where simulation gets its sensing: it bridges Gazebo's odometry to
`/blueboat/odom` (the `OdometryPublisher` plugin runs at 20 Hz with `odom_frame: world`),
plus `/blueboat/pose_gt`, `joint_states` and `cmd_thruster{1,2}`. So on the real boat
`/blueboat/odom` comes from `robot_interface`, and in simulation it comes from the bridge —
same topic, same type, different origin.
