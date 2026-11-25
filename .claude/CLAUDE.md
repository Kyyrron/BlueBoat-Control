# CLAUDE.md — BlueBoat-Control

Working guidance for this submodule. Read §1 before editing anything.

Deeper references: `blueboat_control/src/TRAJECTORY_SYSTEM.md` (where the reference comes
from: trajectory library, τ, governor), `FIELD_TUNING.md` (every tuning knob, real/sim
defaults, symptom→knob index), `log_reviewer/README.md`. Nested `CLAUDE.md` files load in
their directory: `blueboat_control/src/` (measurements and derivations behind §4–§5),
`blueboat_description/` (§8), `log_reviewer/` (§9). The **live path update** (the
BlueBoat-Replanner entry point) is documented in `docs/`: `PATH_UPDATE_DESIGN.md` (why),
`PATH_UPDATE_INTERFACE.md` (the client contract), `REFERENCES.md`, `REQUESTS.md` (what other
modules need), `PATH_UPDATE_REAL_WATER_TEST.md`. The **simulated plant and its configurations**
(2026-11-13): `docs/PARAMETER_REGISTER.md` (every number, its origin and interval),
`FIELD_TUNING.md` "Simulation configurations", `docs/REFERENCES.md` "Plant identification".
Open items: `.claude/TODO.md`. On 2026-11-18 §2–§9 of this file moved **verbatim** to `.claude/interface.md`,
`.claude/build-run.md`, `.claude/control-architecture.md` and `.claude/data-trajectories.md` (section numbers
kept); §0–§1 stay here.

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

**Detail loads by itself:** each `.claude/rules/*.md` is a path-scoped rule that imports one `.claude/` topic
file when you read or edit a matching file; the tables below name the same files for reading by hand.

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
`/mavros/cmd/command` calls stall the command plugin for seconds on one lost ACK (the legacy
`set_servo()` that used them was removed 2026-11-25).

**N7 — Controller iteration happens offline** (rosbags, the harness of §3), never in a field
session.

**N8 — The reference advances from the boat's measured progress, never from wall-clock
time.** The governor (`path_progress_errors` + `advance_governor`, §4) enforces it.
`current_time` only timestamps `/monitoring_data` and the `.npy` (from the run's start in a
session) and times the brake holds and path-request timeouts; never route it back into path
generation. Its clock is `time_source` (wall by default, the node clock in simulation).

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

**Simulation (simulator frozen 2026-11-18).** Simulated results follow `../SIM_VALIDITY_CONDITIONS.md`
(superproject root).
- `sim_config:=field_equivalent` (the identified plant, `docs/PARAMETER_REGISTER.md`) is **frozen with the
  simulator**: no retuning of the plant, no change of its coefficients, odometry profile or thrust timing.
  `legacy` (the BlueROV2 plant) is never a results configuration.
- **MPC with the field gains is the headline controller; LoS is the high-disturbance arm** (simulated MPC
  yaws 25–46 % less than the real boat on straight legs, simulated LoS more: together they bracket the real
  motion). Speed within 1 %, thrust per metre within 13 %, cross-track 25–95 % high, turns 12–30 % long.
- A time difference inside **±3 % plus 0.6 s per turn** (MPC) is not a result.

---

## 2. Interface (full: `.claude/interface.md`)

Executables (node names in `.claude/interface.md` §2.1): `master_control.py` (`master_control`),
`robot_interface.py` (`blueboat_controller`), `simulation_interface.py` (`pid_sim`), `param_set.py`
(`blueboat_parameter_control`), `uwgps_log.py`, `pinger_estimator.py` (`pinger_estimator`, since
2026-11-24 the one producer of `/blueboat/pinger_coordinates` and `/blueboat/pinger/*`; pinger modes
`off|follow|log`, live tuning file, interface §2.7), `path_generation.py`, `path_publisher.py`,
`bag_replay.py`.
Operator commands on `/blueboat/input_str` (`std_msgs/String`): `enable` · `disable` · `stop` · `override` ·
`default` · `arm` · `disarm` · `rearm` · `manual` · `move <left> <right> <seconds>`. Services
`/path_request` (`RequestPath.srv`, `path_generation` only) and `/blueboat/update_path` (`UpdatePath.srv`,
live path update for a replanner-owned run; contract `docs/PATH_UPDATE_INTERFACE.md`). Run-session plumbing
(`/blueboat/run_command`, `run_state`, `run_log`) only in a real-robot launch with `session_mode:=true`.
MAVROS boundary: superproject skill `interface-inventory`. Read `.claude/interface.md` before adding or
consuming any topic, service or parameter (N1).

## 3. Build, launch, run (full: `.claude/build-run.md`)

```bash
colcon build && source /opt/ros/jazzy/setup.bash && source install/setup.bash   # from ~/ros2_ws
ros2 launch blueboat_control Sim_launch.py controller_type:='MPC' trajectory:='kin_square'
ros2 launch blueboat_control BlueBoat_launch.py enable_motors:=True controller_type:='PID' note:='testing_gains'
python3 test/mc_harness.py --compare HEAD --param gain_profile:=sim   # N11: fixed missions bit-identical
python3 .claude/tools/interface_inventory.py --check .claude/tools/interface_baseline.json  # 0 = unchanged
```

`session_mode`, the arm-before-override sequence, `path_source`, `record_bag`, `Replay_launch.py` and the two
simulations of `Sim_launch.py` (`sim_config`): `.claude/build-run.md`. Gates and pass counts: superproject skill
`module-gates`.

## 4–9. Where the rest lives

| Before … | Read |
|---|---|
| changing a controller, the guidance law, the governor or the replanner hold (§4) | `.claude/control-architecture.md`, `blueboat_control/src/CLAUDE.md` |
| touching the thrust path, the calibration table, RC override (§5) | `.claude/control-architecture.md` §5 |
| reading or writing CSV logs, `.npy`, run bags, reports (§6) | `.claude/data-trajectories.md` §6 (known data caveats) |
| a trajectory or `from_yaml` path (§7) | `.claude/data-trajectories.md` §7, `blueboat_control/src/TRAJECTORY_SYSTEM.md` |
| the robot description, hydrodynamics, `sim_config` plant (§8) | `blueboat_description/CLAUDE.md`, `docs/PARAMETER_REGISTER.md` |
| the log reviewer (§9) | `log_reviewer/CLAUDE.md` |
| a field tuning knob | `FIELD_TUNING.md` |
| the live path update | `docs/PATH_UPDATE_DESIGN.md`, `PATH_UPDATE_INTERFACE.md`, `REQUESTS.md` |

Superproject rules hold here: never `git commit`, `git push` or `git tag`; this module changes by additions only,
on an explicit user order.
