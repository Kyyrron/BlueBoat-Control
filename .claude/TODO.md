# TODO — BlueBoat-Control

Everything actionable: open questions, untested assumptions, unresolved decisions, known
limitations, and justified automation. `CLAUDE.md` describes current state only; anything
needing verification or a decision lives here.

**Environment marker.** Items tagged **NOT VERIFIABLE ON THIS MACHINE (Windows, no
ROS2/colcon)** need the Linux development machine — a sourced ROS 2 workspace, `colcon`,
Gazebo, MAVROS, or the boat itself. They have not been attempted here and must not be
guessed at.

---

## 1. Needs a running ROS 2 / Gazebo workspace

- [ ] **Gazebo generation mismatch.** Every plugin in `blueboat_description` is declared with
      Ignition Fortress names (`ignition-gazebo-*-system`,
      `ignition::gazebo::systems::*`) across `world.sdf`, `blueboat.xacro`,
      `hydrodynamics.xacro`, `thrusters_ur.xacro`, `thrusters_uvr.xacro`, while the README
      states Gazebo Harmonic. Establish which generation the tree actually targets: either
      the plugins resolve through a Harmonic compatibility path, or the names need porting to
      `gz-sim-*` / `gz::sim::systems::*`.
      **NOT VERIFIABLE ON THIS MACHINE (Windows, no ROS2/colcon)** — needs Gazebo.
- [ ] **`sliders` launch argument goes nowhere.** `Sim_launch.py` passes `sliders: False` to
      `world_launch.py`, which neither declares nor forwards it; `upload_rov_launch.py` is
      the file that declares `sliders` (default True) and is included without it, so
      `slider_publisher` starts regardless of the request. Confirm whether the undeclared
      argument also raises at launch, then either forward it or drop it.
      **NOT VERIFIABLE ON THIS MACHINE (Windows, no ROS2/colcon)**.
- [ ] **`builtin_interfaces` dependency.** `blueboat_interfaces/CMakeLists.txt` lists
      `builtin_interfaces` in `rosidl_generate_interfaces(... DEPENDENCIES ...)`, but
      `package.xml` declares no dependency on it. `ProcessedSSSPing.msg` uses
      `builtin_interfaces/Time`. Confirm a clean build from an empty workspace.
      **NOT VERIFIABLE ON THIS MACHINE (Windows, no ROS2/colcon)**.

## 2. Needs real hardware or a field session

All five **NOT VERIFIABLE ON THIS MACHINE (Windows, no ROS2/colcon)** — they need the boat,
a MAVLink link, or water.

- [ ] **`/thruster_input` → servo wiring.** The `[right, left]` convention is now verified
      statically end to end (allocation matrix ↔ URDF geometry ↔ `ROV` ordering ↔
      `simulation_interface` ↔ `solve_LoS` ↔ `manualMove` ↔ CLI), so the remaining unknown is
      one link only: that ArduPilot's `SERVO1` is physically the right thruster and `SERVO3`
      the left. Cheapest test: `move` with one side only, boat on blocks, watch which
      propeller turns. If that link is reversed, steering is mirrored on the real boat while
      simulation stays correct.
- [ ] **The governor has never run on the boat** — validated only in numerical simulation
      (sine path, 0.98 correlation, `tau` self-regulating instead of running away). Test
      `straight_line`, then `sin`, at the dock before any survey.
- [ ] **Governor and lookahead tuning.** `gov_Lmin`/`gov_Lmax` (0.5 / 3.0 m) and Δ (2.5 m)
      are reasoned starting values, not measured. Expect to retune for the real boat.
- [ ] **MPC solve time at 20 Hz.** The loop rate requires acados to solve in under 50 ms;
      never timed on target hardware. If it overruns, raise `dt` — the governor rescales with
      it automatically.
- [ ] **Mid-mission MAVLink mission swap** corner cases are untested. Plan was exhaustive
      SITL testing; fallback is to replan only between missions.

## 3. Confirmed defects

Each verified against the tree. Ordered by value; identifiers are those of
`blueboat_control/src/TRAJECTORY_SYSTEM.md` (F-series, which skips F17) and `CONTROLLERS.md`
(C-series), where the reasoning and the measured impact live.

- [ ] **C1 — inner-loop gains ~30× below the drag coefficients.** `inner_gains['u'] = 1.0`
      against `d_u = 29.34`, `inner_gains['r'] = 1.5` against `d_r = 44.65`, `los_ku = 8.0`.
      Measured cost: cruise speed 0.267 m/s against an authored 0.50, mission progress
      0.49×, circle error 1.745 m. `master_control.py:169-170, 195`.
- [ ] **C9 — MPC horizon shorter than one turning radius.** `mpc_time = 2.5 s` covers 0.80 m
      of travel against a 1.89 m minimum turning radius. `master_control.py:132-133`.
- [ ] **F2 — `LoS` cannot station-keep.** `u_cmd = los_speed_scale * U_d * max(0, cos(psi_err))`
      is identically zero when the authored speed is zero, so `station_keeping`, a finished
      mission and the awaiting-YAML fallback all drift with no restoring force.
      `master_control.py:310`.
- [ ] **F18 — no zero-thrust on loss of reference.** Several `timer_callback` paths return
      early without publishing, and `robot_interface` keeps streaming the last received
      `thruster_input` to the motors, so a stalled `master_control` leaves the boat running.
      A watchdog on the interface side is the safer fix. `robot_interface.py:815`.
- [ ] **F1 — `fsin` re-integrates from t=0 on every evaluation**, 0.01 s steps in a Python
      loop, per pose. `path_publisher` requesting 10 001 poses makes this appear to hang the
      launch. `path_generation.py:195-204`.
- [ ] **F9 — an unknown `trajectory:=` name crashes the service.** `single_pose` is an
      `if`-chain with no `else` and no defaults, so a typo leaves `x` unbound →
      `UnboundLocalError` inside the handler → "Nothing to target yet." forever with no hint
      why. `path_generation.py:101`.
- [ ] **F8 — body-frame velocity correction commented out**, with an in-file comment arguing
      it is needed. Conflicts directly with N3, which asserts the opposite. Resolve one way
      or the other and make N3 and the code agree. Needs a bench test, not just an edit.
      `robot_interface.py:523-544`.
- [ ] **F5 — the governor ignores cross-track error.** Only `e_along` throttles `tau`, so a
      boat abreast of its target but far off to the side sees no throttling at all.
      `master_control.py:441`.
- [ ] **F4 — MPC reads 16 poses from a 15-pose window** and pads by duplicating the last one,
      giving a zero-velocity terminal reference; separately the window spacing is 2.5/14 =
      0.1786 s while the solver divides by 2.5/15 = 0.1667 s, inflating every reference speed
      by 7.1 %. `path_steps = mpc_horizon + 1` fixes both. Measured effect on tracking is
      negligible — fix for correctness, not for accuracy. `ur_mpc.py:216-218, 156`.
- [ ] **C2 — MPC heading wrap-around never reconciled.** Reference yaw is wrapped then
      unwrapped forward across the horizon while the measured state is separately wrapped, so
      a ±π crossing shows the cost an error of up to 2π. `ur_mpc.py:226-236`.
- [ ] **F3 — `path_publisher` requests the path once, blocking, in `__init__`** and
      republishes that frozen `Path` forever, so a GPS-anchored YAML mission shows as a single
      dot at the origin for the whole run. `path_publisher.py:38-48`.
- [ ] **F16 — every tuning constant is hard-coded** rather than `declare_parameter`'d, so
      every gain change needs a rebuild. These are exactly the knobs wanted on a boat ramp.
- [ ] **F7 — `sin` and `kin_square` jump backwards** when the parameter runs out
      (`if t > 500: t = 50`) instead of clamping, unlike every other trajectory and the YAML
      loader. `path_generation.py:166, 232`.
- [ ] **F15 — `single_request` is dead code** publishing to a `self.pose_publisher` that is
      never created. `path_generation.py:317`.

## 4. Unresolved decisions

- [ ] **`compensation_gain` in `robot_interface.manualMove`.** The 1.2 / 0.75 conditional is
      dead behind a hard-coded `1.0`, and it keys on `input[1]` (left) while the gain is
      applied to `input[0]` (right). Decide the intended behaviour rather than deleting the
      branch blindly. `robot_interface.py:322-333`.
- [ ] **Fragile relative data paths** (`../../../../data/Robot_data/`, `data/{ctrl}_data/`)
      depend on the launch working directory. Worth making robust, but any change breaks
      downstream analysis scripts — coordinate before touching.
- [ ] **FCU port 14550 collides with a running QGroundControl.** Decide whether to
      parameterise the endpoint or just document the constraint for operators.
- [ ] **The two standalone MPC nodes.** `MPC/ur_mpc_control.py` and `MPC/uvr_mpc_control.py`
      are installed by `CMakeLists.txt`, both claim node name `mpc_control`, and neither
      launch file starts them. Decide whether they are superseded by `master_control`'s MPC
      branch and can go, or whether they are still wanted.
- [ ] **`/controller_target` is published only in the pinger branch** (F10), so anything
      downstream watching the target during path following or manual control receives
      nothing. `world_target` is already computed in every branch; the publish just needs
      hoisting — but that changes what a live topic emits, so treat it as an interface
      decision under N1.

## 5. Tooling gaps

- [ ] **The controller harness does not run as checked in.** `docs/controllers/sim.py:24-28`
      resolves `SRC` from a path that predates the move into the `BlueBoat-Control/`
      submodule directory, and the hard-coded fallback omits that directory level, so
      `import PID` raises `ModuleNotFoundError`. Confirmed by running it. With the correct
      path supplied externally the harness runs and reproduces `CONTROLLERS.md` §5.1 exactly
      (PID: RMS 0.661 m, cruise 0.235 m/s, progress 0.43×; LoS: 1.184 m, 0.107 m/s, 0.23×).
      One-line fix; do it before anyone trusts or re-runs the figures.
- [ ] **Rosbag replay harness — still the strongest automation case.** The closed-loop
      harness at `docs/controllers/` covers *simulated plant* evaluation and closes much of
      the gain-tuning loop offline. What it does not do is replay a **recorded field bag**
      through the controller and emit the trajectory-vs-target comparison, which is the cycle
      N7 forbids repeating in the water. Build that on top of the existing harness rather
      than starting fresh.
- [ ] **`PIDLoS` point-following equivalence is untested and will regress silently.** The
      class documents that its defaults (`lookahead = 1.0`, `u_ff = 0.0`, `psi_path = None`)
      reproduce the pre-rework point controller, and the `Delta = 1/los_gain`
      re-parameterisation is an exact algebraic identity. But `master_control` always
      constructs it with `lookahead = 2.5`, so the equivalence holds only if the pre-rework
      `los_gain` was 0.4 — and that value is not anywhere in the tree. Establish the old
      gain, then commit a small runnable check (the harness already imports the real class,
      so this is cheap).
- [ ] **Interface-contract guard — a hook.** N1 is the constraint most exposed to an agentic
      refactor. A pre-commit or post-edit hook that extracts the
      publisher/subscriber/service/client inventory and fails on any name or type change
      would enforce mechanically what vigilance currently enforces.

Nothing beyond these is justified by evidence yet. In particular, do not add lint/format/docs
pipeline scaffolding: no recurring need for it appears anywhere in this module's history.

## 6. Known limitations to keep visible

- [ ] Hard-coded trajectory shapes are the reference conditions for existing field data;
      changing one invalidates comparison with earlier runs with no error raised.
- [ ] Single-site field data caps how far any result generalises — state plainly in write-ups
      rather than overreaching.
- [ ] `CONTROLLERS.md`'s comparisons grade the MPC against the very model it assumes, and
      substitute SciPy SLSQP for acados. Trends and magnitudes hold; exact traces will differ
      on the water, and the MPC's advantage will narrow.
