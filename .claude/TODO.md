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

- [ ] **Gazebo generation mismatch — port the Fortress plugin names.** Every plugin in
      `blueboat_description` is declared with Ignition Fortress names
      (`ignition-gazebo-*-system`, `ignition::gazebo::systems::*`) across `world.sdf`,
      `blueboat.xacro`, `hydrodynamics.xacro`, `thrusters_ur.xacro`, `thrusters_uvr.xacro`.
      **Answered on the Linux machine:** the installed generation is Gazebo **Harmonic**,
      `gz sim` 8.11.0, ROS 2 Jazzy — plugin libraries
      `/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8-*-system.so`; Fortress is not
      installed (no `ign` binary, no `*ignition-gazebo*system*` library on disk). The
      Fortress names **do** load, through Harmonic's deprecated-name compatibility path —
      `gz sim -s -r -v 4 blueboat_description/urdf/world.sdf` prints, per plugin,
      `[Wrn] [SystemLoader.cc:75] Trying to load deprecated plugin [ignition-gazebo-physics-system].
      Using [gz-sim-physics-system] instead.` plus the matching `SystemLoader.cc:136` line for
      the class name, and then loads it. So this is a deprecation-warning and
      forward-compatibility item (the shim is removed in gz-sim 9 / Ionic), not a live
      failure. Remaining work: rename to `gz-sim-*` / `gz::sim::systems::*` in those five
      files, accepting that it breaks any Fortress machine.
      `BlueBoat-SSS-Sim` has already set its own side to `gz` everywhere (generated worlds
      load with zero deprecation lines) and does not touch these files, per CM-3.
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

- [ ] **C1 — PID inner-loop gains ~30× below the drag coefficients.** `inner_gains['u'] = 1.0`
      against `d_u = 29.34` and `inner_gains['r'] = 1.5` against `d_r = 44.65` are unchanged,
      by decision: they remain the values the boat's existing field data was recorded at.
      Measured cost on the harness: cruise 0.235 m/s against an authored 0.50, mission
      progress 0.43×, acquisition RMS 0.661 m. The `los_ku` half of this item is done
      (8.0 → 20.0). Both gains are now declared parameters, so a candidate set costs a launch
      argument rather than a rebuild — sweep at the dock rather than in a rebuild loop.
      A full u × r sweep is recorded in `CONTROLLERS.md` §6; of 35 sets, 8 improve every
      scenario without steady-state saturation, `u = 5 / r = 30` being the strongest
      (acquisition 0.661 → 0.015 m, circle 0.097 → 0.011 m, cruise 0.460 m/s, progress 0.86×).
      **This gates F5 below.** `master_control.py:279-280`.
- [ ] **`stopping_sequence` latches and is never reset.** `solve_LoS` sets it on arrival
      (`master_control.py:676`) and nothing clears it — not `manual_target_callback` (`:210`),
      not `pinger_callback` (`:200`) — so once armed, every later point-LoS command (manual
      target *and* pinger) returns zero thrust for the rest of the run. Inert at the shipped
      default `safety_distance = -1.0` (`:291`), which no launch file overrides. But it is a
      declared parameter, so a launch argument arms it, and both `CONTROLLERS.md` §7 ("If you
      change only five things", row 5) and finding **C3** recommend setting it to 1.5 m — the
      recommendation and the latch have never been recorded together. Reset the flag when a
      new target arrives. Surfaces on the basestation as a boat that stops responding with no
      indication why; the station must not compensate, see
      `BlueBoat-MCS/.claude/specs/robot-side-limitations-watchlist.SPEC.md`.
- [ ] **C9 — MPC horizon shorter than one turning radius.** `mpc_time = 2.5 s` covers 0.80 m
      of travel against a 1.89 m minimum turning radius. `master_control.py:154-155`.
- [ ] **F5 — the governor's cross-track term is built but disabled.** `advance_governor` now
      takes `e_y` and applies a second unit-bounded factor, parameterised by `gov_Emin` /
      `gov_Emax`; `gov_Emax = 0` disables it and is the default, so today's behaviour is
      unchanged (verified bit-identical on all five harness scenarios).
      It is off because **it is unsafe at the current inner gains**: throttling `tau` on an
      error the controller cannot reduce is positive feedback — the target stalls, the boat
      loses the forward authority it converges laterally with, and the offset grows. Measured
      at the shipped gains: acquisition RMS 0.661 → 3.508 m and progress 0.43× → 0.11×;
      the 10 N side-current case 5.483 → 12.469 m. At `u = 5 / r = 30` the same term is
      neutral-to-better everywhere (acquisition 0.015 → 0.011 m, circle and square unchanged,
      side-current +3 %). **Raise the inner gains (C1), then set `gov_Emax` — 5.0 is a
      reasonable starting point — and re-run the five scenarios.**
      `master_control.py:585-622`.
- [ ] **F4 — MPC reads 16 poses from a 15-pose window** and pads by duplicating the last one,
      giving a zero-velocity terminal reference; separately the window spacing is 2.5/14 =
      0.1786 s while the solver divides by 2.5/15 = 0.1667 s, inflating every reference speed
      by 7.1 %. `path_steps = mpc_horizon + 1` fixes both. Measured effect on tracking is
      negligible — fix for correctness, not for accuracy. `ur_mpc.py:216-218, 156`.
- [ ] **C2 — MPC heading wrap-around never reconciled.** Reference yaw is wrapped then
      unwrapped forward across the horizon while the measured state is separately wrapped, so
      a ±π crossing shows the cost an error of up to 2π. `ur_mpc.py:226-236`.
- [ ] **`master_control` cannot start without acados, whatever the controller.**
      `import ur_mpc` at `master_control.py:73` is unconditional and `ur_mpc.py:6` imports
      `acados_template` at module level, so `controller_type:='PID'` and `'LoS'` die with
      `ModuleNotFoundError` on a machine without acados. `from blueboat_control import ROV`
      pulls in `casadi` the same way, and `master_control` never constructs `ROV`. Confirmed
      by running it. Import `ur_mpc` inside the `controller_type == 'MPC'` branch and drop the
      unused `ROV` import, so the PID and LoS paths need neither.

## 4. Unresolved decisions

- [ ] **`compensation_gain` in `robot_interface.manualMove`.** The 1.2 / 0.75 conditional is
      dead behind a hard-coded `1.0`, and it keys on `input[1]` (left) while the gain is
      applied to `input[0]` (right). Decide the intended behaviour rather than deleting the
      branch blindly. Three unknowns, none of them answerable from the code: whether the right
      thruster really is weaker, whether a single multiplicative gain stacked on an already
      asymmetric bollard-pull interpolator is the right shape for the correction, and whether
      keying on `input[1]` was a typo or a deliberate (odd) design. **Needs the boat on blocks**
      with a thrust or current measurement; simulation cannot substitute, because
      `simulation_interface` never calls `manualMove` and the asymmetry is physical. Establish
      the `/thruster_input` → servo wiring (§2) **first**, or the measurement is read off the
      wrong side. `robot_interface.py:358-369`.
- [ ] **The two standalone MPC nodes.** `MPC/ur_mpc_control.py` and `MPC/uvr_mpc_control.py`
      are installed by `CMakeLists.txt`, both claim node name `mpc_control`, and neither
      launch file starts them. Decide whether they are superseded by `master_control`'s MPC
      branch and can go, or whether they are still wanted.

## 5. Tooling gaps

- [ ] **The replay harness has never been run against a field bag.** `docs/controllers/replay.py`
      is validated against a simulation round-trip (`check_replay.py`) and against the
      2026-08-27 `.npy` logs in `~/ros2_ws/data/PID_data/`, both simulation-derived. No
      recording from the boat exists on this machine — no `.mcap`, no `.db3`, and
      `data/Robot_data/` is empty. Replay a real field bag before quoting any replayed number
      as a field result. This needs an existing recording, not a field session.
- [ ] **A recording cannot recover the reference the controller actually saw.**
      `/monitoring_data` publishes one target pose per tick (`win[0]`), while the controller
      consumes `win[1]` and the window's own span as the speed feedforward `U_d`. Neither is
      on the wire, so `replay.counterfactual` rebuilds the window from consecutive logged
      targets, which span the *governed* advance instead. At the current inner gains
      (0.43× throttle) that costs ~0.17 N RMS on the replayed command; `check_replay.py`
      bounds it at 0.25 N and measures it rather than tolerating it. Publishing `U_d` would
      close it, but that is an interface change (N1) and a cross-repo decision — do not make
      it to suit the harness alone.
- [ ] **Recorded thrust exceeds the ±20 N clamp.** Replaying the two long 2026-08-27 logs
      reports mean |thrust| 24.0 N with 100 % of the tail on the limiter, and single samples
      at 30.6 N, against `thrust_limit = 20.0`. In the current tree the limits are built at
      `master_control.py:318-321` and passed to both `PIDLoS` and the LoS allocator, and
      `ThrustAllocator.allocate` scales uniformly, so the current code should not be able to
      produce this. **Cause not established** — most likely the recordings predate that
      wiring, or the run overrode `thrust_limit`. Reproduce with a Gazebo run before treating
      it as a live defect. Surfaced by `replay.py`, not by inspection.

Nothing beyond these is justified by evidence yet. In particular, do not add lint/format/docs
pipeline scaffolding: no recurring need for it appears anywhere in this module's history.
