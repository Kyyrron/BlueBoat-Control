# TODO — BlueBoat-Control

Everything actionable: open questions, untested assumptions, unresolved decisions, known
limitations, and justified automation. `CLAUDE.md` describes current state only; anything
needing verification or a decision lives here.

**Environment marker.** Items tagged **NOT VERIFIABLE ON THIS MACHINE (Windows, no
ROS2/colcon)** need the Linux development machine — a sourced ROS 2 workspace, `colcon`,
Gazebo, MAVROS, or the boat itself. They have not been attempted here and must not be
guessed at.

---

## 0.0 Bench verification of the 2026-09-04 safety rework

Four changes landed together, in `param_set.py` and `robot_interface.py`. The
desk-checkable half is green (`check_watchdog.py` and the other seven `check_*.py`
pass, `interface_inventory.py --check` re-baselined for the three new declared
parameters, and the Mission Control Station's `smoke_test.py` covers the station
side). None of it is confirmed against a real autopilot.

- [ ] **Neutral hold with the gate closed (N4).** `enable_motors:=False`,
      transmitter on, **props clear**: `SERVO1/3_FUNCTION` reach 51/53, and
      `ros2 topic echo /mavros/rc/override` shows a steady 1500/1500 instead of
      silence. Moving the sticks must not move the thrusters. This is the
      reported "motors spun with the box unchecked" symptom: passthrough was
      mapped, and the old silent gate left the channels to the transmitter.
- [ ] **`param_set` cannot latch (§ the three rules in `CLAUDE.md` §5).** Pull
      the MAVROS link mid-sequence: the watchdog must abandon the attempt within
      `param_sequence_timeout_s` (20 s), log it once, and accept the next
      request. Then confirm the retry locks override when the link returns.
      Measure a cold `ParamPull` on the real link while you are there — if it is
      anywhere near 20 s, raise the parameter rather than removing the watchdog.
- [ ] **The `stop` latch (N4b).** With a controller running and motors enabled,
      publish `stop`: thrust must go to zero and *stay* zero while
      `master_control` keeps publishing, with `param_mode` still `override`.
      Then `enable` and confirm thrust resumes. Confirm
      `/blueboat/controller_ready` carries the `False` the station waits on.
- [ ] **Shutdown hooks (N5).** Ctrl-C the launch and confirm the servo mapping is
      restored, the CSV is closed, and `Robot_data/<stem>/` appears with the
      report PNG. Both restores are best-effort with mavros dying in the same
      process group — record whether they actually complete on the real boat, or
      whether the operator `default` remains the only reliable route.
- [ ] **matplotlib on the boat.** `poslog_report` imports it lazily and degrades
      to "folder and files, no picture". Confirm which behaviour the boat gets,
      and either install it in `/blueboat_ws`'s venv or accept rendering the
      reports ashore with `--all`.

---

## 0. Field verification of the 2026-08-31 local-ENU frame fix

- [ ] **Verify the odom frame fix on the water.** `robot_interface.odom_callback` no longer
      re-zeroes yaw: `/blueboat/odom` is now local ENU (position translated to the launch
      point, yaw absolute — 0 = East, CCW+), matching the simulator's frame in kind. This
      was the root cause of "trajectory following only works when launched facing East",
      wrong manual-target behaviour and broken GPS anchoring (the old frame mixed ENU axes
      with launch-relative yaw; the LoS heading error carried a constant `yaw0` bias, so
      cross-track equilibrium was `Δ·tan(yaw0)`, divergent ≥ 90°). `cf.local_to_enu` (which
      also carried a spurious −π/2) is deleted; pinger/target GPS conversions are identity
      by construction now. **Requires rebuilding the boat's `/blueboat_ws`** — a stale boat
      build silently reintroduces the hybrid frame. Field checks: trajectory following from
      a non-East launch heading; manual target landing where clicked; `relative_psi` in the
      CSV reading the compass-consistent ENU heading (note: logs recorded BEFORE the fix
      have `relative_psi = yaw − yaw0_rad`; the `-origin.yaml` sidecar disambiguates).
      **NOT VERIFIABLE ON THIS MACHINE** (needs the boat).

---

## 0.1 acados / MPC startup — fixed 2026-08-31, environment half not in the repo

- [x] **MPC missions hung forever at launch.** `controller_type:='MPC'` left the boat
      motionless while `PID` and `LoS` were fine. Cause: the acados Tera template renderer
      (`t_renderer`) was **absent from the machine entirely**, so `AcadosOcpSolver`
      construction reached `acados_template.utils.get_tera()` and blocked on its
      `input("... download tera? y/N")` prompt. That construction ran inside
      `timer_callback`, on the single-threaded executor, so the whole node froze: no odom,
      no `/path_request` futures, no `publish_thrust`. stdout is block-buffered under
      `ros2 launch`, so the prompt was never even flushed. Fixed in three places —
      `t_renderer` installed; construction moved into `Controller.__init__` via
      `_build_controller()`; a preflight (`_check_acados_ready`) plus an stdin guard
      (`_no_stdin`) that turn any residual prompt into an immediate FATAL. Verified: the
      negative test (renamed `t_renderer`) now exits 1 in under a second with the install
      commands in the log, instead of hanging.
- [ ] **The environment half is not carried by this repository.** `t_renderer`,
      `ACADOS_SOURCE_DIR` and the built `libacados.so` live on the machine, not in the tree;
      `requirements.txt` pins only the `acados_template` Python package. `README.md` now
      documents the prerequisites, but a fresh clone on a fresh machine still has to do them
      by hand, and nothing checks them until a launch. Consider a `check_acados.py` in the
      untracked harness alongside the other five checks.
- [ ] **`LD_LIBRARY_PATH` is load-bearing but worked around.** `libacados.so` carries no
      rpath to `libblasfeo` / `libhpipm` / `libqpOASES_e`, so a launch without the acados
      lib directory exported died with `libqpOASES_e.so: cannot open shared object file`.
      `ur_mpc.preload_acados_libs()` now dlopens those three by absolute path with
      RTLD_GLOBAL before acados loads, which makes the export optional. Confirm this is
      still true against a differently built acados before relying on it on the boat.

## 0.2 MPC simulation tuning — done 2026-08-31, with two open consequences

- [x] **MPC saturated the thrusters in simulation.** Root causes, in order of size:
      the C2 heading-wrap fault above; an MPC plant model that did not describe the Gazebo
      boat (surge mass 2.0× heavy, **yaw inertia 4.8× heavy**, no quadratic drag at all, so
      it under-predicted the thrust needed at 0.62 m/s by 36 %); and `mpc_R_diag = 0.015`,
      at which any error beyond 0.49 m makes full saturation the cheaper option. Fixed by a
      simulation/real split on the model, `mpc_R_diag` (0.10) and the horizon (6.0 s / 30),
      in the same idiom as the PID and point-following gains. Real-boat configuration
      unchanged. Measured: 0.70 m/s survey 77.5 % → 24.3 % saturated and 1.61 m → 0.30 m
      error; 0.45 m/s survey 64.8 % → 1.4 % and 11.57 m → 0.06 m. `CONTROLLERS.md` §4.1.
- [ ] **Re-run the saturation measurements: they were taken with a failing solver.** The
      "77.5 % → 24.3 %" and "64.8 % → 1.4 %" figures above were measured on the
      `mpc_horizon = 30` configuration whose QP failed on 58-100 % of ticks (§0.4). A frozen
      command sitting on the ±20 N bound counts as "saturated" in that statistic, so both the
      before and after numbers are suspect in an unknown direction. Re-run
      `docs/controllers/mpc_tuning_report.py` on a post-fix run before quoting them anywhere.
- [ ] **The simulator's speed ceiling is 0.78 m/s and nothing enforces it.** The Gazebo hull
      uses the BlueROV2 drag table, whose quadratic term needs 20.1 N per thruster at
      0.78 m/s. Straight-line cruise at 0.70 m/s already consumes 85 % of the limit, so every
      turn saturates — the residual 24 % above is physics, not tuning. PID logs 100 %
      saturation at an authored 0.84 m/s. The MCS Pattern Designer has no knowledge of this
      ceiling; simulated missions should be authored at ≤ 0.45 m/s. Decide whether to teach
      the designer the limit or to fit real BlueBoat hydrodynamics (below).
- [ ] **The offline harness models a third system.** `docs/controllers/sim.py:31-32` uses the
      MPC's *real-boat* constants as its **plant**, so every C-series number in
      `CONTROLLERS.md` describes neither the Gazebo boat nor a measured BlueBoat. That is
      still self-consistent as a real-boat harness — which is why it was left numerically
      untouched — but it cannot be used to predict simulator behaviour. A `--plant gazebo`
      mode would close the gap.
- [ ] **No BlueBoat hydrodynamic identification exists anywhere in the project.** The xacro
      is BlueROV2's (`README.md` says so outright) and the MPC's real-boat coefficients have
      unknown provenance (`CONTROLLERS.md` §6.4). Every controller in the stack is tuned
      against one or the other. A bollard-pull plus a straight-line decay test on the water
      would settle it and is cheap compared with what rests on it.

## 0.3 MPC on `fsin` — orbit limit cycle (analysis UNSUPPORTED since 2026-09-04)

> **The observation behind this section was §0.4, not the cost function.** The 2026-09-01 run
> that prompted it carries 181 distinct commands over 705 ticks with a **517-tick
> bit-identical run**, and **518** `ACADOS solver failed with status 4` lines in its
> `launch.log`. The boat was driving a circle because the solver had stopped solving and the
> stale command was being republished — not because tracking the rotating tangent was cheaper
> than closing position. The analysis below is still internally sound but has **no observation
> supporting it**. Re-fly `fsin` under MPC now that §0.4 is fixed **before** applying any
> experiment in this section; the first three may turn out to be solutions to a problem that
> no longer exists.

Observed in Gazebo through the MCS map: under MPC on `fsin` the boat locks into a perfect
circle near the start while the reference runs away (robot↔target distance oscillating at the
loop period, growing). The full mechanism is `CONTROLLERS.md` C10 — briefly: `fsin`'s heading
swing is 364.75° per half-cycle so the reference is a chain of near-closed loops; at
`_FSIN_V = 0.5` m/s the 1.5 m loops sit at/past the 1.89 m minimum turning radius; once
displaced, `Q_ψ = 30` against `Q_pos = 50` makes tracking the rotating tangent cheaper than
closing position; governor F5 (`e_along` on a revolving tangent averages 0, `gov_Emax = 0`)
lets `tau` run away; SQP_RTI with no stage-1..N re-seeding never escapes the basin.
`LoS`/`PID` steer *toward* the target by construction and recover.

Sim-only experiments to make MPC track `fsin` — none applied yet, real-boat values untouched:

- [ ] **Rebalance the sim `mpc_Q_diag`** so position outweighs heading at loop scale
      (`Q_ψ` 30 → ~5–10, simulation default only), and re-measure on `fsin` and on the
      4 m circle so the C9/0.2 gains are not regressed.
- [ ] **Enable the cross-track governor brake per run in sim** (`gov_Emax` is a declared,
      launch-settable parameter) for looping shapes — this is F5's missing half. Keep the
      committed default 0.0; it is off because it is unsafe at the current inner gains.
- [~] **Warm-start hygiene in `ur_mpc.solve`.** *Half done (§0.4).* Stages 1..N are now
      re-seeded, but **only on the failure path**: `reset()` + a cold seed at the measured
      state + one retry, which is what stopped a bad step latching for 3291 ticks. The
      *per-tick* RTI shift (seed stage `i` from the previous iterate's stage `i+1` on every
      call, not just after a failure) is still not implemented, and is the remaining half.
- [ ] **Decide the `_FSIN_V` divergence.** `_FSIN_V` 0.1 → 0.5 in `path_generation.py`
      (committed in `e6dff70`, not uncommitted as this bullet used to say) contradicts the in-branch comment,
      `TRAJECTORY_SYSTEM.md` (§shape table and revision record), `CLAUDE.md`'s speed list,
      and the oracle: `docs/controllers/check_trajectory_library.py` currently **FAILS**
      3 checks because of it (`fsin` reference poses wrong at every sampled t, both
      Euler-loop comparisons; the 4th failure, `sin` at t = 500 only, is the known one-ULP
      scipy quaternion sensitivity — x/y reproduce the oracle exactly, verified 2026-09-01).
      Until the constant and the oracle/docs agree one way or the other, that gate is red
      and `fsin` field-data comparability is broken. (Deliberately not resolved here — the
      constant is the current experiment's setting.)

## 0.4 MPC QP failure published as a command — fixed 2026-09-04

**Symptom.** Under `controller_type:='MPC'` the simulated boat drove a constant-radius circle
forever, "no matter what" the mission was.

**Cause, in two halves.**

1. *The solver was failing.* `e6dff70` set `mpc_horizon = 30` for simulation (from 15).
   `FULL_CONDENSING_QPOASES` is a dense **active-set** solver, so the condensed QP carries
   `nv = N·nu = 60` variables and `2·nv` bound constraints — the only constraints this OCP has
   besides the initial state — while acados defaults `qp_solver_iter_max` to **50**. Reaching
   a vertex where most of those bounds are active costs about one working-set change per
   active bound, so at `N = 30` the budget is *below* the number of variables and a saturating
   solve fails by construction. At `N = 15` (30 against 50) it was never binding, which is why
   the real boat and every pre-2026-09-01 simulation run were unaffected.
2. *One failure poisoned every later solve — the bigger of the two.* `SQP_RTI` takes **one**
   Newton step per call from the previous iterate, and `solve` re-seeded only stage 0 and the
   `yref`s, never stages 1..N. A single bad step left the linearisation point corrupted and
   the controller never came back. Measured in Gazebo on `circle` at `spawn_yaw = π`: the
   boat was tracking at **0.05 m and 0.10 rad** of error when one tick commanded `[-20, -20]`
   and the QP then failed **3291 consecutive times** — 93.7 % of the run. The heading-error
   correlation below describes *when the first bad step arrives*, not why it lasts; raising
   the working-set budget does nothing for a poisoned iterate.
3. *The failure was published as a command.* acados leaves the primal iterate untouched on a
   non-zero status, so `ur_mpc.solve`'s `self.solver.get(0, 'u')` returned the last
   **successful** solve's command and `master_control` published it verbatim. A constant
   asymmetric thruster pair is a constant-radius circle.

**Forensics.** Distinct `(u1, u2)` pairs and the longest run of bit-identical consecutive
commands in `~/ros2_ws/data/MPC_data/*.npy`, against the `"ACADOS solver failed with status 4"`
count in the matching `~/.ros/log/*/launch.log`:

| run | ticks | distinct `u` | longest frozen | `status 4` lines |
|---|---|---|---|---|
| `2026_09_04-09_55_44` | 656 | **3** | 654 — 32.7 s of one command | *stdout not captured* |
| `2026_09_03-18_50_36` | 340 | **4** | 337 | 339 |
| `2026_09_01-10_50_57` | 705 | 181 | 517 | **518** |
| `2026_08_31-22_40_02` | 2501 | 1042 | 1456 | 1402 |
| healthy, pre-`e6dff70` | 1494 | 1494 | **1** | 0 |

The discriminator across all 34 recorded MPC runs is the **heading error**: every run reaching
`|ψ_err| ≥ 2.2 rad` froze on 58–100 % of ticks, every run under 0.95 rad was clean — same
trajectory, same compiled solver. A large heading error drives full differential across the
whole horizon, which is exactly the all-bounds-active vertex. It became routine when the MCS
started passing a random `spawn_yaw` for GPS-anchored simulated missions (`f37d43a`), and
those missions begin with the reference ~28 m away, which freezes `tau` and hands the OCP a
static, unreachable target.

**Why it went unseen.** The diagnostic was a bare `print()` — it never reaches `/rosout` — and
neither `Sim_launch.py` nor the simulator's `full_mission_launch.py` captures this node's
stdout. The Sep-4 launch logs carry only "process started"/"process has finished cleanly" for
`master_control`.

**Fixed.** No control law, model or gain changed.

- [x] `qp_solver_iter_max` is set on the OCP from a new `mpc_qp_iter_max` parameter, default
      `0` = derive as `max(50, 4·nu·N)` (240 at N = 30, 120 at N = 15).
- [x] A non-zero status triggers `solver.reset(reset_qp_solver_mem=1)`, a cold re-seed of
      every stage at the measured state with zero input, a re-application of the reference,
      and **one retry**. Failure handling, not a control law: nothing of it runs on the
      success path.
- [x] `ur_mpc.solve` and `uvr_mpc.solve` return **zeros** if the retry also fails, never the
      stale iterate, and record `last_status` / `fail_count` / `total_failures` /
      `recoveries` / `last_solve_time`.
- [x] `master_control`'s MPC branch commands zero thrust and logs `MPC solve FAILED` through
      `get_logger().error` at 1 s throttle. Deliberately **not** an early return, so
      `/thruster_input` stays alive and the `.npy` keeps recording.
- [x] `docs/controllers/check_mpc_solver.py` is the gate. On a 28 m static reference at π
      heading error: budget 50 with no recovery fails **396/400** ticks (status 4); budget 50
      *with* the recovery hits the cap 9/400 times but the caller sees **0** failures; the
      shipped budget of 240 hits it **0/1800** and closes 28 m → **0.26 m**. `396 → 9` from
      the reset alone is the measure of the latch.
- [x] Verified end to end in Gazebo (`circle`, `spawn_yaw = π`, 149 s of control): **3**
      isolated failures, each "1 consecutive", **0.03 %** zero-thrust ticks (was 93.7 %),
      longest bit-identical command **1 tick**, median track error **0.054 m**, +1.90 turns
      over **126 m travelled** — following the circle, not spinning on it.

**Open.**

- [ ] **Re-fly `fsin` under MPC and settle C10** (§0.3). Its observation was this defect.
- [ ] **Time the solve on the companion computer** before trusting `mpc_horizon = 30` anywhere
      near the real boat (§2 — still never measured on target hardware). `master_control` now
      warns through `/rosout` when a solve exceeds half the tick, so this is measurable
      without a field harness.
- [ ] **Decide whether the real boat should also move to `mpc_time = 6.0` / `mpc_horizon = 30`**
      once timed. If it does, `mpc_qp_iter_max = 0` covers the budget automatically — but C9
      (a horizon shorter than one turning radius) is the reason to want it.
- [ ] **`uvr_mpc.py` still carries the pre-C2 pairwise `np.unwrap`.** Marked in the source; not
      fixed alongside C6 because mixing an unrelated reference change into this diff would make
      it unreviewable. That node is launched by nothing.
- [ ] **The acados code-reuse hash was verified, not assumed.** `is_code_reuse_possible`
      compares an md5 of the whole `ocp.to_dict()`, and the serialized `model.f_expl_expr` in
      `acados_ocp.json` carries `a_*`/`d_*` as literals — so a coefficient-only retune does
      force a rebuild, and `build_solver(generate=False, build=False)` is safe as written.
      Verified 2026-09-04 against `acados_template` 0.5.1. Re-verify on an acados upgrade.

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
      it automatically. **Now measurable in simulation** (the acados environment is fixed,
      see §0.1): an isolated `Sim_launch.py controller_type:='MPC' trajectory:='kin_square'`
      run held 1391 control ticks over ~70 s with zero `/thruster_input` watchdog trips, so
      the loop is keeping its 20 Hz on this machine. Instrument the solve call itself and
      report the distribution rather than the absence of trips, then repeat on the boat's
      companion computer.
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
- [ ] **`stopping_sequence` latches — pinger branch only, since 2026-09-03.** `solve_LoS` sets
      it on arrival and only `manual_target_callback` clears it, so a latch armed by the
      **pinger** stays armed until someone publishes a manual target. Inert at the shipped
      default `safety_distance = -1.0`, which no launch file overrides — but it is a declared
      parameter, and both `CONTROLLERS.md` §7 (row 5) and finding **C3** recommend setting it
      to 1.5 m, so the recommendation arms the latch. `pinger_callback` should clear it the way
      the manual callback does. Surfaces on the basestation as a boat that stops responding
      with no indication why; the station must not compensate, see
      `BlueBoat-MCS/.claude/specs/robot-side-limitations-watchlist.SPEC.md`.
      **The manual half is closed:** a reached manual target is now held rather than abandoned
      (`manual_keep_location`), and that path no longer consults `safety_distance` or
      `stopping_sequence` at all.
- [ ] **`_FSIN_V = 0.5` — `fsin` runs at 5× its authored speed.** Root cause of the three
      `check_trajectory_library.py` failures measured 2026-09-03 (identical before and after
      the keep-location work, so not a regression from it). `path_generation.py`'s module-scope
      `_FSIN_V` is `0.5`, while the comment beside its own `radius = 1.5`, `TRAJECTORY_SYSTEM.md`
      §3 and the check's pinned reference table all say **0.1 m/s**. Measured against the pinned
      table the ratio is exactly 5.000 at `t = 1`. Consequences: the weave cycles every 60 s
      rather than 300 s, the yaw-rate amplitude is 0.333 rad/s (19 °/s) instead of 0.067, and
      `U_d = 0.5` reaches every controller's feedforward. The geometry is unchanged — the same
      1.5 m turn radius, traversed 5× faster. Fixing it changes a reference that existing field
      data was recorded against, so it is a deliberate re-measurement (and `CONTROLLERS.md`
      §4.4's shipped-gain claims about `fsin` move with it), not an edit. `CLAUDE.md` §3
      previously recorded all five checks as passing, which was stale.
- [ ] **The harness's Point-LoS simulation column uses a `k_psi` the node does not ship.**
      `run_sims.py` scenario G and `CONTROLLERS.md` §5.5 both label 16.0 as the "simulation"
      point-following yaw gain; `master_control` declares `point_k_psi` at **60.0** in
      simulation (10.0 real). `sim.PointLoS` carried the same 16.0 as its default. So fig 8
      panel b, and the C8 divergence table under it, are measurements of a gain the boat does
      not run. Re-running scenario G at 60.0 changes a published figure and the C8 numbers, so
      it is a deliberate re-measurement, not an edit. The real column (10.0) is correct and is
      the one `check_manual_hold.py` uses.
- [ ] **`sim.PointLoS` has no `min_thrust` floor on its pursuit surge.** The node has floored
      that term since 2026-08-31, so the harness models a boat whose sub-2 N commands reach the
      water. Only the *hold* surge is floored in both (2026-09-03). Adding the floor to the
      pursuit path changes scenario G and fig 8, so it is a re-measurement rather than an edit.
      Until then, any Point-LoS figure understates how far out the real boat sits still.
- [ ] **What bounds the manual keep-location hold is the pursuit law, not its own gains.**
      The hold rejects a current up to `2 × manual_hold_umax` (16 N on the real column);
      past that the boat crosses `manual_reacquire_radius` and recovery depends on the pursuit
      law's turning authority, which is what the moment-arm item below is about. Measured: it
      still parks (2.73 m at 20 N) rather than running away, but that margin is not something
      the hold's own gains control.
- [ ] **`from_yaml` reloads with no continuity guard, and the loader accepts a truncated file.**
      `_maybe_reload_yaml` re-reads on any mtime change and nothing compares the new file's pose
      at the current `tau` against the old one, so a mid-mission rewrite steps the reference by
      the whole anchor delta on the next request. Worse, `YamlTrajectory` accepts any file whose
      `format` matches and whose `points` is a non-empty (N,4) array: a partially-written file
      parses, yields a short trajectory, clamps the reference to a false end — and `_yaml_mtime`
      is updated, so it sticks until the next write. The station writes with a plain `open()`,
      not an atomic rename. Either make the write atomic on the station side (cross-repo, CM-3)
      or have the loader require the declared `duration_s`/`length_m` to match the samples.
- [ ] **`compute_target` returns `poses[1]` while everything else uses `poses[0]`.**
      `custom_functions.compute_target` takes `path.poses[:2]` and returns the SECOND pose as
      the reference position, so on the `PID`/`LoS` branches `/monitoring_data[4:6]` is the pose
      at `tau + path_time`, while the MPC branch, `path_progress_errors` and the RViz arrow all
      use `poses[0]`. At the shipped 0.05 s window that is 2.5 cm and harmless, but it means
      `x_d`/`y_d` means something slightly different per controller, and
      `docs/controllers/replay.py:273-275` documents the opposite ("win[0]"). Pick one.
- [ ] **`compute_target` is handed the control period, not the window spacing.**
      `master_control` calls `cf.compute_target(self.controller_path, self.dt)` while the window's
      own spacing is `path_time / (path_steps - 1)`. They are equal by construction today for
      `PID`/`LoS` (both 0.05), so nothing is wrong now — but the coupling is unstated, and if a
      window of different spacing ever arrives the reported `u`, `v`, `r` scale by the ratio.
      `path_progress_errors` already derives `dtau` correctly; pass the same expression.
- [ ] **`solve_LoS` uses the moment arm the wrong way round.** It builds
      `[v + 0.295*yaw_rate, v - 0.295*yaw_rate]` (`master_control.py:716-717`), *multiplying*
      by the arm `r = 0.295`, where `ThrustAllocator` *divides* by it
      (`1/(2r) = 1.695`) — a factor of 5.75 between the two for the same nominal moment. On
      top of that `v` and `yaw_rate` are kinematic quantities (m/s, rad/s) written straight
      into a topic every consumer reads as Newtons, so the law's gains are only meaningful
      relative to themselves. Not changed with the 2026-08-31 dead-zone work, which
      deliberately touched only the surge term: correcting the arm rescales all steering
      authority in the pinger and manual-target branches at once and needs a dock test, not an
      edit. `point_k_psi` is a declared parameter, so a candidate value costs a launch
      argument.
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
- [x] **F4 — MPC reads 16 poses from a 15-pose window** *(fixed 2026-08-31:
      `path_steps = mpc_horizon + 1`, so the window spacing and the solver's `time/horizon`
      are now identically equal and no pose is duplicated)* — it padded by duplicating the last one,
      giving a zero-velocity terminal reference; separately the window spacing is 2.5/14 =
      0.1786 s while the solver divides by 2.5/15 = 0.1667 s, inflating every reference speed
      by 7.1 %. `path_steps = mpc_horizon + 1` fixes both. Measured effect on tracking is
      negligible — fix for correctness, not for accuracy. Line numbers moved with the
      2026-08-31 build-location rework; the padding is in `MPCController.solve`.
      *Explicitly ruled out* as the cause of the MPC startup hang (§0.1) — it is a
      reference-quality defect, not a startup one, and stays open on its own merits.
- [x] **C2 — MPC heading wrap-around never reconciled.** *Fixed 2026-08-31, and it was worse
      than described.* Two distinct faults in `MPCController.solve`: (a) the forward unwrap
      was **pairwise against a re-wrapped predecessor** (`psi_prev` was re-read from the pose
      each iteration), so it never accumulated and a window straddling ±π came out as e.g.
      `3.140, 3.143, −3.100` — a 2π cliff *inside* the horizon, which the equally-weighted
      terminal cost then chased; and (b) the reference branch was never reconciled with the
      independently-wrapped measured yaw. Now one `np.unwrap` over the whole pose list,
      followed by a rigid 2π shift onto the branch nearest `x_current[2]` — rigid so every
      difference along the horizon, and hence the `r` references, is preserved. Measured
      before the fix on a lawnmower whose return legs sit on −π: clean to t = 35 s, then the
      boat drove its yaw the **wrong way** at full differential ([−20, +20] N) on a −0.785 rad
      error and never recovered — 64.8 % of ticks saturated, 11.57 m mean error. After:
      1.4 % saturated, 0.06 m mean error. This, not the weights, was the dominant cause of
      the "MPC just saturates" report.
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
- [x] **Recorded thrust exceeds the ±20 N clamp — CAUSE FOUND, 2026-08-31.** The premise
      that "the current code should not be able to produce this" was wrong: `solve_LoS`
      (manual target, and pinger + LoS) built its `[right, left]` array by hand and went
      through **no allocator and no limit at all**, so it could emit 30–40 N — the yaw term
      alone reaches ±9.3 N on the real boat and the manual double-log surge reaches ~25 N at
      50 m. Nothing clipped it in simulation either (`simulation_interface` → `ROV.move`, both
      clip-free), and on the real boat `manualMove`'s per-side clip reshaped the wrench rather
      than rejecting it. All three now saturate uniformly through
      `thrust_limits.scale_to_limit` against one `thrust_limit` parameter, and
      `publish_thrust` returns the saturated vector so the `.npy` records what went on the
      wire. The 30.6 N sample is explained; no recording is needed to close it.
- [ ] **`replay.read_poslog_csv` cannot read any CSV this system produces.** It requires
      columns named `x`, `y` and hard-raises without them, and reads `psi`, `t`, `u1`, `u2`;
      the schema has always called these `relative_x`, `relative_y`, `relative_psi`,
      `right_thr_in`, `left_thr_in` plus seven date fields. This predates the 2026-08-31
      schema revision — it has never worked — and is why §5's "never run against a field bag"
      item could not have been closed even with a bag in hand. It is a ~6-line column map in
      `docs/controllers/replay.py`; the `.npy` reader is fine and is what
      `check_replay.py` exercises.

Nothing beyond these is justified by evidence yet. In particular, do not add lint/format/docs
pipeline scaffolding: no recurring need for it appears anywhere in this module's history.
