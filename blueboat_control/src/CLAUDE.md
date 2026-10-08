# CLAUDE.md — `blueboat_control/src` (controller and thrust-path evidence)

Loaded when working under `blueboat_control/src/`. Moved verbatim from
`BlueBoat-Control/.claude/CLAUDE.md` §4–§5 on 2026-10-08: that file keeps the rules (and
N1–N9); this one keeps the measurements, derivations and tables behind them. Section
references (§n, Nn) point to that file.

## §4 Control architecture — detail

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

## §5 Thrust path — detail

- `/thruster_input` carries **`[right, left]`**. The convention is consistent across every
  code path: the allocation matrix `B = [[1,1],[0,0],[r,-r]]` with `radius = 0.59/2` puts a
  positive (CCW) yaw moment on column 0; the URDF places `thruster1` at `y = -0.295`
  (starboard) and `thruster2` at `y = +0.295` (port), and the yaw moment of a body-x force at
  `y` is `-y*Fx`, reproducing `+0.295 / -0.295` exactly; `ROV.read_model` sorts thruster
  joints alphabetically, so `forces[0]` drives `thruster1`; `simulation_interface` unpacks
  `r, l = thr_input`; `solve_LoS` builds `[v + 0.295*yaw_rate, v - 0.295*yaw_rate]`;
  `manualMove` treats `input[0]` as right; and the CLI's `move <left> <right> <s>` is stored
  as `[right, left]`.
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
