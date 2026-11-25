# blueboat_control/src — control architecture detail (§4)

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## §4 Control architecture — detail

**The controller object is built in `__init__`, not on the first tick.** Only `MPC` cares
(acados codegen + C compile): from `timer_callback` it froze the single-threaded executor for
the whole build — odom, `/path_request` futures and `publish_thrust` with it.

**Rebuilding between runs without stopping the loop.** A session `configure` may switch
`controller_type`, through the same construction steps as `__init__`:
`_apply_controller_setup(ctype)` (reference window, LoS allocator — cheap, executor thread) and `_make_controller(ctype)` (returns the object,
assigns nothing). PID and LoS are rebuilt inside the callback. MPC is submitted to a
one-thread `ThreadPoolExecutor`: the worker only *constructs* the solver (acados codegen and
compile release the GIL in the `make` subprocess), and `timer_callback` swaps it in on the
executor thread (`_poll_controller_build`) together with its setup, so the control loop —
manual target, arrow keys, `/thruster_input` — keeps its 20 Hz throughout. Measured in the
offline session harness: 27 `/thruster_input` messages during a cached MPC build (~1.3 s).
A failed build leaves the previous controller loaded and reports `state: error`, which keeps
`start` refused until the station reconfigures. At `start` the controller's memory is cleared
— `PIDLoS.reset()`, and for MPC `solver.reset()` (the reset the C6 failure path uses, i.e.
the state a freshly built solver starts from) — so nothing of the previous run leaks in.

**Arrow keys: why through this node, and why the dead-man stays engaged.** The station's
arrow thrust could have gone straight to `/thruster_input`, but two producers on that topic
fight tick by tick; routed here it is one more branch ahead of the manual target, and it
inherits `publish_thrust`'s uniform saturation and the "never silent" rule. A stream older
than `arrow_timeout` commands **zero and stays in arrow control**: falling back to the run's
path or pinger on a lost station would hand the boat to autonomous control nobody asked for.
Only an explicit `[]`, a manual target or a run `start` releases it. While engaged, `tau` is
frozen exactly as for a manual target and `/monitoring_data[4:6]` holds the last world target
registered (`last_world_target`, refreshed after every other branch), so the CSV's target
columns keep meaning "the reference the run was on".

**The generated acados solver is cached outside the working directory**, in
`$ROS_HOME/blueboat_control/mpc` (`ur_mpc.acados_build_dir()`): acados' relative defaults
put the generated C wherever the launch was invoked from (once inside the station's
repository), and a new directory meant a full recompile. `ur_mpc.build_solver()` asks acados
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
| `PID`, `LoS` | `control_dt` (0.05 s) | 2 |
| `MPC` | `mpc_time` (6.0 s sim / 2.5 s real) | `mpc_horizon + 1` (31 sim / 16 real) |

`N + 1`, not `N`: `MPCController.solve` reads N stages plus the terminal pose. Requesting N
made it pad by duplicating the last pose (a zero-velocity terminal reference) and inflated
every reference velocity by 7.1 % (window spacing `path_time/(N−1)` against the solver's
`time/N`).

**Guidance.** `PIDLoS` implements canonical Fossen lookahead LoS,
`psi_d = gamma_p + atan2(-e_y, Delta)`, with path-speed feedforward and optional
turn-slowdown. An invariant holds it compatible with point-following: when `psi_path is None`
the position error is projected onto the **boat heading** and `gamma_p` is taken from
`ref[2]`; when `psi_path` is supplied it is projected onto the **path tangent**.

`PIDLoS` is constructed with `lookahead = pid_lookahead = 2.5 m` (a required argument since
2026-11-25; the old `1.0` default and the `los_gain = 1/Delta` alias were never used).

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
| LoS guidance | `los_lookahead` (2.5), `los_ku` (30.0), `los_kpsi` (4.5), `los_kd` (1.0), `los_speed_scale` (1.0 real / 2.0 sim) |
| Station-keeping hold | `hold_speed` (0.05, the gate) and `hold_radius` (0.5), both shared by `PID` and `LoS`; `los_hold_kx` (1.0) and `los_hold_umax` (0.8), the LoS surge law only |
| PID | **Split simulation/real.** `pid_lookahead` (2.5 both); `outer_gains_x` (`[3.5, 0.01, 0]` real / `[8.0, 0.01, 0]` sim), `outer_gains_psi` (`[3.3, 0.01, 0]` / `[4.0, 0.01, 0]`), `inner_gains_u` (`[1.0, 0, 0]` / `[2.5, 0, 0]`), `inner_gains_r` (`[1.5, 0, 0]` / `[2.5, 0, 0]`) |
| MPC | **Split simulation/real, like the PID and point rows** — `mpc_horizon` (30 sim / 15 real), `mpc_time` (6.0 / 2.5), `mpc_R_diag` (0.10 / 0.015), `mpc_Q_diag` (50,50,30,1,1,1 both). The plant **model** is split too — `self.mpc_model`, not a declared parameter — with the simulation column fitted to `hydrodynamics.xacro` (added mass = the xacro values, damping = secant linearisations of its quadratic drag). Plus `mpc_qp_iter_max` (**0**, meaning "derive as `max(50, 4·nu·N)`" = 240 at N = 30, 120 at N = 15) — the qpOASES **working-set budget**, and not optional: `FULL_CONDENSING_QPOASES` is a dense active-set solver, so `nv = mpc_horizon · 2`, and acados' own default of 50 sits *below* the 60 variables of the simulation horizon. That made a saturating solve fail by construction, which is finding **C6** |
| Point following | `point_k_v` / `point_k_psi` (2.0 / 60.0 in simulation, 0.15 / 100.0 on the real boat), `safety_distance` (1.0 sim / −1.0 real, negative disables the arrival check — **pinger branch only** since the manual branch got its own hold) |
| Manual keep-location | `manual_hold_radius` (1.0 m, `<= 0` disables the whole hold), `manual_reacquire_radius` (1.5 m), `manual_hold_kx` (15.0 simulation / 8.0 real, **Newtons per metre**, not the m/s that `los_hold_kx` is), `manual_hold_umax` (defaults to `kx × (reacquire − hold)`, so retuning a radius cannot silently break the handover), `manual_brake_time` (1.0 s) |
| Thrust | `thrust_limit` (20.0 N) — feeds the allocator clamp, the MPC input bounds and `publish_thrust`'s uniform saturation. `robot_interface` and `simulation_interface` each declare a parameter of the same name and default (§5) |
| Arrow control | `arrow_timeout` (0.5 s) — dead-man on `/blueboat/arrow_thrust`: an older command is not applied (zero thrust, arrow control stays engaged). Session plumbing adds `session_mode` (False), not a tuning knob |
| Clock | `time_source` (`wall`, additive 2026-11-07; also on `simulation_interface`) — `ros` puts `current_time` (the `.npy` / `/monitoring_data` `t`, the pinger stop and manual-brake holds, the path-request / stale timeouts, the pathlog `run_time`) and the sim watchdog / ready republish on the node clock, so a simulation slowed below real time (Gazebo real-time factor < 1, the ray-cast sonar) behaves and records as at real speed. Every simulation launch passes `ros`; real launches keep `wall`. The arrow dead-man and the pathlog `wall_time` stay wall. Proof: `mc_harness --time-source ros` (wall vs ros bit-identical on one clock) |
| Dead zone | `min_thrust` (2.0 N) — the propeller-breakaway floor on `solve_LoS`'s surge term **and on the manual keep-location surge**. `0.0` disables both and restores the pre-2026-08-31 law exactly (§5) |

ROS 2 has no dict or tuple parameter type, so gain triples and the MPC weight diagonals are
declared as double arrays and reassembled in the node. `path_time` and `path_steps` stay
**derived** from `control_dt` / `mpc_time` / `mpc_horizon` and are deliberately not declared,
so the reference window and the solver's horizon cannot disagree.

`FIELD_TUNING.md` carries the field-measured symptom→knob table for most of these.
