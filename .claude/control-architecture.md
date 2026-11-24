# BlueBoat-Control — control architecture and the thrust path

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 4. Control architecture

Branch priority: **arrow keys** → **manual target** → **path following** → **replanner
awaiting-hold** → **pinger** → nothing (zero thrust). MPC is unsupported in pinger mode. In
a session path and pinger following run only between `start` and `stop`. Evidence for
everything here: `blueboat_control/src/CLAUDE.md`; knobs: `FIELD_TUNING.md`.

- **The controller is built in `__init__`**, never on the first tick (an acados build there
  froze the single-threaded executor). `Controller node initiated` = construction;
  `Controller ready` = readiness. A session `configure` rebuilds through the same code
  (`_apply_controller_setup` + `_make_controller`): PID/LoS inline, **MPC on a worker
  thread** swapped in by `_poll_controller_build` (state `loading`, `start` refused), so the
  loop never stops.
- **Arrow keys override everything**: out through `publish_thrust` (this node stays the only
  `/thruster_input` producer), τ frozen, the logged target held. Older than `arrow_timeout`
  (0.5 s) → zero, still engaged. Released by `[]`, a manual target or a run `start`.
- **acados solver cache**: `$ROS_HOME/blueboat_control/mpc` (`ur_mpc.acados_build_dir`),
  never under `cf.data_root()`. Any change to model coefficients, horizon, weights,
  `thrust_limit` or `mpc_qp_iter_max` regenerates it once (~1 min, logs `generated and
  compiled`) — expected; do not delete the cache.
- **The MPC warm start follows the boat's heading** (`ur_mpc.MPCController._align_iterate`,
  2026-11-13, runs on the real boat too). SQP_RTI takes one step from the stored iterate: a
  cold iterate (all zeros) is seeded at the measured state, and when the measured yaw wraps
  at ±180° the iterate is moved by the same whole turn. Before, every crossing gave one tick
  of full differential thrust (±40 N), and a start heading west began half a turn off.
  Regression: `test/test_yaw_wrap.py`.
- **A failed acados solve is not a command (C6).** Non-zero status → reset, cold re-seed,
  one retry, else zeros; reported through the ROS logger (never `print`); no early return,
  so `/thruster_input` and the `.npy` keep flowing. `mpc_qp_iter_max` 0 derives
  `max(50, 4·nu·N)`.
- **Live path update.** A run is **fixed-mission** (`path_source=mission`) or
  **replanner-owned**. A replanner run holds its own plan (`path_update.PathPlan`, evaluated
  by `YamlTrajectory.from_points`), never calls `/path_request`, starts holding at the boat
  (keep-location law, `solve_LoS(keep_location=True)`), takes every path through
  `/blueboat/update_path` and composes each window locally (`_replanner_reference_step`) —
  governor and controllers unchanged. Acceptance, in order: state gate (`not_ready`,
  `no_run`, `mission_owned`, `operator_override`), request validity, change-point range,
  lead `c ≥ τ + max(path_time + 2·dt, gov_Lmax/U_d)`, continuity at c (0.20 m, 10°; heading
  skipped where the reference is stationary). Run boundaries clear the plan. Thresholds:
  `path_update_*` parameters. Handling ≤ 2.2 ms worst case (20 000 samples). N10, N11.
- **Reference generation** (N8): the governor's along/cross factors are clipped to `[0, 1]`
  and multiplied, so τ is monotonic and never faster than authored speed — nothing may scale
  `tau_dot` above unity. `fac_cross` is **off** (`gov_Emax = 0`): throttling on an error the
  inner loops cannot close is positive feedback. The request
  `linspace(tau, tau + path_time, path_steps)` is asynchronous (window 1–2 ticks stale), and
  τ advances only against a **fresh** window (`path_stale_timeout`). `path_time` /
  `path_steps` are **derived**, never declared: PID/LoS `control_dt` / 2, MPC `mpc_time` /
  `mpc_horizon + 1` (6.0 s / 31 sim, 2.5 s / 16 real).
- **Which law runs where:** manual target → `solve_LoS` (every `controller_type`), then
  `manual_keep_location` once reached; path following → MPC `ur_mpc.MPCController.solve`,
  PID `PIDLoS.compute` (with `u_ff`, `psi_path`), LoS `los_guidance`; pinger → PID
  `PIDLoS.compute(state, target)` body-frame with `psi_path=None`, LoS `solve_LoS`.
  `solve_LoS` is a crude proportional pursuit law, not the path LoS, and works as-is.
  `PIDLoS` is always built with `lookahead = pid_lookahead = 2.5 m`.
- **Station-keeping hold** (zero authored speed): gate `w = 1 − U_d/hold_speed` plus
  `hold_radius`. `w` is exactly zero on every library shape (slowest: `sin` at 0.280 m/s),
  so raising `hold_speed` above 0.28 would alter path following.
- **Tuning knobs** are all declared unconditionally in `_declare_tuning_parameters` and read
  once at construction; gain triples are double arrays. Several defaults are **split
  real / sim-tuned**, chosen by `gain_profile` (`''` or `field` = the real boat's column, on
  the boat and, since 2026-11-13, in simulation; `sim` = the legacy column, simulation only)
  — no longer by `simulation`, which now decides only the log name and `/pose_arrow`. The
  split covers (PID gains, point gains, LoS speed scale, MPC horizon/time/R and plant
  model, `manual_hold_kx` — N/m). Also: `path_request_timeout` / `path_stale_timeout`
  (1.0 s), `thruster_input_timeout` (0.5 s, both interface nodes), `param_sequence_timeout_s`
  (20 s, `param_set`).
- **`time_source`** (`master_control`, `simulation_interface`; default `wall`, additive,
  owner order 2026-11-07): `ros` = the node clock for every world-timing use (`src/CLAUDE.md`
  parameter table). `Sim_launch.py` and SSS-Sim's `full_mission_launch.py` pass `ros` in every
  simulation, so a run slowed below real time (Gazebo real-time factor < 1, for the ray-cast
  sonar) keeps one clock: at 0.1 the wall-clock 0.5 s thrust watchdog tripped between
  `master_control`'s sim-time 20 Hz commands, and the `.npy` `t` grew 10× per sim second.

---

## 5. Thrust path — sharp edges

Evidence and tables: `blueboat_control/src/CLAUDE.md` §5.

- `/thruster_input` is **`[right, left]`** on every code path (allocation matrix, URDF
  thruster placement, `ROV.read_model`'s alphabetical joints, `simulation_interface`,
  `solve_LoS`, `manualMove`, the CLI's `move <left> <right> <s>`).
- `left_pwm = 3000 − pwm`: the left thruster is reversed for an asymmetric propeller —
  intentional.
- `manualMove`'s `compensation_gain` conditional (1.2 / 0.75) is overridden by a hard-coded
  `1.0`, and keys on `input[1]` while scaling `input[0]`. Dead — do not tidy without
  deciding what it should do.
- Thrust→PWM is a `PchipInterpolator` on a measured bollard-pull table
  (`cf.generate_interpolator`): about −27.6 N to +55.2 N; PWM clamps to `[1100, 1900]`.
- **Saturation is uniform, and one number**: every exit (`ThrustAllocator.allocate`,
  `publish_thrust`, `manualMove`, `simulation_interface`) scales the pair by one factor to
  `thrust_limit` (20.0 N; `thrust_limits.scale_to_limit`). Never per-side clip (it rewrites
  the wrench: 27 N of yaw collapsed to 2 N). `publish_thrust` returns the saturated vector,
  so monitoring, `.npy` and CSV agree with the wire.
- **0–2 N does not turn the propellers** (T200 ESC deadband). Only `solve_LoS`'s real-boat
  pinger surge stays under 2 N beyond 2 m, so only it is floored:
  `min_thrust · g · max(0, cos(bearing))` (both factors only reduce it; the yaw
  differential is untouched; `min_thrust = 0.0` restores the law). The boat parks at
  ~0.81 m by design.
- **A reached manual target is held**: `manual_keep_location` re-evaluates every tick (on
  station inside `manual_hold_radius`, pursuit beyond `manual_reacquire_radius`,
  proportional capped floored surge between); yaw untouched; `manual_hold_radius <= 0`
  restores the old latch bit-identically.
- **Loss of reference zeroes thrust at both ends**: `master_control` publishes `[0, 0]` on
  every early return; both interface nodes zero a command older than
  `thruster_input_timeout` (0.5 s). The watchdog does not disarm (`full_stop` stays the
  operator's), releases when commands resume, and on the real boat goes through the
  unforced `manualMove` (behind the N4 gate). The `controller_type == ''` `move` timeout
  owns that case.
- **`param_set`**: `override` maps SERVO1/3 to passthrough (51/53) and `SYSID_MYGCS` /
  `MAV_GCS_SYSID` (resolved at run time) to mavros (1); `default` restores 74/73 and 255.
  Every write is read back before `param_ready`. It never blocks forever and always reports
  (three rules in its module docstring). Because `param_mode` is heartbeated, a repeat is
  **not** an acknowledgement: MCS safe-shutdown requires a transition into `default`, and
  `robot_interface.mode_callback` / `param_callback` are edge-triggered.
- **Readiness handshakes survive discovery races**: both interface nodes re-publish
  `/blueboat/controller_ready` every 1.0 s; `robot_interface` re-requests the mode every
  second until `param_set` confirms (handled idempotently). Keep the pairing.

---
