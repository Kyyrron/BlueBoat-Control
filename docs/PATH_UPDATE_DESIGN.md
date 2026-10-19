# Live path update — design

Lets another node, the future **BlueBoat-Replanner**, do two things:
- replace the rest of the followed path **during a run**;
- load the **first** path of a run that starts with the boat holding position.

It behaves identically in simulation and on the real boat, adds only, and leaves every
fixed-mission run as it was.

- The contract a client codes against: [PATH_UPDATE_INTERFACE.md](PATH_UPDATE_INTERFACE.md).
- Sources: [REFERENCES.md](REFERENCES.md) (pointers Rn / Pn below).
- Code:
  - `blueboat_control/src/_custom_libraries/path_update.py`, the ROS-free core: every
    decision;
  - `master_control.py` section 7, the wiring;
  - `_custom_libraries/path_update_log.py`, the run's pathlog.

## 0. What the design starts from (verified in the code, 2026-10-15)

- **The path is a function of τ, owned by another node.** `path_generation` serves
  `/path_request`: parameter values in, one pose each out (`frame_id 'world'` =
  `/blueboat/odom` local ENU). A path is either a library shape or a `from_yaml` file in the
  `blueboat_trajectory/1` format (`yaml_trajectory.YamlTrajectory`: linear in t, yaw the
  short way, clamped).
- **`master_control` never holds the path.** It holds only the **window**
  `linspace(τ, τ+path_time, path_steps)`:
  - PID/LoS: 2 poses over 0.05 s;
  - MPC: N+1 poses over `mpc_time`, i.e. 31 over 6.0 s in simulation and 16 over 2.5 s real.

  It requests the window asynchronously, so the window in use is 1–2 ticks old.
- **Progress is τ** (`advance_governor`).
  - τ advances at most at authored speed, slows on the along-track gap, freezes at
    `gov_Lmax`, and never decreases.
  - It also freezes on a stale window, a manual target or arrow control.
- **How the window is used.**
  - The governor and LoS read the pose yaw as the path tangent γ_p
    (`path_progress_errors`, `los_guidance`).
  - PID steers at pose[1] (`cf.compute_target`).
  - MPC finite-differences the whole window and warm-starts SQP_RTI from its previous iterate
    (`ur_mpc.solve`; `src/CLAUDE.md` C6).
- **Run sessions exist only in a real-robot session launch.**
  - `run_status` is idle / loading / running / error.
  - Simulation has no session: `run_active` is always true.
  - The E-STOP and the motor gate live in `robot_interface`. `master_control` sees an E-STOP
    only as `controller_ready=False`, and simulation has neither.
- **The run CSV belongs to the interface nodes**, `robot_interface` and
  `simulation_interface`. `poslog_report.finalise_run` files an explicit list of files into
  `Robot_data/<stem>/`.

## D-1 Mechanism: one entry point, in `master_control`, local splice

**Where.** The entry point `/blueboat/update_path` is served by `master_control`. That is the
node that owns τ, the run state, the operator overrides and the controllers. One server
handles every request about the task it runs (R3 §III.B).

**Two kinds of run.** The kind is fixed per run (D-3):
- **Fixed mission** — `path_source=mission`, the default and today's behaviour: the run
  follows `path_generation`.
- **Replanner-owned** — `path_source=replanner`:
  - the run **never** calls `/path_request`;
  - its path is a **plan** held by `master_control`, an ordered list of segments
    `(c_k, samples_k, id_k)`;
  - each segment is evaluated by `YamlTrajectory` (`from_points`), exactly as a designer file
    with the same samples would be;
  - `plan(t)` is the last segment with `c_k ≤ t`, evaluated at t;
  - **accepting** `(c, samples)` drops every segment with `c_k ≥ c` and appends the new one;
  - segments wholly behind τ are pruned.

**Window composition.** Each tick, `_replanner_reference_step` composes the window at τ
locally:
- the same parameters, poses, quaternion and stamps (`path_stamp.encode`) that
  `path_generation` would return;
- `path_rx_time = now`, so the window is always fresh.

The existing governor and the PID/LoS/MPC branches then run **unchanged** on it. The window is
computed at the current τ instead of arriving 1–2 ticks late, and the governor measures
against the window the controller uses, as before.

Replacing a followed path while moving is established practice: Nav2 re-plans at 1 Hz while
its controller follows (R3 §IV.A).

## D-2 Progress = τ (no second notion)

τ is the governor's path parameter in **path seconds**, the `t` axis of the samples.

**What τ is in the literature.**
- It is the parameter of a virtual target whose rate is a control variable (R2 §II.B).
- It is an internal state of the controller (R1 Remark 1, eq. 6d).
- Its rate is a speed assignment reduced by the tracking error: R6 eq. (39). That is what
  `advance_governor` does.
- It is monotonic (R1 Problem 1(ii)).
- The geometric task (be on the path) is separate from the dynamic task (progress along it)
  (R5 §2, R6 §2).

The boat **trails** τ (PID 0.7–1.2 m, LoS 1.3–1.4 m, MPC slightly ahead; P2), so τ bounds the
boat's progress from above.

**Progress publishes τ itself.** It adds what follows from it and the plan:
- `following_id`;
- `active_id`;
- `pending_ids`;
- `t_end`;
- `end_reached`;
- `U_d`;
- `lead_min`;
- `min_change_point`.

Path seconds are not wall-clock seconds: when the boat lags, τ advances slower than time.

## D-3 States, gate, precedence

**The gate.** Evaluated at request time, in this order, on flags that both graphs share:

| Condition | Refusal | Why |
|---|---|---|
| `controller_ready` false | `not_ready` | E-STOP latched, or readiness not yet received |
| session with no run active | `no_run` | idle, loading, error |
| run follows the pinger or a fixed mission | `mission_owned` | |
| manual target or arrow control engaged | `operator_override` | |
| replanner run before its first odometry | `not_ready` | the hold pose is not captured yet |

**Precedence.**
1. **Safety and operator controls outrank updates.** These are E-STOP, stop, motor gate,
   manual target and arrows.
   - A request is refused while one of them is engaged.
   - An update never touches readiness, the motor gate, `run_active`, the manual target or
     the arrow state.
   - It can therefore not re-enable motors, restart a run, or hand control back to autonomy.
   - The motor gate stays authoritative downstream (`robot_interface.manualMove`). An update
     changes the reference, never the actuation path.
   - After an E-STOP release, or a manual-target resume sentinel, following continues on the
     same plan. That is today's behaviour for a fixed mission, caused by the operator's
     action and not by an update.
2. **Run boundaries outrank updates.** Launch, `start` and `stop` discard the plan and the
   accepted ids (`PathUpdater.reset`), and emit a `cleared` event.
3. **Ownership is fixed per run, so the two mechanisms never meet.**
   - In a replanner-owned run the plan is the only path source: `path_generation` is not
     consulted, so a station file rewrite or a GPS deploy cannot touch it.
   - A fixed-mission run refuses every request with `mission_owned`.
   - The mode is observable in progress (`path_source`, `state`), in `/blueboat/run_state`
     (`path_source`) and in the pathlog.

**Survey start (awaiting the first path).** A replanner run captures the **hold pose**
(boat x, y, ψ):
- in a session at `start`;
- outside one at the first tick with odometry.

It then holds there with the **manual-target keep-location law**:
- `solve_LoS(…, keep_location=True)`, then `manual_keep_location`;
- this is the field-used "hold where released" of the station;
- it is a new branch after path following and before the pinger branch;
- arrows and a manual target preempt it, and a release returns to the same hold pose.

The monitoring target, and so the poslog `target_*` columns, is the hold pose.

**Activation.** A segment becomes the followed one on the tick where τ ≥ c_k. This is the
along-track analogue of waypoint switching (R4 slides 37, 48). It emits an `activated` event
carrying the boat pose.

**End of path.** Past the last sample the plan clamps:
- U_d becomes 0, so the existing `hold_speed` gate holds station:
  - PID/LoS: the zero-authored-speed hold;
  - MPC: a stationary reference.

  This is exactly how a fixed `from_yaml` mission ends.
- `end_reached` is emitted once.
- τ keeps advancing, as today.
- A later update extends the path and clears `end_reached`. Its lead is then the floor only,
  because U_d = 0.
- Control never stops the run by itself.

## D-4 Acceptance checks

The checks run in order; the first failure answers. All thresholds are declared parameters.

### Step 1 — Request validity

The request must be fixed by the client if any of these fail.

| Check | Code |
|---|---|
| id matches `[A-Za-z0-9._:-]{1,64}`; it goes into a file name and an unquoted CSV | `bad_request` |
| id not already **accepted** in this run (a refused id may be reused) | `duplicate_id` |
| `frame == "world"` | `bad_request` |
| equal array lengths | `bad_request` |
| at least 2 samples | `too_few_points` |
| at most `path_update_max_points` = 20 000 samples | `too_many_points` |
| all values finite | `not_finite` |
| Δt ≥ 1 ms | `time_not_increasing` |
| 0 ≤ t < 1e6 (the `path_stamp` plausibility bound) | `time_out_of_range` |
| segment speed ≤ `path_update_max_speed` = 2.0 m/s | `too_fast` |
| yaw coherent with the motion, within `path_update_heading_coherence` = 45° | `heading_incoherent` |

Why the 20 000-sample cap: worst-case handling there is 2.2 ms, and it is 2.8 h of path at
0.5 s spacing (§ real-time safety).

Why 2.0 m/s: twice the §8.5 survey nominal (P1). Feasibility in maximum speed and turn rate is
the path designer's job (R4 slide 35; R2 §V.A). This bound only catches unit errors, e.g.
milliseconds for seconds.

**Yaw coherence**, in detail:
- **The test.** On every segment that moves (≥ 1 mm and ≥ `hold_speed`),
  min(|wrap(yaw_k − χ_k)|, |wrap(yaw_{k+1} − χ_k)|) ≤ 45°, where χ_k is the segment's course.
  Yaw may be the incoming or the outgoing tangent at a corner.
- **Why it is needed.** The governor (γ_p), PID (ψ_path) and MPC (ψ reference) all read yaw
  as the tangent.
- **What it catches.**
  - A compass heading in place of ENU yaw, which is caught except within ±45° of the two
    headings where both conventions coincide (NE / SW).
  - Degrees in place of radians, but **only when** the wrapped value misses by more than 45°.
    Example: 90 (deg) wraps to 2.04 rad, 27° from π/2, and passes.
- **Strictness.** The YAML loader checks none of this; the check is deliberately stricter.

### Step 2 — Change-point range (`change_point_outside`)

- First path: c is NaN or equals t[0].
- Otherwise c is required, with t[0] ≤ c < t[−1].

### Step 3 — Lead (`lead_too_short`; not applied to a first path)

```
c ≥ τ + lead_min,      lead_min = max( path_time + 2·dt ,  k · gov_Lmax / U_d )
```

- k = `path_update_lead_factor`, default 1.
- U_d is the plan's authored speed at τ over the window spacing, exactly as the governor
  computes it.
- The second term applies only while the reference moves (U_d ≥ `hold_speed`).

**Floor, `path_time + 2·dt`.** The controller is never handed a rewritten part of a window it
already holds, plus the tick on which the request is applied.
- For MPC that is the **whole horizon**. The OCP is posed on p(θ̄) over [t_k, t_k+T]
  (R1 eq. 6e), and SQP_RTI warm-starts from the iterate it computed for the old reference
  (`src/CLAUDE.md` C6).
- Beyond the floor, the composite path is, for every controller, indistinguishable from a
  path that was always that way.
- That is the precondition maneuvering theory states: *the path is specified a priori, only
  the speed assignment changes online* (R6 §3.2).

**Governor term, `gov_Lmax / U_d`.** This is the governor's catch-up distance, the gap at
which τ freezes, converted from metres to path seconds at the current authored speed (your
rule).
- It places the junction at least `gov_Lmax` = 3 m of path ahead of τ.
- The boat trails τ by at most that while τ advances; the field shows ≤ 1.4 m (P2).
- So the junction is always ahead of the boat, with the governor's whole range as margin.

Resulting `lead_min`:

| Controller | U_d = 1.0 m/s | U_d = 0.5 m/s | U_d = 0.28 m/s | stationary reference |
|---|---|---|---|---|
| PID / LoS | 3.0 s | 6.0 s | 10.7 s | 0.15 s |
| MPC, simulation (horizon 6.0 s) | 6.1 s | 6.1 s | 10.7 s | 6.1 s |
| MPC, real (horizon 2.5 s) | 3.0 s | 6.0 s | 10.7 s | 2.6 s |

`min_change_point = τ + lead_min` is published live in progress and in every answer.

### Step 4 — Continuity at c

The new path at c is compared with `plan(c)`, or with the hold pose for a first path.

**Position (`discontinuous_position`): ‖Δp‖ ≤ `path_update_pos_tol` = 0.20 m.**
- It is below the best field mean cross-track error, 0.24 m (P2), so a junction gap is under
  the tracking noise floor.
- It is below `gov_Lmin` = 0.5 m, so the governor stays in its full-speed regime.
- Through lookahead LoS, χ_d = π_h − atan(y_e/Δ) (R4 slide 46; R5 eq. 7), it steps the heading
  command by at most atan(0.20/2.5) = 4.6°. Δ = 2.5 m is `los_lookahead` = `pid_lookahead`.

**Heading (`discontinuous_heading`): |wrap(Δψ)| ≤ `path_update_heading_tol` = 10°.**
- γ_p enters ψ_d one-to-one (R4 slide 46).
- At `los_kpsi` = 10 a 10° step is a 1.75 N·m yaw-moment step: ±3.0 N per thruster
  (N/(2r), r = 0.295 m), far below the 20 N limit.
- The LoS and PID laws have no step filter. Fossen's `LOSchi` needs an observer for exactly
  these switching steps (R4 slide 48).
- Breivik & Fossen's controller assumes ψ_d, ψ̇_d, ψ̈_d bounded (R5 Prop. 5).
- **The heading check is skipped where the current reference is stationary at c.** That means
  U < `hold_speed` on both sides of c: past the end of the path, or the hold before a first
  path. There the boat turns in place, as R6 Ex. 2 traces corners by slowing to zero. The
  event says whether it was enforced (`heading_checked`).

**Curvature is not required continuous.**
- No law here feeds curvature forward. R2 Prop. 2 would need ∂c_c/∂s.
- A curvature step keeps the derivatives bounded (R6 Assumption 2.1).
- Dubins paths have yaw-rate jumps by construction (R4 slide 38).

**A refusal changes nothing.** The plan, τ, the accepted ids and the hold stay as they were;
only the counters and the last answer move. This is tested.

## D-5 First path

Accepted only while awaiting. Continuity is position vs the hold pose: the path starts at the
boat's start point, as θ(t₀) = argmin‖h(x₀) − p(θ)‖ (R1 eq. 8b). Heading is not checked,
because the reference is stationary. On acceptance:
1. τ := t[0];
2. plan = that segment;
3. the shared keep-location state is reset;
4. path following takes over on the next tick (`activated` event).

## D-6 Defined behaviour

Each case below is covered by `test/test_path_update.py` and/or the harness
(`test/mc_harness.py --replanner`).

| Situation | Behaviour |
|---|---|
| End of the active path | clamp, station-keeping hold, `end_reached` once, run continues |
| Station file rewrite / GPS deploy during a replanner run | no effect (`path_generation` not consulted) |
| Update during a turn | junction checked at the interpolated pose/tangent at c; accepted when continuous, refused on a kink |
| Update right after another | checked against the composite; may land before (supersedes) or after the earlier change point |
| Update while the boat lags (τ frozen) | lead measured from τ; unaffected |
| Update after the end | floor-only lead; heading free (stationary) |
| E-STOP latched | refused `not_ready`; plan kept |
| Manual target / arrows | refused `operator_override`; τ frozen as today; release resumes the same plan |
| Stop | plan cleared (`cleared`); idle requests refused `no_run` |
| New run | plan and ids cleared; new hold captured; awaiting |
| `configure` | may switch `path_source` between runs only (refused during a run, as before); `replanner` + `use_pinger` refused |
| Launch with `path_source:=replanner use_pinger:=true` | FATAL at construction |

## D-7 Traceability

**`/blueboat/path_update_event`** carries one JSON message per answer and per transition:
`request_accepted`, `request_refused`, `activated`, `end_reached`, `cleared`.

**The pathlog.** Every event becomes a row of the run's **`<date>-<note>-pathlog.csv`**:
- It sits beside the poslog and follows the poslog's rules: description row, unit row, column
  names, rows filled by column name, no commas.
- `robot_interface` writes it on the real boat and `simulation_interface` in simulation.
- `poslog_report.finalise_run` files it into `Robot_data/<stem>/`.
- The file is created **only** by the run's first request event; earlier `cleared` rows are
  held and written then. A run that received no request leaves no file, and a pinger run never
  gets one.
- The name does not contain `-poslog`, so the log reviewer's smoke-test filter ignores it.
- The file is never overwritten (opened with `'x'`).

**What else records the run.**
- The run bag records the event and progress topics; services are not bagged.
- The poslog's `target_x/target_y` columns record the reference actually followed, about
  3 Hz, so the log reviewer's target trail shows the junctions with no change.
- Exactly *which* path was active *when* comes from the pathlog's `activated` rows, with
  `ros_time` / `wall_time` / `run_time` and the boat pose.

**Known gaps.**
- `/set_path` (`path_publisher`, simulation only) still draws `path_generation`'s path.
- Bag replay draws the run-start `-path.yaml`, and a replanner run has none.
- Neither shows an updated path's geometry. That is in the event stream and the poslog
  target trail. See `REQUESTS.md` for an optional visualisation topic.

## D-8 Real-time safety

The service callback runs on `master_control`'s single-threaded executor, between control
ticks. Its cost, measured on this laptop (Ryzen 9 7940HS):

| Step | Measured cost |
|---|---|
| Typed request decode | 0.03 ms for 5 000 samples, 0.14 ms for 20 000. The JSON alternative would have cost 8.3 / 33 ms with the GIL held, which is why it was not chosen |
| `PathUpdater.handle`: validation, splice, continuity | worst 0.74 ms at 5 000 samples, 2.24 ms at 20 000 (cap), 9.2 ms at 100 000 |
| Window composition + tick, per control tick (31-pose MPC window) | median 0.13 ms |
| `/blueboat/path_progress` payload + JSON (10 Hz) | 0.005 ms in a fixed-mission run, 0.020 ms in a replanner run (median) |

**Closed loop.**
- **Offline harness** (`test/mc_harness.py --replanner`, Python GC disabled during timing).
  - Whole `timer_callback`, worst tick: 1.5 ms (PID), 1.6 ms (LoS), 3.0 ms (MPC, solve
    included).
  - Request handling, worst: 0.56 ms.
- **Gazebo** (`/blueboat/path_update_event.handling_ms`, real executor). Every request handled
  in under 1 ms.
- **Budget.** Both against a 50 ms tick.
- **GC spikes are the harness's, not the feature's.** With the GC left on, single 50–80 ms
  ticks appear. They come from the harness's own growing trace lists (a generation-2
  collection) and vanish with the GC disabled. The feature keeps no growing state: segments
  behind τ are pruned and events are not retained.

The swap itself is a list operation inside the callback, on the executor thread, so a tick
never sees a half-built plan.

## D-9 Changes to existing code (everything else is new code)

Each change is behaviour-neutral when the feature is unused. This is proven by
`test/mc_harness.py --compare HEAD`: 21 scenario runs bit-identical, see the report.

| File | Change | Why it is neutral |
|---|---|---|
| `master_control.timer_callback` | the `/path_request` block's `if` became `elif` after a new `path_source == 'replanner'` branch | branch false for `mission` |
| `master_control.timer_callback` | new awaiting-hold `elif` before the pinger branch | only true in a replanner run |
| `master_control.timer_callback` | monitoring condition `… or self._awaiting_hold_active()` | term false for `mission` |
| `master_control.solve_LoS` | optional `keep_location=None`; None derives the variant from `manual_target` exactly as before | every existing call passes None |
| `master_control._start_run` / `_stop_run` | `_path_update_reset()` call | clears an empty plan; no event in a fixed-mission run |
| `master_control._configure` | optional `path_source` field | absent = unchanged |
| `master_control._publish_run_state` | additive key `path_source` | consumers read keys by name |
| `robot_interface` | event subscription; pathlog open/close beside the CSV; `_snapshot_trajectory` returns early for replanner runs | additions only; the snapshot is unchanged for fixed missions |
| `simulation_interface` | event subscription; pathlog | additions only |
| `poslog_report.finalise_run` | one more name in the moved tuple, `pathlog_path()` | an absent file is skipped, like the sidecars |
| `yaml_trajectory.YamlTrajectory` | additive `from_points()` | `__init__` untouched; bit-identical poses tested |
| `Sim_launch.py`, `BlueBoat_launch.py` | `path_source` argument, default `mission` → `master_control` | default = previous behaviour |
| `blueboat_interfaces` | new `srv/UpdatePath.srv` | additive type; existing field lists unchanged |

## D-10 Limitations

- **The yaw-coherence check** catches compass-vs-ENU and most deg/rad errors, not all (D-4).
- **The Mission Control Station cannot yet start a replanner run.** It is frozen at v1.0 and
  passes neither the launch argument nor the `configure` field; see `REQUESTS.md`.
  Simulation launched by hand (`Sim_launch.py path_source:=replanner`) and a CLI session
  can.
- **The simulator's `full_mission_launch.py`** (BlueBoat-SSS-Sim, built worlds) does not pass
  `path_source`; see `REQUESTS.md`.
- **Tolerances and leads rest on single-site field data (P2)** and on simulation. The
  on-water protocol is in [PATH_UPDATE_REAL_WATER_TEST.md](PATH_UPDATE_REAL_WATER_TEST.md).
