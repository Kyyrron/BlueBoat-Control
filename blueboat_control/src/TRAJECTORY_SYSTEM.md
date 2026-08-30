# The BlueBoat Trajectory System — Complete Review

**Scope:** how a reference trajectory is defined, evaluated, advanced in time, and turned
into a target for the controller, in `blueboat_control`.

**See also:** [CONTROLLERS.md](CONTROLLERS.md) — what each controller does with that target,
compared side by side with simulated plots. Numbers quoted in this document come from that
same closed-loop harness — **simulation evidence, not field data**; see "How the numbers in
this document were produced" there.

**Files covered:**
[master_control.py](master_control.py) ·
[_custom_libraries/path_generation.py](_custom_libraries/path_generation.py) ·
[_custom_libraries/path_publisher.py](_custom_libraries/path_publisher.py) ·
[_custom_libraries/yaml_trajectory.py](_custom_libraries/yaml_trajectory.py) ·
[_custom_libraries/custom_functions.py](_custom_libraries/custom_functions.py) ·
[PID/PID.py](PID/PID.py) · [MPC/ur_mpc.py](MPC/ur_mpc.py)

---

## 1. The one-paragraph version

A trajectory is a **mathematical function of one number**: give it `t`, it gives you back a
pose `(x, y, yaw)`. Nothing more. A separate node, `path_generation`, owns that function and
serves it over a ROS service. The controller (`master_control`) keeps its own private
counter called **`tau`** (τ), asks `path_generation` "where should I be at τ, and at τ+a bit?",
and steers toward the answer.

The key design decision — and the thing most people get wrong when reading this code — is
that **τ is not the clock**. τ is a *progress dial* that the controller turns forward itself,
20 times a second, and **only as fast as the boat is actually keeping up**. If the boat falls
behind, τ slows down or stops entirely, and waits. That mechanism is called the **governor**.

> **Mental image:** imagine a friend walking a dog on a leash. The friend (the virtual target)
> walks the planned route. If the dog (the boat) lags too far behind, the friend slows down,
> and eventually stops and waits. The friend never runs off and never walks backwards.

---

## 2. The cast of characters

```
                         ┌──────────────────────────────┐
                         │      path_generation         │   "the map"
                         │  a pure function t -> pose   │   stateless, no memory
                         │  service: /path_request      │
                         └───────────┬──────────────────┘
                           ▲         │
           [t0, t1, ...]   │         │  nav_msgs/Path (list of poses)
                           │         ▼
   ┌───────────────────────┴──────────────────────────────┐
   │                   master_control                     │   "the driver"
   │   owns tau, runs the governor, runs the controller   │   20 Hz
   └───────────┬──────────────────────────────────────────┘
               │ /thruster_input  [right, left]
               ▼
   ┌──────────────────────────┐        ┌──────────────────────────┐
   │  robot_interface (real)  │   or   │ simulation_interface     │
   │  -> PWM over MAVROS      │        │ -> Gazebo thrusters      │
   │  publishes /blueboat/odom│        │ publishes /blueboat/odom │
   └──────────┬───────────────┘        └──────────┬───────────────┘
              └───────────── feedback ────────────┘
                              (x, y, yaw, u, v, r)

   ┌──────────────────────────┐
   │      path_publisher      │   "the map on the wall" — RViz only,
   │  re-asks for t=0..1000   │   not in the control loop at all
   │  republishes on /set_path│
   └──────────────────────────┘
```

| Node | Role | Rate | In the control loop? |
|---|---|---|---|
| `path_generation` | Evaluates the trajectory function | on demand | **yes** |
| `master_control` | Advances τ, computes thrust | 20 Hz | **yes** |
| `path_publisher` | Draws the whole path in RViz | re-requests every 5 s, republishes at 1 Hz | no |
| `robot_interface` / `simulation_interface` | Motors + odometry | ~20 Hz | yes |

---

## 3. Layer 1 — What a trajectory *is*

Everything lives in one function:
[`PathGeneration.single_pose(t, path_shape)`](_custom_libraries/path_generation.py).

It is a long `if`/`elif` chain. Give it `t = 12.0` and `path_shape = 'circle'`, it computes x,
y and yaw with a bit of trigonometry and returns a `PoseStamped`. It is **pure in `t`** — ask
for `t = 12.0` a thousand times, in any order, you get the same pose a thousand times. `fsin`
is the one shape that cannot be evaluated in closed form; it reads an integration table that
is built once and only ever extended, which is a cache, not state: what comes back for a given
`t` does not depend on what was asked for before it (§9, F1). A name that is not a shape
raises rather than falling through (§9, F9).

The service [`generate_path`](_custom_libraries/path_generation.py) is just a loop:
receive a list of `t` values, call `single_pose` on each, return them as a `nav_msgs/Path`.

```
request:  [10.00, 10.05]                 (a list of numbers)
response: Path{ pose@t=10.00, pose@t=10.05 }
```

### The built-in shapes

Selected at launch with `trajectory:=<name>`. **The speed of the boat is baked into the
formula** — there is no separate speed setting. `x = 0.5*t` *means* 0.5 m/s.

| `trajectory:=` | Shape | Authored speed | Starts at |
|---|---|---|---|
| `station_keeping` | Stay at the origin | 0 m/s | (0, 0), yaw 0 |
| `straight_line` | Line along +x | 0.5 m/s | (0, **1**), yaw 0 |
| `circle` | 4 m radius circle, centre (−4, 0) | 0.32 m/s | (0, 0), yaw **π/2** |
| `sin` | Sine weave along +x, amplitude 3.5 m | 0.28–0.56 m/s | (0.5, 0), yaw 0 |
| `fsin` | Oscillating heading, constant surge | 0.1 m/s | (0, 0), yaw 0 |
| `square` | Square *wave* — instantaneous ±4 m jumps | 0.5 m/s + ∞ spikes | (0, **2**), yaw 0 |
| `kin_square` | Zig-zag: +x, +y, +x, −y, 5 m legs | 0.3 m/s | (0, 0), yaw 0 |
| `seabed_scanning` | Scripted survey with arcs and a helix | 0.5 m/s | (0, 0), yaw 0 |
| `from_yaml:<path>` | Designer-generated file | whatever was authored | (0, 0), yaw 0 |

> ⚠️ **Start alignment matters.** `robot_interface` zeroes the world frame at the boat's
> position *and heading* when it boots
> ([robot_interface.py:675-682](robot_interaction/robot_interface.py#L675-L682)). So the
> trajectory always starts relative to wherever the boat was switched on. A trajectory that
> begins at (0, 2) or at yaw π/2 asks the boat to make an immediate correction manoeuvre.

> ⚠️ **These shapes are reference conditions for existing field data.** Every earlier field
> run was recorded against the formula as it stands here. Changing one invalidates comparison
> with those runs and **nothing raises an error** — the shape is not versioned in the code, the
> position CSV or the `.npy` log. Field data is write-once; it cannot be re-collected to match
> a changed formula.
>
> **Shape revision record** — append a row whenever a formula changes, naming the shape and the
> date, so a later comparison can be checked.
>
> | Date | Shape(s) | What changed | Prior runs comparable? |
> |---|---|---|---|
> | 2026-08-28 | — | Baseline: every shape is at its original formula. | — |
> | 2026-08-30 | `sin`, `kin_square` | **F7.** `t > 500` holds the last pose instead of teleporting back to the pose at t = 50. Below t = 500, bit-identical. | **Yes.** The path parameter advances at most 1.0 per second, and no run has come near τ = 500 (the longest harness scenario reaches τ ≈ 160), so the changed region was never exercised. |
> | 2026-08-30 | `fsin` | **F1.** Per-pose re-integration replaced by a cumulative table on the same 0.01 s grid. | **Yes.** Bit-identical to the original loop at every sampled t; `check_trajectory_library.py` asserts it against an embedded copy of that loop. |

---

## 4. Layer 2 — YAML trajectories (the Mission Designer path)

`from_yaml` replaces the maths with a lookup table.
[`yaml_trajectory.py`](_custom_libraries/yaml_trajectory.py) loads a file of dense samples:

```yaml
format: blueboat_trajectory/1
loop: false
points:                    # [ t (s), x (m), y (m), yaw (rad) ]
  - [0.0, 0.0,  0.0, 0.0]
  - [0.5, 0.25, 0.0, 0.0]
  ...
```

Evaluation is a binary search plus linear interpolation
([`YamlTrajectory.pose`](_custom_libraries/yaml_trajectory.py#L49)), with yaw interpolated
the short way around the circle. Two edge rules:

* **past the end** → clamps to the last sample (the boat stops there), unless `loop: true`,
  in which case `t` wraps modulo the duration;
* **before the start** → clamps to the first sample.

All the hard geometry (arcs, Béziers, splines, lawnmower patterns, per-segment speeds) is
resolved on the laptop at export time. The robot only ever does linear interpolation.

### The "file appears later" trick (GPS-anchored missions)

`path_generation` **watches** the YAML file
([`_maybe_reload_yaml`](_custom_libraries/path_generation.py#L220), called on every service
request). If the file doesn't exist yet, `single_pose` returns the origin — i.e. the boat
station-keeps where it started. Once the Mission Control Station has established the
odom↔GPS fit and writes the deployed file, the next path request picks it up (mtime change)
and the boat transitions onto the real-world path. Same mechanism handles editing a
trajectory mid-run.

---

## 5. Layer 3 — How the target moves: τ and the governor

This is the heart of the system. It lives in
[master_control.py:254-288](master_control.py#L254-L288).

### 5.1 What the old version did (and why it was replaced)

The header comment on [master_control.py](master_control.py#L3-L33) documents the previous
design: `t = time.time() - t0`. The reference advanced with **wall clock**, at **1 Hz**. If
the boat was slow, or turned the wrong way, or hit wind — the target kept going without it.
The boat chased a point that had already left, and the result was "smooth path-blind arcs
with no resemblance to the path."

### 5.2 What it does now

```
self.tau      # the progress dial, in "path seconds"
self.dt = 0.05    # 20 Hz control loop
```

Every tick, three things happen in order:

**Step 1 — measure the gap.**
[`path_progress_errors`](master_control.py#L254) takes the two poses currently in hand
(`poses[0]` = the target at τ, `poses[1]` = a little further along) and computes:

```
gamma_p  = heading of the path at the target       (its tangent)
e_along  = how far AHEAD the target is, measured along the path      [metres]
e_y      = how far SIDEWAYS the boat is from the path                [metres]
U_d      = the authored speed of the path right there                [m/s]
           = distance(pose0, pose1) / (tau spacing)
```

**Step 2 — turn the dial.** [`advance_governor`](master_control.py#L339):

```python
fac_along = clip((gov_Lmax - e_along)/(gov_Lmax - gov_Lmin), 0, 1)   # 3.0 - 0.5 = 2.5 m
fac_cross = clip((gov_Emax - |e_y|)  /(gov_Emax - gov_Emin), 0, 1)   # 1 when gov_Emax = 0
tau      += path_speed_scale * fac_along * fac_cross * dt
```

`fac_cross` is disabled by default (`gov_Emax = 0`); see F5 below.

`factor` is the throttle on the target's motion:

| Along-track gap `e_along` | `factor` | What the virtual target does |
|---|---|---|
| ≤ 0.5 m (boat is right on it) | **1.0** | moves at the full authored speed |
| 1.0 m | 0.8 | 80 % of authored speed |
| 1.75 m | 0.5 | half speed |
| 2.5 m | 0.2 | crawling |
| ≥ 3.0 m (boat far behind) | **0.0** | **frozen — waits for the boat** |

Two properties fall out of the `clip(..., 0, 1)`:

* **τ can never run backwards** (factor ≥ 0) — the mission never un-does progress;
* **τ can never exceed the authored speed** (factor ≤ 1) — even if the boat overshoots
  and gets *ahead* of the target, the target does not sprint to catch up.

**Step 3 — ask for the next window.** [master_control.py:687-692](master_control.py#L687-L692):

```python
request.path_request.data = np.linspace(tau, tau + path_time, path_steps)
self.future = self.client.call_async(request)      # asynchronous: never blocks the loop
```

The result is collected on a **later** tick, when `future.done()` is true
([master_control.py:668-678](master_control.py#L668-L678)). Meanwhile the controller keeps
using the previous window. So the reference is typically 1–2 ticks (50–100 ms) stale — a
deliberate trade to keep the 20 Hz loop from ever blocking on a service call.

### 5.3 The self-balancing behaviour (worked example)

Path authored at **0.5 m/s**, boat physically capable of only **0.4 m/s** (wind, fouling,
low battery):

```
t=0s    boat and target together      e_along = 0.0 m   factor 1.00   target 0.50 m/s
t=2s    boat losing ground            e_along = 0.2 m   factor 1.00   target 0.50 m/s
t=6s    gap growing                   e_along = 0.6 m   factor 0.96   target 0.48 m/s
t=15s   governor biting               e_along = 0.9 m   factor 0.84   target 0.42 m/s
t=30s   EQUILIBRIUM                   e_along = 1.0 m   factor 0.80   target 0.40 m/s
                                                       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                                       target speed now exactly matches boat speed,
                                       and the lag stays at a constant 1 metre.
```

The system finds its own steady state. **Consequence: the geometry of the mission is
deterministic, but its schedule is not.** A survey authored to take 200 s will take longer
if the boat is slow — which is exactly what you want, because every metre of the pattern
still gets covered.

---

## 6. Layer 4 — From target to thrust

The shape of the request depends on the controller, and that is the only thing
`controller_type` changes about the trajectory system:

| `controller_type` | `path_time` | `path_steps` | Window requested |
|---|---|---|---|
| `PID` | 0.05 s | 2 | `[τ, τ+0.05]` — just enough for a finite difference |
| `LoS` | 0.05 s | 2 | same |
| `MPC` | 2.5 s | 15 | `[τ, …, τ+2.5]` — a whole prediction horizon |

### PID and LoS — two poses are enough

[`cf.compute_target`](_custom_libraries/custom_functions.py#L77) turns the two poses into a
6-element target `[x, y, psi, u, v, r]`: position and heading from the *second* pose, and
velocities from the difference between them divided by `dt`.

Both then run the **canonical Fossen lookahead line-of-sight law**:

```
psi_d = gamma_p + atan2(-e_y, Delta)          Delta = 2.5 m lookahead
```

In words: *aim at a point on the path 2.5 m ahead of your closest approach.* Far from the
path, `atan2` saturates near ±90° and the boat cuts straight at it; close to the path, the
correction fades and the boat settles onto the tangent. Bigger `Delta` = gentler, more
damped; smaller = more aggressive, risks weaving.

* **`LoS`** ([`los_guidance`](master_control.py#L293)) is purely kinematic — proportional
  gains straight to a wrench `[X, 0, N]`, then `ThrustAllocator` splits it into two thrusters.
  Surge command is `U_d * max(0, cos(psi_err))`: **it slows down while turning hard**, which
  stops the boat from spiralling around a corner it cannot make.
* **`PID`** ([`PIDLoS.compute`](PID/PID.py#L134)) is a cascade: outer loop turns position
  error into speed/yaw-rate references, inner loop turns those into forces. It receives
  `u_ff = U_d` as a **feedforward**, so the PID only has to correct the *residual* speed
  error rather than build the whole command from scratch.

### MPC — a whole horizon

[`ur_mpc.MPCController.solve`](MPC/ur_mpc.py#L215) consumes the 15-pose window, converts it
into a full state reference `[x, y, psi, u, v, r]` per node by finite differences, and solves
an acados optimal-control problem that respects thruster bounds. Weights:
position 50, heading 30, velocities 1, control effort 0.015.

### The two overrides

Path following is not always in charge. Priority order in
[`timer_callback`](master_control.py#L698-L767):

1. **Manual target** (`/blueboat/manual_target`, from the visualisation app) — point LoS in
   the body frame. **τ is frozen while this is active** ([line 440](master_control.py#L682)),
   so when you release manual control the mission resumes exactly where it left off. Nice
   detail.
2. **Pinger** (`use_pinger:=True`) — chases acoustic coordinates; `path_generation` isn't
   even launched in that mode.
3. **Path following** — the subject of this document.

---

## 7. One complete tick, start to finish

```
  ┌── every 50 ms ────────────────────────────────────────────────────────┐
  │                                                                       │
  │  0. ready? initialised? odometry received?           else return      │
  │                                                                       │
  │  1. read boat state from /blueboat/odom                               │
  │        current_state = [x, y, psi, u, v, r]                           │
  │                                                                       │
  │  2. collect the pending /path_request result, if it finished          │
  │        -> self.controller_path  (the window of poses)                 │
  │                                                                       │
  │  3. measure e_along and e_y against poses[0]                          │
  │  4. GOVERNOR:  tau += path_speed_scale * factor(e_along, e_y) * dt    │
  │  5. fire the next /path_request at the new tau     (async)            │
  │                                                                       │
  │  6. compute thrust from the CURRENT window                            │
  │        LoS / PID / MPC   ->  u = [right, left]                        │
  │                                                                       │
  │  7. publish /thruster_input, log a row to /monitoring_data,           │
  │     save the .npy file every 0.1 s                                    │
  └───────────────────────────────────────────────────────────────────────┘
```

---

## 8. Design verdict

**The architecture is sound and the maths is correct.** Specifically, three things are
genuinely well done:

1. **Path-parameter control instead of clock control.** This is the right answer to the
   original problem, and the governor is a clean, minimal implementation of it: three lines
   of code, no tuning traps, provably monotonic and speed-bounded.
2. **The stateless-function trajectory model.** Because `single_pose(t)` is pure, the
   trajectory can be swapped, re-derived, replayed, or hot-reloaded from disk with zero
   coupling to the controller. It is also why the YAML feature could be bolted on without
   touching a single line of control code.
3. **Sign conventions are consistent.** I checked the cross-track error and lookahead law in
   all three places it appears ([master_control.py:274-275](master_control.py#L274-L275),
   [master_control.py:310-311](master_control.py#L310-L311),
   [PID.py:166-176](PID/PID.py#L166-L176)) — all three agree with each other and with the
   standard Fossen formulation. That is unusual and worth keeping.

The problems below are all *around* the core, not in it.

---

## 9. Review findings

### 🔴 Blocking  — *both closed*

**F1 — `fsin` will stall `path_generation` and, through it, the control loop.** *(closed)*
`fsin` re-integrated the trajectory from t=0 in a Python loop with a 0.01 s step **on every
single evaluation** — 30 000 iterations per pose at τ = 300 s, and ≈5×10⁸ for the 10 001-pose
whole-path request `path_publisher` makes, which appeared to hang the launch.

Closed form is not available: the heading is the integral of a sine and is analytic, but x
and y are integrals of the cosine and sine *of that heading* and are not. So the integration
is done once on the same fixed 0.01 s grid and read out by index
([`_fsin_state`](_custom_libraries/path_generation.py)). The table only grows, and each
extension continues the accumulation from its stored last value —
`cumsum([last, *increments])`, never `cumsum(increments) + last` — so the float sequence is
identical to a single pass and the function stays **pure in t**: the same t gives the same
pose whatever order poses are asked for.

Measured: bit-identical to the original loop at every sampled t, and the 10 001-pose window
evaluates in **0.14 s** against ≈15 min extrapolated from the old loop. `check_trajectory_library.py`
carries the original loop as its oracle and asserts both.

**F2 — neither path controller can station-keep, or hold the end of a finished mission.**
*(closed)*
`LoS`'s surge command was `u_cmd = los_speed_scale * U_d * max(0, cos(psi_err))`, identically
zero whenever the authored speed `U_d` is — `station_keeping`, a clamped-out YAML mission, or
the not-yet-deployed-file fallback — so the boat steered onto the line through the target and
then drifted off it with no restoring force.

**`PID` was not fine either**, contrary to what this document and `CONTROLLERS.md` both used to
say. Its outer `pid_x` loop does act on the along-track error whatever `u_ff` is, but it
projects onto the *path tangent*, and a stationary reference has no meaningful tangent: a pure
cross-track error produces no along-track error and therefore no surge. Measured, not reasoned:
against a 4 N lateral current it drifted to 11.9 m and against 10 N to 30.6 m, as far as the
broken `LoS`. Raising the inner gains to the C1 recommendation does not help — the limit is
structural, not authority.

Both are now gated by `hold_speed` (0.05 m/s) and `hold_radius` (0.5 m). `los_guidance` steers
at the reference point and commands `min(los_hold_umax, w * los_hold_kx * gap)` of surge for
the range `gap` outside the radius, never reverse — a lookahead law steers the wrong way
backwards, so the yaw channel turns the boat round instead. The `PID` branch instead rotates
the `psi_path` it hands `PIDLoS` toward the **bearing** to the hold point, fading in over
`hold_radius`, so the class's own along-track error becomes the range and its own steering
points at the point; `slow_on_turn`, the class's own option, stops it driving away while it
turns round. Neither is a new control law — both re-aim laws that already existed.

The blend weight `w = 1 - U_d/hold_speed` is **exactly zero** for every trajectory in the
library (all ≥ 0.28 m/s against the gate), so path following is unchanged to the last bit for
both controllers; `check_los_hold.py` asserts that against the real harness controllers rather
than assuming it. Measured at 150 s, distance from the hold point:

| case | LoS before | LoS after | PID before | PID after |
|---|---|---|---|---|
| displaced 2.2 m, no disturbance | 2.236 m | **0.499 m** | 1.433 m | **0.631 m** |
| 4 N lateral current | 12.052 m, growing | **0.700 m** | 11.893 m, growing | **0.898 m** |
| 10 N lateral current | 30.162 m, growing | **1.000 m** | 30.585 m, growing | **2.137 m** |
| started 3 m past the point | 3.000 m | **0.499 m** | 3.000 m | **0.462 m** |

Every "after" figure is steady (σ ≤ 0.05 m over the last 20 s) with zero thrust sign changes,
so the hold settles rather than hunting, and neither controller overshoots outward on the way
in.

### 🟠 Important

**F3 — `path_publisher` asks for the path exactly once, at construction.** *(closed)*
It made a single blocking request in `__init__` and then republished that frozen `Path` at
1 Hz forever, so for every GPS-anchored mission RViz showed a single dot at the origin for the
entire run: the deployed file did not exist when `path_publisher` started.

The request now repeats on a `refresh_period` (default 5 s, a declared parameter) and nothing
blocks in `__init__` — neither on service discovery nor on a response — so the node reaches
`spin()` even if `path_generation` is slow, starts later, or never starts. The last good path
keeps publishing while a request is in flight, so the display never blanks, and a response
that never arrives is dropped and retried rather than wedging the node. `/set_path` keeps its
name, type and QoS (N1).

Verified end to end: with the publisher started **first** and no service running, it reaches
spin and publishes an empty path; when `path_generation` appears it picks up the origin hold;
when the mission file is written it shows the real mission within one refresh; when the file is
edited mid-run it follows.

**F4 — MPC receives 15 poses but needs 16, at the wrong spacing.**
`path_steps = 15`, `mpc_horizon = 15`, but `solve()` reads `poses[:N+1]` = 16
([ur_mpc.py:216-218](MPC/ur_mpc.py#L216-L218)) and pads by duplicating the last pose — so the
terminal reference always has **zero velocity**, telling the MPC to brake at the end of every
horizon. Separately, the window spacing is `2.5/14 = 0.1786 s` while the MPC divides by
`self.dt = 2.5/15 = 0.1667 s` ([ur_mpc.py:156](MPC/ur_mpc.py#L156)), so every reference speed
is **7.1 % too high**.
*Fix:* `self.path_steps = self.mpc_horizon + 1` — with `path_time` left at `mpc_time`, that
single change makes the spacing `2.5/15` exactly, correcting both problems at once.

*Measured impact:* small. Making this fix changes circle-tracking RMS error from 1.027 m to
1.048 m — i.e. not at all. It is a real bug and worth fixing for correctness, but the metre of
error it was suspected of causing turns out to come from the MPC's too-short prediction
horizon instead (finding **C9** in [CONTROLLERS.md](CONTROLLERS.md), which has the evidence).
Fix F4, but do not expect it to buy accuracy on its own.

**F5 — The governor's cross-track term is built, and disabled.**
A boat perfectly abreast of its target but **20 m off to the side** sees `e_along ≈ 0`, so on
the along-track factor alone the governor runs at full authored speed and the target walks the
whole mission while the boat is nowhere near the path. That was the one case where the
reference could still "escape".

`advance_governor` now takes `e_y` and multiplies in a second unit-bounded factor
`clip((gov_Emax - |e_y|)/(gov_Emax - gov_Emin), 0, 1)`. The multiplicative form was chosen
over `hypot(e_along, e_y)` because it is exactly 1 when `e_y = 0`, so it cannot perturb the
along-track behaviour, and because a product of two `[0,1]` factors is still in `[0,1]` — τ
stays monotonic and stays bounded by the authored speed.

**`gov_Emax = 0` disables it, and that is the default.** The term is only safe once the inner
loops can close a lateral gap. At the current gains they cannot, and throttling τ on an error
the controller cannot reduce is positive feedback: the target stalls, the boat loses the
forward authority it converges laterally with, and the offset grows. Measured at the current
gains, acquisition RMS goes 0.661 → 3.508 m and progress 0.43× → 0.11×. At `u = 5 / r = 30`
the same term is neutral-to-better everywhere. See `TODO.md` C1 and F5.

**F6 — `square` is not physically followable.**
[path_generation.py:351-359](_custom_libraries/path_generation.py#L351-L359) flips `y`
between +2 and −2 with `math.floor` — an **instantaneous 4 m teleport**. When that
discontinuity falls inside the 0.05 s window, `compute_target` reports a desired speed of
`4.0 / 0.05 = 80 m/s` and a 90° heading step, which goes straight into the LoS surge
feedforward and the PID feedforward. Either remove it or replace it with `kin_square`, which
is the properly time-parameterised version of the same idea.

**F7 — `sin` and `kin_square` jump *backwards* when the parameter runs out.** *(closed)*
`if t > 500: t = 50` was not a clamp — it teleported the reference back to the pose at t = 50
(`kin_square` went from x = 75.0 m at t = 499 to x = 10.0 m at t = 501). Both are now
`t = min(t, 500.0)`, the "hold the last point" convention every other shape and the YAML
loader already used, so the library has one rule rather than two.

Past t = 500 the reference is stationary, so the authored speed `U_d` goes to zero and the
station-keeping hold (F2) takes over: the boat holds the end of the shape instead of chasing a
teleport. Below t = 500 both shapes are bit-identical to what they were, and no harness
scenario reaches t = 500 at all (the longest gets to τ ≈ 102), so every number in
`CONTROLLERS.md` is unaffected. The same change is mirrored in
[docs/controllers/sim.py](docs/controllers/sim.py), whose copy of `single_pose` must stay
verbatim; `check_trajectory_library.py` asserts that it has.

### 🟡 Worth fixing

**F8 — Body-frame velocity feedback. Decided: no change, the disabled correction was wrong.**
[robot_interface.py:467-472](robot_interaction/robot_interface.py#L467-L472) — the disabled
block argued that MAVROS's linear velocity arrives in the raw `map` frame, so that rotating it
by −yaw0 was needed to keep pose and twist in one frame. It does not arrive in `map`. MAVROS
publishes `/mavros/local_position/odom` with `child_frame_id: base_link` and its twist already
rotated into that frame (`local_position.cpp`, `transform_frame_enu_baselink`, both the
`LOCAL_POSITION_NED` and `_COV` handlers); the ENU velocity goes out separately on
`local_position/velocity_local`.

Measured against mavros 2.14.0 driven by a synthetic FCU holding 1 m/s due **north**, so the
two readings separate: at heading north the odom twist reads (1.000, 0.000), at heading NE it
reads (0.707, 0.707) — the body-frame values — while `velocity_local` reads (0.000, 1.000) at
both. End to end through `robot_interface` at `yaw0 = 45°`, `/blueboat/odom` carries (0.707,
0.707) with the pose re-zeroed to the boot frame. So `master_control`'s `current_twist[0]`
([master_control.py:401](master_control.py#L401)) is already body-frame surge, and enabling the
block would have turned a correct 0.707 into 1.000. The block and its argument are removed;
N3 stands. Two independent facts agree: the pinger dead-reckoning at `robot_interface.py:505`
subtracts `self.vel + ω × p` from body-frame pinger coordinates, which is only dimensionally
correct if `self.vel` is body-frame too.

**F9 — An unknown `trajectory:=` name crashes the service.** *(closed)*
`single_pose` was a chain of `if`s with no `else` and no defaults, so a typo
(`trajectory:=circel`) left a local undefined → `UnboundLocalError` inside the service handler.
rclpy does not marshal a callback exception back to the caller, so the effect was that
`path_generation` **died on the first request** and `master_control` logged "Nothing to target
yet." forever with no hint as to why.

The name is now validated in `PathGeneration.__init__` against `SHAPES`: an unrecognised one
logs a FATAL naming the shape and the full valid set, and the node exits at launch, before
anything is armed. `master_control` then holds at zero thrust, which it already does on the
no-path early return. The `if` chain is also `if`/`elif` with a terminating `else` that raises
`ValueError` carrying the same message, so the pure function is diagnosable on its own and the
unbound local cannot come back.

```
[FATAL] [path_generation]: unknown trajectory 'circel'. valid: station_keeping, circle,
  straight_line, sin, fsin, square, kin_square, seabed_scanning, from_yaml:<abs path>
```

**F10 — `/controller_target` is only published in pinger mode. Decided: no change.**
[master_control.py:432-435](master_control.py#L432-L435) sits inside the `elif use_pinger`
branch, so during path following and manual-target control the topic is silent. It stays that
way. The topic carries the **body-frame** pinger vector, while the world-frame target is
already on `/monitoring_data[4:6]` in every branch (N9) — which is what the station map and
the no-pinger CSV read — and `robot_interface`, the only subscriber in the project, stores the
value without ever reading it. Hoisting would either put two different frames on one topic or
duplicate a signal that already exists.

**F11 — The path tangent comes from the authored yaw, not from the geometry.**
`gamma_p` is read from the pose's quaternion. For the built-in trajectories yaw and direction
of travel agree, so this is correct today. But nothing enforces it: a YAML mission that
authors a crab-wise heading (yaw ≠ course, entirely plausible for a side-scan survey in
current) would feed a wrong tangent into the LoS law and bend the path. Consider deriving
`gamma_p` from `atan2(y1-y0, x1-x0)` and treating the authored yaw as a separate
heading *setpoint*.

**F12 — τ never resets on re-arm.**
`self.time_set` latches `True` on the first tick ([line 402-405](master_control.py#L386-L389))
and is never cleared, so if `/blueboat/controller_ready` drops and comes back (motor
disable/enable, safety stop), τ resumes mid-mission rather than restarting. That may well be
desirable — but it is undocumented and there is no way to command a reset. A `reset_tau`
service would be two lines.

**F13 — Monitoring uses wall clock while control uses the ROS clock.**
`current_time = time.time() - self.initial_time` ([line 407](master_control.py#L649)) versus
`self.get_time()` from `get_clock()` ([line 224](master_control.py#L228)). Under
`use_sim_time:=True` these diverge whenever Gazebo does not run at real time, so the saved
`.npy` timestamps do not line up with the simulation. Also, since the log stores a string
header row alongside float rows, `np.save` silently coerces **the entire array to strings**
([line 214](master_control.py#L751) + [line 568](master_control.py#L399)).

**F14 — No mission-complete signal.**
When a finite mission ends, τ keeps incrementing forever into the clamped region. Nothing
publishes "done", nothing stops the thrusters, nothing tells the operator. Worth a
`/mission_complete` latched Bool once `tau > duration`.

**F15 — Dead code.** *(closed)* `single_request` published to a `self.pose_publisher` that
was never created, so it would have raised `AttributeError` if anything had called it. Nothing
did. It and the then-unused `std_msgs` import are gone.

**F16 — Tuning constants are ROS parameters.** *(closed)*
`_declare_tuning_parameters` declares all of them — the governor pair and the new cross-track
pair, both lookaheads, every LoS and PID gain, the MPC horizon and weights, the point-following
gains, `safety_distance`, `thrust_limit` and `control_dt` — each with its previous value as the
default, so nothing moved. These are exactly the knobs you want to change on a boat ramp
without a rebuild, and now you can: `--params-file`, or `-p <name>:=<value>`.

**F18 — No zero-thrust on loss of reference.** *(closed)*
Several paths in `timer_callback` returned early without publishing, and `robot_interface`
kept streaming the **last received** `thruster_input` to the motors, so a stalled
`master_control` left the boat running at its last commanded thrust indefinitely. Fixed at
both ends. `master_control` publishes an explicit `[0, 0]` on each of its three early returns
instead of falling silent. `robot_interface` and `simulation_interface` each run a
`thruster_input_timeout` watchdog (0.5 s, a declared parameter — ten missed ticks of the
producer's 20 Hz loop, and well inside ArduPilot's own `RC_OVERRIDE_TIME`) that zeroes the
thrust and releases itself when commands resume. That is what covers the case a publish cannot:
a crashed or hung controller.

It zeroes and does **not** disarm: these stalls are transient by design, and `full_stop()`
stays bound to the operator `stop` command. On the real boat the zeroing goes through
`manualMove([0, 0])` without `force`, so the `enable_motors` gate still holds.

Measured. Node-level, at the PWM on `/mavros/rc/override`: 400 messages over 20 s of steady
20 Hz publication with no trip, neutral 1500/1500 reached 0.53 s after the last command, held,
and back to the commanded PWM on resume. In Gazebo, with the boat under way at 0.193 m/s and
3.03 N per thruster, killing `master_control` zeroed both thrusters at t+0.51 s and the boat
coasted to 0.003 m/s.

---

## 10. Suggested order of work

| Priority | Items | Effort |
|---|---|---|
| 1 | **F4** (MPC off-by-one + 7 % speed error), **F5** (cross-track gating) | small |
| 2 | **F14** (mission complete) | small |
| 3 | **F6** (fix or remove `square`) | trivial |

**F1**, **F3**, **F7**, **F9**, **F15** and **F16** are closed; see §9.

---

## 11. Cheat sheet

### Launch

```bash
# Simulation
ros2 launch blueboat_control Sim_launch.py trajectory:=kin_square controller_type:=LoS

# Real boat
ros2 launch blueboat_control BlueBoat_launch.py \
    controller_type:=PID trajectory:=circle enable_motors:=True

# Designer mission
ros2 launch blueboat_control BlueBoat_launch.py \
    controller_type:=LoS trajectory:=from_yaml:/home/op/.config/blueboat_mcs/trajectories/survey.yaml
```

### The knobs that shape the trajectory behaviour

All of the `master_control` rows below are **ROS parameters** (F16), declared in
[`_declare_tuning_parameters`](master_control.py#L753); the "default" column is what they
declare. Override with `-p <name>:=<value>` or `--params-file`, no rebuild.

| Constant | File / line | Default | Effect |
|---|---|---|---|
| `control_dt` | [master_control.py:257](master_control.py#L257) | 0.05 | Control loop period (20 Hz) |
| `path_speed_scale` | [master_control.py:260](master_control.py#L260) | 1.0 | Global mission speed multiplier |
| `gov_Lmin` | [master_control.py:261](master_control.py#L261) | 0.5 m | Along-track gap below which τ runs at full speed |
| `gov_Lmax` | [master_control.py:262](master_control.py#L262) | 3.0 m | Along-track gap at which τ **freezes** |
| `gov_Emin` / `gov_Emax` | [master_control.py:273-274](master_control.py#L273-L274) | 0.5 m / **0** | Cross-track equivalents; `gov_Emax = 0` disables the term (F5) |
| `los_lookahead` | [master_control.py:284](master_control.py#L284) | 2.5 m | LoS aggressiveness (↑ = gentler) |
| `pid_lookahead` | [master_control.py:281](master_control.py#L281) | 2.5 m | Same, for the PID controller |
| `los_ku` / `los_kpsi` / `los_kd` | [master_control.py:285-287](master_control.py#L285-L287) | 20 / 10 / 1 | LoS surge, heading, yaw damping |
| `inner_gains_u` / `inner_gains_r` | [master_control.py:279-280](master_control.py#L279-L280) | 1.0 / 1.5 | PID inner loops (C1 — low against drag) |
| `mpc_horizon` / `mpc_time` | [master_control.py:298-299](master_control.py#L298-L299) | 15 / 2.5 s | MPC prediction window |
| `thrust_limit` | [master_control.py:312](master_control.py#L312) | 20.0 N | Allocator clamp and MPC input bounds |
| `total_time` / `dt` | [path_publisher.py](_custom_libraries/path_publisher.py) | 1000 s / 0.1 s | RViz preview extent only |
| `refresh_period` | [path_publisher.py](_custom_libraries/path_publisher.py) | 5.0 s | How often the whole path is re-requested — what picks up a mission deployed or edited after launch |

### Debugging by symptom

| Symptom | Look at |
|---|---|
| Thrusters go to zero mid-mission | The loss-of-reference watchdog fired: `master_control` stopped publishing. Look for `No /thruster_input for …` in the interface node's log |
| "Nothing to target yet." forever | `/path_request` service down. A bad `trajectory:=` name is no longer a cause — since **F9** it is refused at launch with a FATAL naming the valid set |
| Boat sits still, mission never starts | τ frozen → `e_along` ≥ 3 m. Check the trajectory's start offset (§3) |
| Boat drifts off during station-keeping | Not F2 — that is closed for both controllers. Check `hold_speed` was not launched at 0, which disables the hold in both |
| RViz shows nothing / one dot | Since **F3** the path is re-requested every `refresh_period`, so check `path_generation` is up and, for a `from_yaml` mission, that the file has been deployed |
| Mission runs slower than authored | Working as designed — the governor is throttling. Check `e_along` |
| Wild speed spikes in the log | `trajectory:=square` (**F6**). Not a τ wrap-around — **F7** is closed, the parameter range clamps |
| Path mirrored / diagonal drift on the real boat | Not the velocity frame — **F8** is settled, the twist is body-frame. Check the `SERVO1`/`SERVO3` → right/left wiring, `TODO.md` §2 |
