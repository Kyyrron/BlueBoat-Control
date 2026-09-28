# The BlueBoat Trajectory System

**Scope:** how a reference trajectory is defined, evaluated, advanced in time, and turned
into a target for the controller, in `blueboat_control`.

**See also:** `FIELD_TUNING.md` — every tuning knob, with real/sim defaults and a
symptom→knob index. Numbers quoted here were measured in simulation, not on the water.

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
`t` does not depend on what was asked for before it. A name that is not a shape raises rather
than falling through.

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
| `fsin` | Oscillating heading, constant surge — weave of **1.5 m turn radius**, 60 s per cycle | 0.5 m/s | (0, 0), yaw 0 |
| `square` | Square *wave* — instantaneous ±4 m jumps ⚠ | 0.5 m/s + ∞ spikes | (0, **2**), yaw 0 |
| `kin_square` | Zig-zag: +x, +y, +x, −y, 5 m legs | 0.3 m/s | (0, 0), yaw 0 |
| `seabed_scanning` | Scripted survey with arcs and a helix | 0.5 m/s | (0, 0), yaw 0 |
| `from_yaml:<path>` | Designer-generated file | whatever was authored | (0, 0), yaw 0 |

> ⚠️ **`square` is not physically followable.** The `y` flip between +2 and −2 is an
> instantaneous 4 m teleport. When that discontinuity falls inside the 0.05 s reference window,
> `compute_target` reports a desired speed of `4.0 / 0.05 = 80 m/s` and a 90° heading step,
> which goes straight into the LoS and PID speed feedforward. Use `kin_square`, the properly
> time-parameterised version of the same idea.

> ⚠️ **Start alignment matters.** Every shape is expressed in the `/blueboat/odom` world frame,
> which is **local ENU**: the origin is the boat's launch point (position only — yaw is absolute
> ENU and is *not* re-zeroed, fixed 2026-08-31), so a shape authored to start at (0, 0) with
> `yaw = 0` starts at the launch position heading **East**. A trajectory that begins at (0, 2)
> or at yaw π/2 asks the boat for an immediate correction manoeuvre.

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
> | 2026-08-30 | `sin`, `kin_square` | `t > 500` holds the last pose instead of teleporting back to the pose at t = 50. Below t = 500, bit-identical. | **Yes.** The path parameter advances at most 1.0 per second, and no run has come near τ = 500 (the longest harness scenario reaches τ ≈ 160), so the changed region was never exercised. |
> | 2026-08-30 | `fsin` | Per-pose re-integration replaced by a cumulative table on the same 0.01 s grid. | **Yes.** Verified bit-identical to the original loop at every sampled t. |
> | 2026-09-01 | `fsin` | **Turn radius 0.1 m → 1.5 m**, now a named `radius` in the `fsin` branch of `single_pose`. The radius sets the yaw-rate amplitude (`A = V/radius`) and the frequency follows it (`f = A/20`), holding the total yaw swing fixed: the same curve, scaled 15×. | **No.** Every pose moves. The old weave was ±0.23 m wide — the boat could not resolve it — so no earlier run on `fsin` is worth comparing against. |
> | 2026-09-07 | `fsin` | **Surge `_FSIN_V` 0.1 → 0.5 m/s** (commit `e6dff70`). The geometry is unchanged — the same 1.5 m turn radius — but it is traversed 5× faster: one cycle every 60 s instead of 300 s, yaw-rate amplitude 0.333 rad/s instead of 0.067, and `U_d = 0.5` reaching every controller's feedforward. | **No.** `fsin` runs recorded before and after this commit are not comparable. |

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
resolved by the MCS Pattern Designer at export time. `path_generation` only ever does linear
interpolation.

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

`fac_cross` is disabled by default (`gov_Emax = 0`): throttling the target on an error the
inner loops cannot reduce is positive feedback — the target stalls, the boat loses the forward
authority it converges laterally with, and the offset grows. Raise the inner gains first.

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

[`cf.compute_target`](_custom_libraries/custom_functions.py) turns the two poses into a
6-element target `[x, y, psi, u, v, r]`: position and heading from the *second* pose, and
velocities from the difference between them divided by `dt`.

> The **second** pose is what the law is steered at, and that is deliberate — the velocities
> have to come from somewhere. It is **not** what gets logged: `/monitoring_data[4:6]` (and so
> the CSV's `target_x`/`target_y` and the `.npy`'s `x_d`/`y_d`) reports `poses[0]` in every
> branch, via `_reference_pose`, so "target" means one thing whatever controller ran. At the
> 0.05 s window the two differ by about 2.5 cm.

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
   detail. Once the target is reached the boat **holds** it rather than stopping on it
   (`manual_keep_location`), so a current cannot carry it away while
   the operator decides what to do next.
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

## 8. Cheat sheet

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

### The knobs

Every tuning constant is a declared ROS parameter — the full table, with real/sim
defaults and a symptom→knob index, is in `FIELD_TUNING.md`. The two that belong to the
*preview* rather than the control loop live in `path_publisher.py`: `total_time` / `dt`
(1000 s / 0.1 s, the RViz extent) and `refresh_period` (5.0 s, how often the whole path is
re-requested — what picks up a mission deployed or edited after launch).

### Debugging by symptom

| Symptom | Look at |
|---|---|
| Thrusters go to zero mid-mission | The loss-of-reference watchdog fired: `master_control` stopped publishing. Look for `No /thruster_input for …` in the interface node's log |
| "Nothing to target yet." forever | `/path_request` service down. A bad `trajectory:=` name is no longer a cause — it is refused at launch with a FATAL naming the valid set |
| Boat sits still, mission never starts | τ frozen → `e_along` ≥ 3 m. Check the trajectory's start offset (§3) |
| Boat drifts off during station-keeping | Check `hold_speed` was not launched at 0, which disables the zero-authored-speed hold in both controllers |
| Boat drifts off a reached **manual** target | Check `manual_hold_radius` was not launched at 0, which disables the keep-location hold and restores the old abandon-on-arrival behaviour |
| Target jumps somewhere far away every few ticks, on every controller | **Two `/path_request` servers** — a second mission launch that was never shut down. Since 2026-09-03 the second `path_generation` refuses to start, and `master_control` rejects any response that does not answer its own request and logs an error naming both servers. On an older build, check `ros2 node list` for two `path_generation` entries. The station's LIVE DISTANCE plot renders the tick-by-tick alternation as a square wave because it decimates by stride |
| `Path window stale (…) - holding tau` in the log | The path server stopped answering. `tau` is deliberately frozen rather than advanced against a window that is no longer a reference, so the boat holds the last good target instead of running open loop |
| RViz shows nothing / one dot | The path is re-requested every `refresh_period`, so check `path_generation` is up and, for a `from_yaml` mission, that the file has been deployed |
| Mission runs slower than authored | Working as designed — the governor is throttling. Check `e_along` |
| Wild speed spikes in the log | `trajectory:=square` — its 4 m discontinuity (§3). Not a τ wrap-around: the parameter range clamps |
| Path mirrored / diagonal drift on the real boat | Not the velocity frame — the MAVROS twist is body-frame and measured as such (CLAUDE.md N3). Check the `SERVO1`/`SERVO3` → right/left thruster wiring |
