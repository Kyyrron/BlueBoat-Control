# The BlueBoat Trajectory System — Complete Review

**Scope:** how a reference trajectory is defined, evaluated, advanced in time, and turned
into a target for the controller, in `blueboat_control`.

**See also:** [CONTROLLERS.md](CONTROLLERS.md) — what each controller does with that target,
compared side by side with simulated plots.

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
   │  asks ONCE for t=0..1000 │   not in the control loop at all
   │  republishes on /set_path│
   └──────────────────────────┘
```

| Node | Role | Rate | In the control loop? |
|---|---|---|---|
| `path_generation` | Evaluates the trajectory function | on demand | **yes** |
| `master_control` | Advances τ, computes thrust | 20 Hz | **yes** |
| `path_publisher` | Draws the whole path in RViz | once, then 1 Hz replay | no |
| `robot_interface` / `simulation_interface` | Motors + odometry | ~20 Hz | yes |

---

## 3. Layer 1 — What a trajectory *is*

Everything lives in one function:
[`PathGeneration.single_pose(t, path_shape)`](_custom_libraries/path_generation.py#L101).

It is a long `if` chain. Give it `t = 12.0` and `path_shape = 'circle'`, it computes x, y and
yaw with a bit of trigonometry and returns a `PoseStamped`. There is **no state, no memory,
no integration between calls** (with one exception, `fsin`, see §9). Ask for `t = 12.0` a
thousand times, you get the same pose a thousand times.

The service [`generate_path`](_custom_libraries/path_generation.py#L295) is just a loop:
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
> ([robot_interface.py:488-495](robot_interaction/robot_interface.py#L488-L495)). So the
> trajectory always starts relative to wherever the boat was switched on. A trajectory that
> begins at (0, 2) or at yaw π/2 asks the boat to make an immediate correction manoeuvre.

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
([`_maybe_reload_yaml`](_custom_libraries/path_generation.py#L77), called on every service
request). If the file doesn't exist yet, `single_pose` returns the origin — i.e. the boat
station-keeps where it started. Once the Mission Control Station has established the
odom↔GPS fit and writes the deployed file, the next path request picks it up (mtime change)
and the boat transitions onto the real-world path. Same mechanism handles editing a
trajectory mid-run.

---

## 5. Layer 3 — How the target moves: τ and the governor

This is the heart of the system. It lives in
[master_control.py:250-284](master_control.py#L250-L284).

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
[`path_progress_errors`](master_control.py#L250) takes the two poses currently in hand
(`poses[0]` = the target at τ, `poses[1]` = a little further along) and computes:

```
gamma_p  = heading of the path at the target       (its tangent)
e_along  = how far AHEAD the target is, measured along the path      [metres]
e_y      = how far SIDEWAYS the boat is from the path                [metres]
U_d      = the authored speed of the path right there                [m/s]
           = distance(pose0, pose1) / (tau spacing)
```

**Step 2 — turn the dial.** [`advance_governor`](master_control.py#L274):

```python
span   = gov_Lmax - gov_Lmin              # 3.0 - 0.5 = 2.5 m
factor = clip((gov_Lmax - e_along)/span, 0, 1)
tau   += path_speed_scale * factor * dt
```

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

**Step 3 — ask for the next window.** [master_control.py:445-450](master_control.py#L445-L450):

```python
request.path_request.data = np.linspace(tau, tau + path_time, path_steps)
self.future = self.client.call_async(request)      # asynchronous: never blocks the loop
```

The result is collected on a **later** tick, when `future.done()` is true
([master_control.py:426-436](master_control.py#L426-L436)). Meanwhile the controller keeps
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

* **`LoS`** ([`los_guidance`](master_control.py#L289)) is purely kinematic — proportional
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
[`timer_callback`](master_control.py#L456-L514):

1. **Manual target** (`/blueboat/manual_target`, from the visualisation app) — point LoS in
   the body frame. **τ is frozen while this is active** ([line 440](master_control.py#L440)),
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
  │  3. measure e_along against poses[0]                                  │
  │  4. GOVERNOR:  tau += path_speed_scale * factor(e_along) * dt         │
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
   all three places it appears ([master_control.py:270-271](master_control.py#L270-L271),
   [master_control.py:306-307](master_control.py#L306-L307),
   [PID.py:166-176](PID/PID.py#L166-L176)) — all three agree with each other and with the
   standard Fossen formulation. That is unusual and worth keeping.

The problems below are all *around* the core, not in it.

---

## 9. Review findings

### 🔴 Blocking

**F1 — `fsin` will stall `path_generation` and, through it, the control loop.**
[path_generation.py:195-204](_custom_libraries/path_generation.py#L195-L204) re-integrates
the trajectory from t=0 in a Python loop with a 0.01 s step, **on every single evaluation**:

```python
steps = int(t / dt)          # t = 300  ->  30,000 iterations, per pose
for i in range(steps): ...
```

At τ = 300 s that is 30 000 iterations × 2 poses × 20 Hz = 1.2 M iterations/s in a Python
loop. `path_publisher` is worse: it requests 10 001 poses up to t = 1000 in one call ≈ **5×10⁸
iterations**, which will appear to hang the launch. It is also the only trajectory that is
not a pure function of `t` in constant time.
*Fix:* solve it in closed form (the yaw integral of a sine is analytic), or cache the
integration and extend it incrementally.

**F2 — `LoS` cannot station-keep, and cannot hold the end of a finished mission.**
The surge command is `u_cmd = los_speed_scale * U_d * max(0, cos(psi_err))`
([master_control.py:310](master_control.py#L310)). When the authored speed `U_d` is zero —
`station_keeping`, a clamped-out YAML mission, or the not-yet-deployed-file fallback — the
surge command is **identically zero regardless of position error**. The boat only steers onto
the x-axis line through the target and then drifts off it with wind and current, with no
force pulling it back. `PID` is fine here because its outer `pid_x` loop acts on the
along-track error directly.
*Fix:* add an along-track proportional term, `u_cmd = U_d*cos + k*e_along`, or fall back to
the PID controller for hold phases.

### 🟠 Important

**F3 — `path_publisher` asks for the path exactly once, at construction.**
[path_publisher.py:38-48](_custom_libraries/path_publisher.py#L38-L48) makes a single
blocking request in `__init__` and then republishes that same frozen `Path` at 1 Hz forever.
Combined with §4's hot-reload feature this means: **for every GPS-anchored mission, RViz
shows a single dot at the origin for the entire run**, because the deployed file did not
exist when `path_publisher` started. The operator's map never shows the real mission.
*Fix:* re-request periodically (e.g. every 5 s), or re-request whenever `path_generation`
announces a reload on a latched topic.

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

**F5 — The governor ignores cross-track error entirely.**
Only `e_along` throttles τ ([master_control.py:441](master_control.py#L441)). A boat that is
perfectly abreast of its target but **20 m off to the side** sees `e_along ≈ 0`, so the
governor runs at full authored speed and the target walks the whole mission while the boat is
nowhere near the path. This is the one case where the reference can still "escape".
*Fix:* gate on the true distance, `hypot(e_along, e_y)`, or multiply in a second factor
`clip((y_max - |e_y|)/y_span, 0, 1)`.

**F6 — `square` is not physically followable.**
[path_generation.py:212-216](_custom_libraries/path_generation.py#L212-L216) flips `y`
between +2 and −2 with `math.floor` — an **instantaneous 4 m teleport**. When that
discontinuity falls inside the 0.05 s window, `compute_target` reports a desired speed of
`4.0 / 0.05 = 80 m/s` and a 90° heading step, which goes straight into the LoS surge
feedforward and the PID feedforward. Either remove it or replace it with `kin_square`, which
is the properly time-parameterised version of the same idea.

**F7 — `sin` and `kin_square` jump *backwards* when the parameter runs out.**
`if t > 500: t = 50` ([line 166](_custom_libraries/path_generation.py#L166) and
[line 232](_custom_libraries/path_generation.py#L232)) is not a clamp — it teleports the
reference back to the pose at t = 50. Every other trajectory in the file, and the YAML
loader, use the "hold the last point" convention. Should be `t = min(t, 500)`.

### 🟡 Worth fixing

**F8 — Body-frame velocity feedback is disabled on the real robot.**
[robot_interface.py:522-543](robot_interaction/robot_interface.py#L522-L543) — the
frame-consistency correction that rotates MAVROS's linear velocity into the boot-relative
frame is **commented out**, with a comment explaining precisely why it is needed ("a fixed
diagonal drift and mirroring heading-swept paths"). Meanwhile `master_control` reads
`current_twist[0]` as body-frame surge `u` ([master_control.py:416](master_control.py#L416)),
which feeds the inner speed loop of both `PID` and `LoS`. Someone disabled this deliberately;
it should be resolved one way or the other and documented, because right now the speed
feedback frame is ambiguous.

**F9 — An unknown `trajectory:=` name crashes the service.**
`single_pose` is a chain of `if`s with no `else` and no defaults, so a typo
(`trajectory:=circel`) leaves `x` undefined → `UnboundLocalError` inside the service handler →
the controller never gets a path and logs "Nothing to target yet." forever, with no hint as
to why. Initialise `x = y = z = roll = pitch = yaw = 0.0` at the top and log a warning on an
unrecognised name. (The `#TODO` on [line 105](_custom_libraries/path_generation.py#L105)
already proposes the dictionary dispatch that would fix this structurally.)

**F10 — `/controller_target` is only published in pinger mode.**
[master_control.py:507-510](master_control.py#L507-L510) sits inside the `elif use_pinger`
branch, so during normal path following the topic is silent. Anything downstream watching the
target (visualisation app, logging) gets nothing. The data exists — `world_target` is computed
in every branch; the publish just needs to be hoisted out.

**F11 — The path tangent comes from the authored yaw, not from the geometry.**
`gamma_p` is read from the pose's quaternion. For the built-in trajectories yaw and direction
of travel agree, so this is correct today. But nothing enforces it: a YAML mission that
authors a crab-wise heading (yaw ≠ course, entirely plausible for a side-scan survey in
current) would feed a wrong tangent into the LoS law and bend the path. Consider deriving
`gamma_p` from `atan2(y1-y0, x1-x0)` and treating the authored yaw as a separate
heading *setpoint*.

**F12 — τ never resets on re-arm.**
`self.time_set` latches `True` on the first tick ([line 402-405](master_control.py#L402-L405))
and is never cleared, so if `/blueboat/controller_ready` drops and comes back (motor
disable/enable, safety stop), τ resumes mid-mission rather than restarting. That may well be
desirable — but it is undocumented and there is no way to command a reset. A `reset_tau`
service would be two lines.

**F13 — Monitoring uses wall clock while control uses the ROS clock.**
`current_time = time.time() - self.initial_time` ([line 407](master_control.py#L407)) versus
`self.get_time()` from `get_clock()` ([line 224](master_control.py#L224)). Under
`use_sim_time:=True` these diverge whenever Gazebo does not run at real time, so the saved
`.npy` timestamps do not line up with the simulation. Also, since the log stores a string
header row alongside float rows, `np.save` silently coerces **the entire array to strings**
([line 214](master_control.py#L214) + [line 568](master_control.py#L568)).

**F14 — No mission-complete signal.**
When a finite mission ends, τ keeps incrementing forever into the clamped region. Nothing
publishes "done", nothing stops the thrusters, nothing tells the operator. Worth a
`/mission_complete` latched Bool once `tau > duration`.

**F15 — Dead code.** [`single_request`](_custom_libraries/path_generation.py#L317) publishes
to `self.pose_publisher`, which is never created — it would raise `AttributeError` if
anything called it. Nothing does. Delete it.

**F16 — Tuning constants are hard-coded, not ROS parameters.**
`path_speed_scale`, `gov_Lmin`, `gov_Lmax`, `los_lookahead`, `pid_lookahead`, `los_ku`,
`los_kpsi`, `los_kd`. These are exactly the knobs you want to change on a boat ramp without a
rebuild. `declare_parameter` for each, with the current values as defaults, costs nothing.

**F17 — Manual target cannot be the origin.**
`manual_active` is `list(self.manual_target) != [0.0, 0.0]`
([line 421](master_control.py#L421)) — the sentinel for "no manual target" is a legal
coordinate. A separate Bool or a NaN sentinel would be cleaner.

**F18 — No zero-thrust on loss of reference.** Several paths in `timer_callback` `return`
early ([lines 371, 411, 514](master_control.py#L514)) without publishing. `robot_interface`
keeps streaming the **last received** `thruster_input` to the motors
([robot_interface.py:815](robot_interaction/robot_interface.py#L815)), so if
`master_control` stops publishing mid-run the boat continues at its last commanded thrust
indefinitely. A watchdog on the interface side (zero the thrusters if no command for ~0.5 s)
would be the safer place to fix this.

---

## 10. Suggested order of work

| Priority | Items | Effort |
|---|---|---|
| 1 | **F1** (`fsin` stall), **F2** (LoS cannot hold station) | small, both localised |
| 2 | **F4** (MPC off-by-one + 7 % speed error), **F5** (cross-track gating) | small |
| 3 | **F3** (RViz never shows YAML missions), **F9** (typo → silent death) | small, big usability win |
| 4 | **F16** (expose the knobs), **F10** (publish the target), **F14** (mission complete) | small |
| 5 | **F8** (velocity frame) — needs a bench test, not just a code change | medium |
| 6 | **F6, F7** (fix or remove `square`, clamp properly) | trivial |

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

| Constant | File / line | Default | Effect |
|---|---|---|---|
| `dt` | [master_control.py:106](master_control.py#L106) | 0.05 | Control loop period (20 Hz) |
| `path_speed_scale` | [master_control.py:125](master_control.py#L125) | 1.0 | Global mission speed multiplier |
| `gov_Lmin` | [master_control.py:126](master_control.py#L126) | 0.5 m | Gap below which τ runs at full speed |
| `gov_Lmax` | [master_control.py:127](master_control.py#L127) | 3.0 m | Gap at which τ **freezes** |
| `los_lookahead` | [master_control.py:194](master_control.py#L194) | 2.5 m | LoS aggressiveness (↑ = gentler) |
| `pid_lookahead` | [master_control.py:180](master_control.py#L180) | 2.5 m | Same, for the PID controller |
| `los_ku` / `los_kpsi` / `los_kd` | [master_control.py:195-197](master_control.py#L195-L197) | 8 / 10 / 1 | LoS surge, heading, yaw damping |
| `mpc_horizon` / `mpc_time` | [master_control.py:132-133](master_control.py#L132-L133) | 15 / 2.5 s | MPC prediction window |
| `total_time` / `dt` | [path_publisher.py:20-21](_custom_libraries/path_publisher.py#L20-L21) | 1000 s / 0.1 s | RViz preview extent only |

### Debugging by symptom

| Symptom | Look at |
|---|---|
| "Nothing to target yet." forever | Bad `trajectory:=` name (**F9**), or `/path_request` service down |
| Boat sits still, mission never starts | τ frozen → `e_along` ≥ 3 m. Check the trajectory's start offset (§3) |
| Boat drifts off during station-keeping | **F2**, LoS with `U_d = 0`. Use `controller_type:=PID` |
| RViz shows nothing / one dot | **F3** — `path_publisher` snapshotted an empty path at boot |
| Mission runs slower than authored | Working as designed — the governor is throttling. Check `e_along` |
| Wild speed spikes in the log | `trajectory:=square` (**F6**), or a τ wrap-around (**F7**) |
| Path mirrored / diagonal drift on the real boat | **F8** — the velocity-frame fix is commented out |
