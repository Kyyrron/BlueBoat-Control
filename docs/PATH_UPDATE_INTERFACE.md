# Live path update — interface contract

This is the contract a client codes against: BlueBoat-Replanner, the Mission Control Station
(MCS), or a test script. It is complete without reading Control's code.

- Why each rule exists: [PATH_UPDATE_DESIGN.md](PATH_UPDATE_DESIGN.md).
- A worked client: `test/path_update_demo.py`, written against this document only.

It is identical in simulation and on the real boat: same names, types, QoS and semantics.

## 1. Concepts

**Run kinds.** Every run is one of two kinds, fixed for the run:

| `path_source` | What the run follows | Path updates |
|---|---|---|
| `mission` (default) | the fixed mission, as today: the station file or a library shape | refused, `mission_owned` |
| `replanner` | a path held by Control, built only from accepted updates | accepted subject to §5 |

A **pinger** run reports `path_source: pinger` in progress and refuses updates
(`mission_owned`).

**Path seconds and τ.**
- Samples are timed in **path seconds**, written t.
- Control follows a **virtual target** whose position on the path is the path parameter
  **τ**, in the same path seconds.
- τ advances at most one path second per second, and slower when the boat falls behind; it
  freezes when the boat trails by 3 m. **It never goes backwards.**
- So t is a *schedule only if the boat keeps up*: never derive wall-clock times from it. Read
  τ from `/blueboat/path_progress`.
- Within one run, all paths share one t axis. A first path may start at any t[0] ≥ 0, and
  later paths continue that axis.

**Frame `world`.** The frame is the local ENU frame of `/blueboat/odom`:
- origin: the boat's launch point on the real boat; the Gazebo origin in simulation;
- x = East, y = North, metres;
- yaw in **radians, ENU**: counter-clockwise from East. It is **not** a compass heading.

Yaw is the **path tangent** (the direction of travel). It is also the heading the controllers
steer.

## 2. Names, types, QoS

| Name | Type | Direction | QoS |
|---|---|---|---|
| `/blueboat/update_path` | service `blueboat_interfaces/srv/UpdatePath` | client → Control | default service QoS |
| `/blueboat/path_progress` | `std_msgs/String`, JSON | Control → all | reliable, volatile, depth 10. Published at 10 Hz (`path_progress_period`) and immediately after every answer or transition |
| `/blueboat/path_update_event` | `std_msgs/String`, JSON | Control → all | reliable, volatile, **depth 50**. One message per answer and per transition; not heartbeated |

The service exists in **every** launch (`Sim_launch.py`, `BlueBoat_launch.py`), whatever the
run kind. All three names are recorded by the real boat's run bag except the service itself;
ROS 2 bags do not record services.

## 3. The request

```
string id                 # unique among this run's ACCEPTED updates, [A-Za-z0-9._:-]{1,64}
float64 change_point      # path seconds; NaN (or t[0]) for the first path
string frame              # "world"
float64[] t               # path seconds, strictly increasing, steps >= 1 ms, 0 <= t < 1e6
float64[] x               # m, East
float64[] y               # m, North
float64[] yaw             # rad, ENU, the path tangent
```

**How Control evaluates a path.** Use exactly this rule when computing your own junctions. At
a parameter s:
- If s ≤ t[0], the result is sample 0.
- If s ≥ t[n−1], the result is the last sample (**clamped**: the boat holds there).
- Otherwise, with i such that t[i−1] ≤ s < t[i] and u = (s − t[i−1]) / (t[i] − t[i−1]):
  - x = x[i−1] + u·(x[i] − x[i−1]);
  - y = y[i−1] + u·(y[i] − y[i−1]);
  - yaw = wrap( yaw[i−1] + u·wrap(yaw[i] − yaw[i−1]) ), with wrap(a) = atan2(sin a, cos a).

This is the `blueboat_trajectory/1` rule of the Pattern Designer's files.

**What an accepted update does.** It replaces **everything from `change_point` on**:
- the active path is now: the old path before c, the new samples from c;
- samples before c are ignored, so you may start the new samples earlier than c, e.g. on your
  own grid;
- any earlier update whose change point is ≥ c is discarded.

**First path.** While progress `state` is `awaiting_path`:
- set `change_point` = NaN, or equal to t[0];
- the first sample must be within 0.20 m of `progress.hold`;
- on acceptance τ := t[0], and following starts on the next control tick.

## 4. The answer

```
bool accepted
string reason             # "ok", or a refusal code (§5)
string message            # human-readable detail: measured gap, required lead, offending sample
string active_id          # id owning the remainder of the path after this call ("" = none)
float64 tau               # τ at decision time (after a first path: t[0])
float64 min_change_point  # earliest change point accepted now; NaN when none
```

`min_change_point` is NaN in three cases: while awaiting the first path, when no path is
loaded, and when a state refusal applies.

- Every request gets exactly one answer.
- **A refusal changes nothing.** The followed path, τ, the accepted ids and the hold all stay
  as they were.
- A refused id may be reused. An accepted id may not be reused within the run.

## 5. Refusal codes

Checks run in the order of this list; the first failure is the answer.

### State — nothing wrong with the request: retry later, the same request may succeed

| Code | Meaning | What to do |
|---|---|---|
| `not_ready` | E-STOP latched, controller not ready yet, or (replanner run) no odometry yet | wait for progress `state` to leave `not_ready` / show `hold` |
| `no_run` | real-boat session with no run active (idle, loading, error) | wait for the operator's run start |
| `mission_owned` | this run follows a fixed mission or the pinger | not your run: updates need `path_source=replanner` (§7) |
| `operator_override` | a manual target or arrow control is engaged | wait until progress `override` is `none` |

### Request — fix the request

| Code | Meaning |
|---|---|
| `bad_request` | id empty / too long / bad characters; frame not `world`; array lengths differ; non-numeric |
| `duplicate_id` | id already accepted in this run |
| `too_few_points` / `too_many_points` | fewer than 2, or more than `path_update_max_points` (20 000) samples |
| `not_finite` | NaN or inf in t, x, y or yaw |
| `time_not_increasing` | a t step < 1 ms |
| `time_out_of_range` | t < 0 or t ≥ 1e6 |
| `too_fast` | a segment faster than `path_update_max_speed` (2.0 m/s). Usually a unit error |
| `heading_incoherent` | yaw differs from the direction of motion by > 45° on a moving segment |
| `change_point_outside` | first path: c is not NaN / t[0]. Otherwise: c missing, or not in [t[0], t[n−1]) |

`heading_incoherent` usually means a compass heading or degrees were sent. It is caught except
near NE/SW for compass headings, and only when the wrapped value is more than 45° off for
degrees.

### Geometry — recompute the junction

| Code | Meaning |
|---|---|
| `lead_too_short` | c < τ + lead_min. Answer and progress carry `min_change_point` |
| `discontinuous_position` | the new path at c is more than 0.20 m (`path_update_pos_tol`) from the current path at c (first path: from `hold`) |
| `discontinuous_heading` | the new path's yaw at c differs by more than 10° (`path_update_heading_tol`) from the current path's yaw at c |

`discontinuous_heading` is **not** checked where the current path is stationary at c: past
its end, or the hold before a first path. There the boat turns in place.

## 6. Minimum lead

```
lead_min = max( path_time + 2·0.05 s ,  3.0 m / U_d )        (second term only if U_d ≥ 0.05 m/s)
```

- `path_time` is the controller's reference window: 0.05 s for PID/LoS, 6.0 s for MPC in
  simulation, 2.5 s for MPC on the real boat.
- U_d is the current path's speed at τ.
- 3.0 m is the governor's catch-up distance, `gov_Lmax`.
- The factor on that term is `path_update_lead_factor` (1.0).

Typical values:

| Controller | 1.0 m/s | 0.5 m/s | 0.28 m/s | stationary |
|---|---|---|---|---|
| PID / LoS | 3.0 s | 6.0 s | 10.7 s | 0.15 s |
| MPC, simulation | 6.1 s | 6.1 s | 10.7 s | 6.1 s |
| MPC, real | 3.0 s | 6.0 s | 10.7 s | 2.6 s |

**Do not compute this yourself.** Read `min_change_point` from progress, add your own latency
margin, and choose c ≥ that. The demo uses +1 s to +6 s.

## 7. Progress — `/blueboat/path_progress`

```json
{
 "schema": "blueboat_path_progress/1",
 "ros_time": 132.4, "wall_time": 1792042584.25, "run": 2,
 "path_source": "replanner",
 "state": "following",
 "accepting": true, "refusal_now": "",
 "tau": 41.85, "U_d": 0.4, "lead_min": 7.5, "min_change_point": 49.35,
 "following_id": "U1", "active_id": "U2", "pending_ids": ["U2"],
 "t_start": 0.0, "t_end": 118.2, "end_reached": false,
 "hold": null, "override": "none",
 "last_answer": {"id": "U2", "accepted": true, "reason": "ok", "message": "...", "wall_time": 1792042580.1},
 "accepted_count": 3, "refused_count": 0
}
```

**Clock and run fields.**
- `ros_time` is the node clock: simulated time in Gazebo, wall clock on the real boat.
- `run` is the session run number, 0 outside a session.

**`state`** is one of:

| Value | Meaning |
|---|---|
| `not_ready` | E-STOP latched or not ready yet |
| `no_run` | idle session |
| `mission` | fixed mission or pinger run |
| `override` | manual target or arrows engaged |
| `awaiting_path` | replanner run waiting for its first path; `hold` gives its pose |
| `following` | a path is loaded and τ is on it |
| `end_reached` | τ ≥ `t_end`; the boat holds the last pose |

**Request and progress fields.**

| Field | Meaning |
|---|---|
| `accepting` / `refusal_now` | whether a request would pass the state checks now, and the code if not |
| `tau` | the progress along the path, in path seconds. In a fixed-mission run it is that mission's τ |
| `U_d` | the authored speed at τ |
| `lead_min`, `min_change_point` | §6. `min_change_point` is null when not accepting or before the first path |
| `following_id` | the update whose samples τ is currently on |
| `active_id` | the last accepted update, which owns the remainder |
| `pending_ids` | accepted updates whose change point τ has not reached yet |
| `t_start`, `t_end` | the start of the loaded path and its last sample |
| `end_reached` | null when there is no loaded replanner path |

**Progress along the path** is `tau − t_start`; the remaining path is `t_end − tau`, in path
seconds.

## 8. Events — `/blueboat/path_update_event`

The `event` field is one of:
- `request_accepted`;
- `request_refused`;
- `activated` — τ reached a change point: that update is now followed;
- `end_reached`;
- `cleared` — a launch, run start or run stop discarded the path.

Every message carries every key (null when not applicable):

```json
{"schema": "blueboat_path_update_event/1", "seq": 17, "run": 2,
 "ros_time": 140.2, "wall_time": 1792042592.1, "run_time": 140.2,
 "event": "request_refused", "id": "BAD-offset", "reason": "discontinuous_position",
 "message": "the new path starts 1.000 m from the current path at t=63.3 ...",
 "change_point": 63.3, "tau": 55.9, "lead_min": 7.5,
 "points": 51, "t_first": 63.3, "t_last": 88.3, "x_first": 4.1, "y_first": 13.2, "yaw_first": 3.14,
 "pos_gap": 1.0, "heading_gap": 0.0, "heading_checked": true,
 "boat": {"x": 3.0, "y": 11.9, "psi": 3.1},
 "active_id": "U4", "handling_ms": 0.21}
```

**Where events end up.**
- Each event becomes a row of the run's **`<date>-<note>-pathlog.csv`**. It is written beside
  the run's poslog, then filed into `Robot_data/<run>/`.
- The file exists only if the run received at least one request. A pinger run never gets one.
- Its first three rows are description, unit and column names, the same rules as the poslog.

## 9. Precedence — what an update can never do

Safety and operator controls always win:
- E-STOP;
- run stop;
- the motor gate (`enable_motors`);
- manual target;
- arrow control.

While any of them is engaged, requests are refused (`not_ready`, `no_run`,
`operator_override`). Beyond that, an update **never**:
- re-enables the motors;
- releases an E-STOP;
- starts or restarts a run;
- releases a manual target or arrow control.

It only changes the reference the controller follows. When the operator releases an E-STOP or
a manual target, following resumes on the path that was loaded, as a fixed mission resumes
today.

**Run boundaries.**
- `start` and `stop` (sessions) and a launch (simulation, CLI) discard the path and the
  accepted ids, and emit a `cleared` event.
- A new replanner run starts again in `awaiting_path` with a new `hold`.

## 10. Integration guide — who starts what

### 10.1 Fixed mission (today's behaviour — unchanged)

There is nothing to change. MCS launches and configures exactly as before, and the boat
follows the station's file. `/blueboat/path_progress` additionally reports τ (`state: mission`).
Updates sent to such a run are refused with `mission_owned` and logged.

### 10.2 Updatable-path mission — real boat (MCS "Replanner mode", to be added by MCS)

**Until MCS has the mode:** type `path_source:=replanner` in MCS's Launch dialog
**Extra args**. Every run of that session is then replanner-owned.

**1. Launch (session).** As today:

```bash
ros2 launch blueboat_control BlueBoat_launch.py session_mode:=true controller_type:=PID \
    enable_motors:=True trajectory:=from_yaml:<session file>
```

Optionally add `path_source:=replanner` to make it the session's default.

**2. Configure each run.** Add **one** field to the existing `configure` command:

```json
{"cmd": "configure", "controller_type": "PID", "use_pinger": false, "note": "survey3",
 "mission": "replanner", "path_source": "replanner", "seq": 12}
```

- Absent means unchanged.
- `path_source: "replanner"` together with `use_pinger: true` is refused.
- `configure` stays refused during a run, as today.
- `/blueboat/run_state` echoes `"path_source"`.

**3. The session file is not used** by a replanner run. MCS may skip writing `run.yaml`, and
Control does not snapshot it as `-path.yaml` for such runs.

**4. Start** (`{"cmd": "start"}`). The boat holds where it is. Progress shows
`state: awaiting_path` with the `hold` pose.

**5. The Replanner sends the first path, then updates (§10.4).**

**6. MCS should show:**
- progress `state`, `following_id`, `tau / t_end`, `end_reached`;
- the last refusal (`last_answer`).

MCS keeps every safety control as today: E-STOP, Release E-STOP, manual target, arrows,
Stop Run, Turn Off Connection.

**7. Stop** (`{"cmd": "stop"}`) ends the run and discards its path. The next `start` holds
again.

### 10.3 Updatable-path mission — simulation

```bash
ros2 launch blueboat_control Sim_launch.py controller_type:=PID path_source:=replanner
```

- From MCS: the same launch is obtained by adding `path_source:=replanner` in the Launch
  dialog's **Extra args**.
- The launch is the run. The boat holds at its spawn pose until the first path.
- `trajectory:=` is ignored by such a run. `/set_path` (RViz) still shows that argument's
  path, not the updated one.
- The `/blueboat/path_update_event` and progress topics and the pathlog behave exactly as on
  the real boat.
- There is no E-STOP and no run session in simulation, so `not_ready` and `no_run` cannot
  occur after start-up.

### 10.4 The Replanner's loop

**1. Wait** for the service `/blueboat/update_path` and for a progress message.

**2. First path.** Wait for `state == "awaiting_path"` with `hold` set. Build the survey path:
- its first sample at `hold` (x, y), within 0.20 m;
- any yaw, since the boat turns in place;
- t[0] ≥ 0, e.g. 0.

Call the service with `change_point = NaN`.

**3. Track** `tau`, `following_id`, `end_reached` from progress.

**4. To change the rest of the path:**
1. Choose c ≥ `min_change_point` + your latency margin (≥ 1 s recommended).
2. Evaluate **your own last accepted path** at c with the §3 rule. That gives (x_c, y_c, yaw_c).
3. Build the new samples starting at (x_c, y_c) with tangent yaw_c, timed from c on the same
   t axis.
4. Send them with `change_point = c` and a new id, e.g. `"<run>-<k>"`.

**5. On refusal:**
- *state* codes: retry later;
- *request* codes: fix the path;
- `lead_too_short`: re-choose c from the answer's `min_change_point`;
- `discontinuous_*`: re-evaluate your path at c — your copy differs from what Control holds;
  the answer's message gives the gap.

**6. Feasibility is yours.**
- Speeds: author at least 0.28 m/s. Below `hold_speed` = 0.05 m/s the controllers
  station-keep instead of following. In simulation stay at or below 0.45 m/s; the simulated
  boat tops out at about 0.78 m/s.
- Turn radii: the field missions used 1.5–6 m.
- Control checks only sanity bounds (§5). It does not check turn rate.

**7. The end.** At `end_reached` the boat holds the last pose. Append with an update whose c is
past τ: the floor-only lead applies, and the heading is free.

### 10.5 Minimal client (Python, rclpy)

```python
from blueboat_interfaces.srv import UpdatePath
req = UpdatePath.Request()
req.id, req.change_point, req.frame = "survey-1", float("nan"), "world"
req.t, req.x, req.y, req.yaw = ts, xs, ys, yaws          # lists of float
future = client.call_async(req)                         # client = node.create_client(UpdatePath, "/blueboat/update_path")
# ... spin until future.done(); resp = future.result(); resp.accepted, resp.reason, resp.message
```

### 10.6 Command line

```bash
# watch progress and events
ros2 topic echo /blueboat/path_progress --full-length
ros2 topic echo /blueboat/path_update_event --full-length

# first path from a hold at (0, 0): 10 m East at 0.4 m/s
ros2 service call /blueboat/update_path blueboat_interfaces/srv/UpdatePath \
  "{id: 'cli-1', change_point: .nan, frame: world, t: [0.0, 25.0], x: [0.0, 10.0], y: [0.0, 0.0], yaw: [0.0, 0.0]}"
```

## 11. Versioning

- The JSON payloads carry `schema` (`blueboat_path_progress/1`, `blueboat_path_update_event/1`).
- Fields may be **added** within a version. Readers must ignore unknown keys.
- A rename, removal or change of meaning bumps the version.
- `UpdatePath.srv` is a typed interface. It is changed only as a cross-repository decision
  (Control CLAUDE.md N1, superproject CM-1).
