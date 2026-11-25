# BlueBoat-Control — interface: the authoritative contract (nodes, topics, MAVROS, services, operator CLI)

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 2. Interface — the authoritative contract

Names are written **absolutely** in the source; `master_control` has `namespace='blueboat'`,
which ROS 2 does not apply to absolute names — rewriting them relative would rename half the
interface (N1). Relative names: `path_publisher` (`set_path`, `path_request`, root namespace)
and the `ROV` helper in `blueboat_control/__init__.py` (`robot_description`, `cmd_<thruster>`,
display-only `blueboat_<thruster>_wrench`
and `blueboat_base`, all under `blueboat`). Cross-module behaviour of these topics (QoS of
`controller_ready`, `/thruster_input` never silent, the 3-value body-frame pinger vector) is
in the superproject `CLAUDE.md` §4.1.

### 2.1 Nodes

All executables are installed **flat** into `lib/blueboat_control` (`CMakeLists.txt`), so
sources import each other as bare modules (`import custom_functions`, `import PID`).

| Executable | Source under `blueboat_control/` | Node | Purpose |
|---|---|---|---|
| `master_control.py` | `src/` | `master_control` (ns `blueboat`) | MPC / PID / LoS; serves `/blueboat/update_path` |
| `simulation_interface.py` | `src/` | `pid_sim` (ns `blueboat`) | Gazebo thrust bridge via `ROV`; sim readiness; position CSV (no-pinger layout) |
| `robot_interface.py` | `src/robot_interaction/` | `blueboat_controller` | MAVROS bridge, thrust→PWM, odom republish, CSV, run bags |
| `param_set.py` | `src/robot_interaction/` | `blueboat_parameter_control` | SERVO function + GCS sysid remapping; watchdogged |
| `uwgps_log.py` | `src/robot_interaction/` | `underwater_gps_logger` | Water Linked UGPS HTTP poller: `/uw_gps/fix` JSON at `raw_rate_hz` (4), the legacy `/uw_gps_data` packet on the 2 Hz full cycle; every request has a (connect, read) timeout (2026-11-24) |
| `pinger_estimator.py` | `src/robot_interaction/` | `pinger_estimator` | The USBL pinger (2026-11-24): filter + legacy value, live tuning file; the ONE publisher of `/blueboat/pinger_coordinates` and `/blueboat/pinger/*`. Filter: ROS-free `pinger_filter.py`; parameters: `pinger_params.py` |
| `path_generation.py` | `src/_custom_libraries/` | `path_generation` | `/path_request` server; trajectory library |
| `path_publisher.py` | `src/_custom_libraries/` | `path_publisher` | Whole-path preview for RViz |
| `bag_replay.py` | `src/` | `bag_replay` | Kinematic Gazebo replay of one run's bag (§2.3) |

**ROS-free by construction** (numpy or stdlib only; keep them so — anything needing `rclpy`
goes in `custom_functions.py`): `frame_math.py`, `robot_log_schema.py`, `thrust_limits.py`,
`path_stamp.py`, `poslog_report.py`, `run_bag.py`, `path_update.py`, `path_update_log.py`,
`replay_attitude.py`, `yaml_trajectory.py`, `odom_emulation.py`, `pinger_filter.py`,
`pinger_params.py` (all `src/_custom_libraries/`), plus `MPC/ur_mpc.py` (acados, no ROS).

`poslog_report.py` (also a CLI) files a finished run: creates `Robot_data/<csv stem>/` and
**moves** the CSV, its `-origin.yaml`, a PNG report and, from a session, the `-path.yaml`
snapshot and the bag in. Moving is allowed, rewriting is not (CM-7): an existing folder is
never touched. matplotlib is imported **lazily** (a flight node must never fail on a missing
plotting library). It decides the plotting frame for itself and the log reviewer (a simulated
run is local ENU metres, never lat/lon). Speeds above `MAX_PLAUSIBLE_SPEED_MS` (5 m/s) are
pose discontinuities: excluded from every statistic and counted, never clipped.

**File organisation.** `master_control.py`, `robot_interface.py` and `path_generation.py`
open with a FILE MAP and banner-numbered sections; `master_control.timer_callback` delegates
to `_mission_reference_step` / `_replanner_reference_step`, `_compute_command`
(`_path_following_command`, `_pinger_command`) and `_record_monitoring`. Method order may
change; what file a thing lives in may not (N1). The `fsin` table must stay at
`path_generation`'s module scope (its functions rebind the globals with `global`).

### 2.2 Internal topics

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface` (real) · `simulation_interface` (sim, field-equivalent: relayed from `/blueboat/odom_gz` at the field rate and delay) · Gazebo bridge (sim, `odom_raw:=false`: legacy, §8) · `bag_replay` | `master_control`, `simulation_interface`, `pinger_estimator` (pose buffer, legacy dead reckoning) — **local ENU** on both: origin = launch point (real) / Gazebo origin (sim), yaw **absolute ENU** |
| `/blueboat/odom_gz` | `nav_msgs/Odometry` | Gazebo bridge — **simulation only**, `odom_raw:=true` (default): Gazebo's own 20 Hz odometry | `simulation_interface` (the relay; also the surge speed its actuator model uses) |
| `/blueboat/pinger_coordinates` | `Float32MultiArray` `[x, y, z]` body frame | `pinger_estimator` (was `robot_interface` until 2026-11-24: same name, type, frame and QoS) — at odom rate, the FILTERED vector `b = R(ψ)ᵀ(p̂ − p_boat)` (zeros until the filter initialises) with `pinger_source:=filter` (default), exactly the v1.1 legacy vector with `legacy` | `master_control` (follow target), GCS `pinger_listener`, MCS |
| `/blueboat/pinger/legacy_body` | `Float32MultiArray` `[x, y, z]` body frame | `pinger_estimator`, always — the v1.1 algorithm ("dragged by the boat between fixes", `pinger_filter.LegacyPinger`) | `robot_interface` (`corrected_pinger_*` columns, same formula as before), MCS legacy layer |
| `/blueboat/pinger/estimate` | `String` (JSON `blueboat_pinger_estimate/1`) | `pinger_estimator`, at each processed fix, every second and right after a parameter change. `status` = the filter's; `source_status` = the status of the source `/blueboat/pinger_coordinates` carries (the filter's, or with `pinger_source:=legacy` the legacy input's freshness) | `master_control` (`source_status` gates following), `robot_interface` (`pf_*` columns), MCS |
| `/blueboat/pinger/fix` | `String` (JSON `blueboat_pinger_fix/1`), reliable depth 50 | `pinger_estimator`, one per processed fix (duplicates and invalid ones included, with the reason) | MCS raw-fix layer, the run bag |
| `/blueboat/pinger/filter_config` | `String` (JSON `blueboat_pinger_filter_config/1`), reliable depth 1 TRANSIENT_LOCAL | `pinger_estimator`: the parameter set in force, its hash, preset, source, and the `configure` seq it answers (`ok`, `detail`; `seq` null after a tuning-file reload) | `robot_interface` (sidecar), MCS ("applied") |
| `/uw_gps/fix` | `String` (JSON `blueboat_uwgps_fix/1`), reliable depth 50 | `uwgps_log`, one per raw poll: every field of every polled endpoint, with request/response ROS times, HTTP status, error | `pinger_estimator` (the filter's input), the bag |
| `/blueboat/path_info` | `String` (JSON `blueboat_path_info/1`), reliable depth 1 TRANSIENT_LOCAL | `path_generation`: the served trajectory, `duration_s` (null when it never ends), `loop`, `loaded` | `master_control` (`path_complete`) |
| `/blueboat/controller_ready` | `Bool` | `robot_interface` · `simulation_interface` | `master_control` |
| `/thruster_input` | `Float32MultiArray` `[right, left]` N | `master_control` | `robot_interface`, `simulation_interface` |
| `/controller_target` | `Float32MultiArray` | `master_control` — **pinger branch only** | `robot_interface` (stored, never read) |
| `/monitoring_data` | `Float32MultiArray` | `master_control` (`simulation_interface` creates a publisher, never publishes) | `robot_interface`, `simulation_interface` (CSV target columns) |
| `/blueboat/param_str` | `String` | `robot_interface` | `param_set` |
| `/blueboat/param_ready` | `Bool` | `param_set` | `robot_interface` |
| `/blueboat/param_mode` | `String` | `param_set` | `robot_interface` |
| `/uw_gps_data` | `Float32MultiArray` (19, legacy packet, unchanged) | `uwgps_log` | `pinger_estimator` (legacy reseed; filter fallback when `/uw_gps/fix` is silent 3 s), `robot_interface` (CSV `aco..filaco`) |
| `/blueboat/run_state` | `String` (JSON) | `master_control` — session only | `robot_interface`, MCS |

### 2.3 External-facing topics

| Topic | Type | Dir | This side | Other party |
|---|---|---|---|---|
| `/blueboat/input_str` | `String` | in | `robot_interface` | Operator CLI, MCS (§2.6) |
| `/blueboat/manual_target` | `Float32MultiArray` `[x, y]` world | in | `master_control` | MCS, GCS (`[0, 0]` = resume sentinel) |
| `/monitoring_data` | `Float32MultiArray` | out | `master_control` | MCS map display |
| `/pose_arrow` | `visualization_msgs/Marker` | out | `master_control` | RViz (simulation only) |
| `/set_path` | `nav_msgs/Path` | out | `path_publisher` | RViz / GCS |
| `/blueboat/arrow_thrust` | `Float32MultiArray` | in | `master_control` | MCS ARROW CONTROL: `[right, left]` N at 20 Hz, `[]` = release (every mode) |
| `/blueboat/run_command` | `String` (JSON) | in | `master_control` — session only; `pinger_estimator` reads a `configure`'s pinger block | MCS: `configure` / `start` / `stop` |
| `/blueboat/run_state` | `String` (JSON) | out | `master_control` — session only | MCS (ack + 1 Hz heartbeat) |
| `/blueboat/run_log` | `String` (JSON) | out | `robot_interface` — session only | MCS (the run's CSV, 1 Hz heartbeat) |
| `/blueboat/update_path` | srv `blueboat_interfaces/srv/UpdatePath` | in | `master_control` — every launch | Replanner, tests: live path update (§4) |
| `/blueboat/path_progress` | `String` (JSON `blueboat_path_progress/1`) | out | `master_control` — every launch, 10 Hz + on change | Replanner, MCS: tau, state, ids, `min_change_point` |
| `/blueboat/path_update_event` | `String` (JSON `blueboat_path_update_event/1`), reliable depth 50 | out | `master_control` | `robot_interface` / `simulation_interface` (the run's `-pathlog.csv`), the bag |

**Run sessions** (`session_mode:=true`, every station real-robot launch). The four
"session only" rows exist only then and only on the real graph; `Sim_launch.py` and a CLI
launch without the argument expose the v1.0 interface (operator plumbing, not a sensor
interface — CM-2). Contracts:

```
run_command  {"cmd":"configure","controller_type":"PID|LoS|MPC","use_pinger":b,"note":s,"mission":s,"seq":n,
              "path_source":"mission|replanner",                     # path_source optional, additive
              "pinger_mode":"off|follow|log","has_path":b,           # additive 2026-11-24 (pinger_mode wins over use_pinger)
              "pinger_source":"filter|legacy",                       # additive: echoed; applied by pinger_estimator
              "pinger_filter":{"preset":s,"params":{...}}}           # additive: applied by pinger_estimator, any time
             {"cmd":"start","seq":n}   {"cmd":"stop","seq":n}
run_state    {"seq":n,"ok":b,"detail":s,"state":"idle|loading|running|error","controller_type":s,
              "pinger_mode":s,"pinger_source":s,"has_path":b,"tau":x,"path_complete":b,   # additive 2026-11-24
              "use_pinger":b,"note":s,"mission":s,"run":n,"npy":path,"path_source":s}
run_log      {"run":n|null,"logging":b,"csv":path,"layout":"no_pinger|pinger|pinger_log",
              "bag_enabled":b,"bag_recording":b,"bag":path}          # bag_*: additive (v1.1)
```

`seq` echoes the last command handled; `configure` is refused during a run (and MPC with the
pinger, and `replanner` with the pinger), `start` unless `state == idle`. All four topics use
the default profile (depth 10, reliable, volatile) at both ends (CM-4); both states are
heartbeated at 1 Hz because one-shots race DDS discovery.

**Bag replay** (`Replay_launch.py` → `bag_replay`): a playback of one real run, never a
simulation (CM-2).

| Topic | Type | Contract |
|---|---|---|
| `/replay/state` | `String` (JSON), out, TRANSIENT_LOCAL depth 1, 2 Hz | `{bag, t, duration, rate, paused, finished, frame: empty\|anchored, world, world_offset, pose_source, gazebo}` + additive `{placement, attitude_source, attitude_rate_hz, attitude_interp, heave: replayed\|pinned_noisy\|off, boat_pose}` |
| `/sim/sea_state/boat_pose` | `geometry_msgs/TransformStamped`, out, 100 Hz, reliable depth 50 (only with `publish_boat_pose`, set by every `Replay_launch.py`) | the replayed pose (frame `world`, child `blueboat`, sim-time stamp, z on the simulator's datum `boat_pose_waterline_z` −0.111) — what `sss_sim_node` renders every ping from |
| `/replay/command` | `String` (JSON), in | `{"cmd":"pause"\|"resume"}`, `{"cmd":"rate","rate":x}` |
| `/blueboat/odom` | `Odometry`, out | rebuilt from the recorded `/mavros/local_position/odom`: **full** attitude, body twist, world frame |
| `/monitoring_data`, `/set_path` | as live, out | targets shifted into the world frame; the path from the run's `-path.yaml` |
| live names | as recorded, out | every other `/mavros/*` but `rc/override`, `/thruster_input`, `/controller_target`, `/uw_gps_data`, `/uw_gps/fix`, `/blueboat/pinger_coordinates`, `/blueboat/pinger/*` (`filter_config` republished latched) |
| `/replay/<name>` | as recorded, out | everything else (commands, session plumbing, the raw odom) — never on a live name |

Every `header.stamp` is rewritten to replay time; the bag's **receive** times drive the
schedule (`BagClock`, pausable). Frames: the empty world is the run's launch frame; a built
world is placed by the median of (recorded fix in the anchor's EN − boat in the launch frame)
over the bag's fixes, or on the world origin with `placement:=origin`. Attitude
(`attitude_source:=auto`): the autopilot's ATTITUDE from `/uas1/mavlink_source`, timed by
`time_boot_ms` (WiFi delay removed), else the v1.1 source; `attitude_interp:=hermite` (cubic,
body rates as tangents; `slerp` = v1.1 bit for bit). Heave: detrended EKF z, replayed only if
its std ≤ `heave_max_std_m` (0.10) and ≤ 5 % clipped (`heave_gate`), else the waterline is
pinned. The replay never starts a sonar; the operator (or the GCS) starts
`sss_sim_launch.py world_dir:=<world> with_mavros_shim:=false auto_ping:=false`. Maths in
`replay_attitude.py` (ROS-free); evidence in `blueboat_control/src/CLAUDE.md`; tests
`test/test_bag_replay_track.py`.

### 2.4 MAVROS boundary

**`robot_interface`** subscribes `/mavros/state` (default QoS), `/mavros/imu/data`,
`/mavros/local_position/odom`, `/mavros/global_position/global` (the last three BEST_EFFORT
depth 10); publishes `/mavros/rc/override`; clients `/mavros/cmd/arming`,
`/mavros/set_mode`. **`param_set`** owns `/mavros/param/pull` and
`/mavros/param/{get,set}_parameters` (rcl_interfaces on the mavros node), never blocks on
service availability in its constructor (checked per request; `robot_interface` re-sends).
FCU endpoint: `BlueBoat_launch.py`'s `fcu_url` (default `udp://:14550@192.168.2.2:14550`;
collides with a running QGroundControl — superproject §4.4).

### 2.5 Services

`/blueboat/update_path` (`UpdatePath`, served by `master_control`): contract in
`docs/PATH_UPDATE_INTERFACE.md`, summary in §4.

`/path_request` (`RequestPath`): request `Float32MultiArray path_request` = path-parameter
values; response `nav_msgs/Path`, one pose per value, `frame_id: "world"`. Server
`path_generation`; clients `master_control`, `path_publisher`, both standalone MPC nodes.
Deliberately parameter-agnostic — keep it so.

**Exactly one server may run, enforced twice** (with two, every request is answered by
whichever replies first and the reference alternated between trajectories — measured):
* `path_generation` refuses to be the second server: before creating the service it
  enumerates the graph (no self-exclusion — the duplicate has its very name) and exits with
  a FATAL (`allow_duplicate_server:=true` opts out; `server_discovery_wait` 2.0 s).
* each pose is **stamped with its path parameter** (`path_stamp.encode`) and
  `master_control.accept_path` rejects a response whose pose count or `poses[0]` stamp does
  not match its request (`RequestPath` untouched). A server predating the stamp is detected
  once ("rebuild"), then a geometric plausibility check applies.

`master_control` also logs an error at startup if it sees more than one server.

### 2.6 Operator CLI

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: <value>"
```

`enable` · `disable` · `stop` · `override` · `default` · `arm` · `disarm` · `rearm` (session
only: `default` mapping → arm → `override`; refused during a run or while latched) · `manual`
(autopilot mode → MANUAL; the station sends it after `default` when the guardian was turned
OFF) · `move <left> <right> <seconds>` (stored `[right, left]`; only without a
`controller_type`). `arm` works only in `default` (Rover refuses to arm on passthrough). An
unrecognised first token falls through to `move_callback`, which needs exactly four fields
(`x 1.0 1.0 5` moves; `1.0 1.0 5` does not); an empty message is ignored.

```bash
ros2 topic pub --once /blueboat/run_command std_msgs/msg/String '{data: "{\"cmd\": \"start\", \"seq\": 1}"}'
```

---

### 2.7 Pinger modes and the pinger pipeline (2026-11-24)

**Modes** (`pinger_mode`, launch argument and `configure` field; `use_pinger` stays as its alias, `true` ≡
`follow`, and keeps meaning "follow" everywhere it exists):

| Mode | Boat | Path | CSV layout | Refused with |
|---|---|---|---|---|
| `off` | path following as before. With `has_path:=false` (2026-11-24): **no `/path_request`**; zero thrust unless a manual target (keep-location) or arrow keys | optional | `no_pinger` (27) | replanner with `has_path` false |
| `follow` | steers to `/blueboat/pinger_coordinates` while the estimate's `source_status` is `tracking`/`coasting`, the estimate is younger than `pinger_stale_timeout` (3 s) and the vector is finite and non-zero; otherwise HOLDS the position where it was lost (keep-location law), warns, resumes on `tracking`. A manual target or the arrow keys drop the hold point (a later hold latches where the operator leaves the boat). A zero vector is never chased | no | `pinger` (52) | MPC; `path_source` replanner |
| `log` | never steers to the pinger. With a path (`has_path` true): follows it with any controller (MPC and replanner allowed). With `has_path:=false`: **no `/path_request` at all**; zero thrust unless a manual target (keep-location) or arrow keys | optional | `pinger_log` (56) | replanner with `has_path` false |

The pinger is estimated and logged in follow and log modes; `pinger_estimator` runs in every real launch.

**Parameters.** `pinger_estimator`: `pinger_source` (`filter` | `legacy`), `pinger_preset` (`slow_rov` |
`static_target`), `pinger_filter_yaml` (the live tuning file, '' = none; `BlueBoat_launch.py` defaults it
to `~/.config/blueboat_control/pinger_tuning.yaml`). `master_control`: `pinger_mode` (''), `has_path`
(true), `pinger_stale_timeout` (3.0 s). `robot_interface`: `pinger_mode` (''). `uwgps_log`: `raw_rate_hz`
(4), `full_rate_hz` (2), `connect_timeout_s` (0.25), `read_timeout_s` (0.4). Every filter parameter (name,
unit, range, what raising / lowering it does): `blueboat_control/config/pinger_filter.yaml`, the one file
the node and the station read; values: `docs/PARAMETER_REGISTER.md`.

**Live tuning.** The node creates its tuning file if missing, every parameter listed commented out with its
value, range and description; a saved edit is applied within a second, a station `configure` pinger block
at any time. The estimate is kept (only a `model` change restarts it); a bad set is refused with the
reason and the previous one stays. Both echo on `/blueboat/pinger/filter_config`. The station re-sends
its dialog set at every `configure`, so that set is the one in force at each run start.

**Timing truth** is the receive time on the estimator's clock (mavros header stamps read 0 on the real
launch), corrected for the time a `/uw_gps/fix` cycle spent on later requests after that endpoint
answered (`t_end − t_resp`, uwgps_log's own clock). A fix's measurement time is `t_rx − latency_s`; the
pose there comes from a 30 s pose history of `/blueboat/odom` receive times (no pose across an odometry
gap longer than 2 s; a clock stepping back restarts it). Only `/uw_gps/fix` feeds the filter;
`/uw_gps_data` drives the legacy pinger. Between fixes the world estimate does not move; the body vector
is recomputed from it at odom rate.
