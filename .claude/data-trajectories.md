# BlueBoat-Control — data, trajectories, robot description, log reviewer

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 6. Data

| Artifact | Path | Nature |
|---|---|---|
| Position/pinger CSV | `<root>/data/Robot_data/{date}-{note}-poslog.csv` | **Raw field record** (`robot_interface` real, `simulation_interface` Gazebo) |
| Controller log | `<root>/data/{ctrl}_data/{date}-{ctrl}_{sim}_data.npy` | Per run |
| Run bag | `<root>/data/Robot_data/<csv stem>/{date}-{note}-bag/` (mcap) | **Raw field record**, sessions only: every topic except `/side_scan_sonar/*`, `/sss_processor/*`, `/sss_ai/*` |
| Path snapshot | `<root>/data/Robot_data/<csv stem>/{date}-{note}-path.yaml` | Copy of a path run's `from_yaml` mission; not for replanner runs |
| Path-update log | `<root>/data/Robot_data/<csv stem>/{date}-{note}-pathlog.csv` | **Raw field record**: one row per `/blueboat/path_update_event`, 3 header rows (`path_update_log.LEGEND`); only if the run received a request; never for the pinger layout |

**Per run in a session**: nothing is claimed until `start`; `master_control` reserves the
`.npy` at `start`, `robot_interface` opens the CSV when `run_state` reports the run (layout
from its follow mode). At `stop` the `.npy` is written and the CSV filed by
`poslog_report.py` **in a subprocess** (own session; a render never holds the GIL against the
20 Hz RC stream). Every run's sidecar carries the same session origin.

**The run bag**: a separate `ros2 bag record` process (`run_bag.py`) on the laptop, never in
`robot_interface`'s interpreter. Pre-armed **paused** while idle (hidden
`Robot_data/.run-bag-<pid>-<k>/`), resumed via `~/resume` at run start (first message within
1 ms), stopped at `stop` on a worker thread (SIGINT 10 s → SIGTERM → SIGKILL), then moved in
by `poslog_report.py <csv> --bag <staging>` (the destination must not exist — write-once).
A launch teardown SIGINTs it too; the shutdown hook waits (bounded) and files the open run.
A recorder that never recorded is deleted; a partial bag is kept (`ros2 bag reindex`).
Services are not recorded. ⚠ simple_launch gives mavros, `param_set` and `path_generation`
`use_sim_time:=True` on the real launch, so their stamps may be 0: use receive timestamps.

`<root>` = `custom_functions.data_root`: `data_dir` parameter → `$BLUEBOAT_DATA_DIR` → the
sourced workspace (normally `~/ros2_ws/data/`) → the working directory. An unwritable root
fails the launch naming the path; names are claimed with `O_EXCL` (`reserve_run_file`), so
same-second runs get `-2`, `-3` (CM-7). Read back only by `poslog_report.py` and the log
reviewer.

**`.npy` schema** `['t','x','y','psi','x_d','y_d','psi_d','u1','u2']` (`NPY_HEADER`),
targets world-frame (N9). The header is a row of **strings** in the same array, so `np.save`
coerces everything to strings — cast back on load. Written by `save_monitoring` on a 2 s
timer and at shutdown, never from the control tick.

**CSV.** Two layouts by `use_UWgps`: **no pinger, 27 columns** and **pinger, 39 columns**,
sharing the first 19 (`robot_log_schema.py`'s docstring is the column list and revision
record). Rows are filled **by column name**. Two **legend rows** (description, unit) precede
the column names, written at file creation (a killed run keeps them); older logs have none
and are never rewritten — `robot_log_schema.split_header` finds the header row in both,
pandas reads a new log with `header=2`. No description or unit may contain a comma. In the
pinger layout `ant_*` is all-zero and `dep` duplicates `aco_z`. `actuation_state`: `0` motors
disabled, `1` live, `2` enabled but not in override, `3` watchdog forcing zero. The
`<stem>-origin.yaml` sidecar carries `latitude`, `longitude` (the local-ENU frame's origin,
latched at the first odom) and `yaw0_rad` (provenance only; since 2026-08-31 `relative_psi`
is absolute ENU yaw, before it was `yaw − yaw0`).

**Simulation writes the same CSV** (`simulation_interface`, always no-pinger): date columns
are **sim time** (1970-01-01), `lin_acc_*` is differentiated odometry (no gravity),
`actuation_state` only 1 or 3, GPS (0, 0) unless something publishes
`/mavros/global_position/global` (MCS bridge or the simulator's shim).

All field data comes from a single site and is weather-limited: treat it as irreplaceable,
and state any control result derived from it as single-site (`project_synthesis.md` §4, §10).

### Known data caveats

- **Manual-target and pinger thrust columns are not calibrated Newtons.** `solve_LoS` builds
  `[v ± 0.295·yaw_rate/2]` (keep-location) / `[v ± 0.295·yaw_rate]` (pursuit, pinger) from a
  surge in m/s and a yaw rate in rad/s, published as force; the allocator *divides* by the
  moment arm where this law multiplies. Those columns are **relative** commands, comparable
  within one run only. Path following (PID, LoS, MPC) is unaffected. Fixing it rescales all
  steering authority in two branches and needs a dock test.
- **`fsin` runs at 0.5 m/s**, not the 0.1 m/s it was authored at (`_FSIN_V`, `e6dff70`): runs
  either side of that commit are not comparable (`TRAJECTORY_SYSTEM.md` §3).
- **The target columns changed meaning once**: logs before `_reference_pose` carry a PID/LoS
  target ~2.5 cm further along the path (the window's second pose).

---

## 7. Trajectories

`PathGeneration.single_pose(t, shape)` serves `station_keeping`, `circle`, `straight_line`,
`sin`, `fsin`, `square`, `kin_square`, `seabed_scanning` and `from_yaml:<abs path>`;
`SHAPES` / `is_valid_shape()` are the importable half of the contract. Library, revision
record and known defects (`square`'s 4 m discontinuity): `TRAJECTORY_SYSTEM.md`.

- **Pure in `t`** for every shape (`fsin` reads an append-only cumulative 0.01 s table), so
  trajectories can be swapped, replayed or hot-reloaded. Speed is baked into each formula.
- **Frame**: `/blueboat/odom`'s local ENU — a shape starting at `(0, 0)`, yaw 0 starts at the
  launch point heading East. GPS-anchored missions arrive pre-translated by the MCS deploy
  step; in a session a non-anchored mission arrives placed at the boat. No transform here.
- The hard-coded shapes are **reference conditions for existing field data**: changing one
  silently invalidates comparison with earlier runs.
- Shapes that end hold their last pose (`sin`, `kin_square` at t = 500, `seabed_scanning` at
  ≈ 77.7 s) and the station-keeping hold takes over; `straight_line`, `square`, `circle`
  never clamp. `fsin`'s `radius` (1.5 m) scales the weave, not its speed.
- An unknown `trajectory:=` is a FATAL at construction; `single_pose` raises `ValueError`.
- `from_yaml` (`blueboat_trajectory/1`: dense `[t, x, y, yaw]`, linear, short-way yaw,
  clamped or `loop: true`) rides inside `trajectory`; the `yaml_path` parameter wins when set
  but no launch file passes it. The file is **watched** (reload on mtime change) and a
  station-keeping pose at the origin is served **until it appears** — load-bearing for
  deferred GPS deployment and the session file.

---

## 8. `blueboat_description`

URDF/xacro, meshes, Gazebo world, spawn chain `world_launch.py` → `upload_rov_launch.py` →
`state_publisher_launch.py`; details in `blueboat_description/CLAUDE.md`. Gazebo plugins
still carry Fortress names (Harmonic's deprecated-name shim — superproject §4.6). In
simulation Gazebo's odometry (20 Hz, `odom_frame: world`) is bridged by
`upload_rov_launch.py` to `/blueboat/odom_gz` (`odom_raw:=true`, default) or straight to
`/blueboat/odom` (`odom_raw:=false`). **Two plants** (`plant`, 2026-11-13): `identified`
(default; `hydrodynamics_identified.xacro` + the yaw-axis inertia + an actuator model that
`simulation_interface` reads from the description and applies) and `bluerov2_legacy`
(`hydrodynamics.xacro`, the description of before, element for element). The **sim-tuned** MPC
model is fitted to the legacy plant (secant damping); the **real-boat** model is what the
field-equivalent configuration runs. Since 2026-11-15 (R-8) the identified plant's heave / roll
/ pitch depend on the xacro argument `vertical_model`: `sea_node` (default, world runs: the
sea node owns damping and hydrostatics, physical CoG) or `standalone` (`Sim_launch.py`'s
default: box + plugin damping + box CoG). The plugin's explicit added mass bounds every added
inertia: `blueboat_description/CLAUDE.md` item 4. A `replay`
xacro argument (default false, default description byte-identical) builds the force-free
boat a bag replay moves by `set_pose`.

---

## 9. `log_reviewer/` — the mission log reviewer app

Standalone PySide6 app (`COLCON_IGNORE`); guidance in `log_reviewer/CLAUDE.md`. Two rules
that bind beyond it: it opens `~/ros2_ws/data/Robot_data/` **strictly read-only** (CM-7;
exports go to `Processed_Robot_data/`), and it takes every number and the plotting frame
from `poslog_report`, so the archived PNG and the app cannot disagree.
