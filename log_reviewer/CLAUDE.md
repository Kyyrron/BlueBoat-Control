# CLAUDE.md — `log_reviewer`

Loaded when working under `log_reviewer/`. Section references like §6, N7 point to
`BlueBoat-Control/.claude/CLAUDE.md`.

A standalone **PySide6 desktop app** at the repository root for reading a recorded mission:
open a poslog CSV, trim it on a timeline that re-derives every figure and every number live,
zoom and replay the track over satellite imagery, retitle the panels, and export the trimmed
run with a fresh picture.

**It is not a ROS package.** No `package.xml`, no `setup.py`, no colcon — and it carries a
`COLCON_IGNORE`, because colcon scans every directory under `src/` and would otherwise try.
Nothing in it imports `rclpy`, so it needs no sourced workspace.

```bash
python3 log_reviewer/run.py [<csv>] [--no-fetch-tiles]
QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py     # the gate; skips without a poslog
```

Runs under both `/usr/bin/python3` and `~/ros2_ws/.venv` (run the gate under both);
dependencies PySide6, matplotlib, numpy, PyYAML, requests, Pillow (all in
`requirements.txt`). **It must not acquire `pandas`:** the system interpreter lacks it.

## 9.1 The rule that shapes the whole app

`~/ros2_ws/data/Robot_data/` is **primary field record** (CM-7 / N7 / §6). The app opens it
**strictly read-only** and never renames, moves, rewrites or deletes anything in it. The
toolbar's rename entry renames the **export**, not the log. Everything written goes under
`~/ros2_ws/data/Processed_Robot_data/`, which is derived data, so re-exporting is expected and
overwriting is the operator's call (the app asks first).

Two functions of `poslog_report` are therefore **never called from the app**: `finalise_run`,
which MOVES a CSV and its sidecar, and `render_report`, whose first statement is
`matplotlib.use("Agg")` and would hijack the Qt backend process-wide. `smoke_test.py` §[6]
hashes every file under `Robot_data/` before and after a full session — content and mtime —
and fails if one byte moved.

## 9.2 Reuse, and the two places it deliberately stops

`reviewer/poslog_bridge.py` puts `blueboat_control/src/_custom_libraries/` on `sys.path` and
imports `poslog_report` as a plain module — it is ROS-free and numpy-only at module scope, so
this is safe from a bare Python prompt. `read_poslog`, `read_origin`, `sidecar_path`,
`speed_series`, `distance_series`, `compute_metrics`, `_valid_gps`, the formatters and the
whole palette are re-exported **unchanged**. A change to how speed is smoothed or how a metric
is computed therefore lands in the app automatically, and the app can never report a different
number than the archived per-run PNG for the same rows. The gate asserts that equality
directly.

The **painting** is the app's own (`reviewer/figures.py`), and a change to `render_report`'s
layout does *not* reach the app. That is deliberate — four things differ:

1. every title and description is operator text, not a literal;
2. the track carries satellite tiles and must survive pan and zoom;
3. **there is no actuation-state strip** (the app was asked not to have one). The
   `actuation_state` column is still read: "Thrust live (state 1)" stays in the summary, so
   dropping the plot does not drop the fact;
4. the summary lays **22 entries into 8 rows × 3 pairs** (`plot_table` grows its rows to fit). `poslog_report._plot_table` uses
   `per_col = 6` against 3 columns — 18 slots for 19 entries — so **its last row, the world
   origin, is silently dropped from every archived PNG**. Do not copy that back. The gate
   counts rendered cells against entries.

The other reworked default is the speed panel. The PNG titles it
`Ground speed (|v|, 1 s smoothed, differenced from pose)`, which is a definition wearing a
title's clothes. The app titles it **Ground speed** and says the rest in the description
underneath, in words. The gate asserts the old parenthetical has not come back.

`poslog_report.py` itself is **not modified by any of this** — it sits on a flight node's
shutdown path.

## 9.3 Satellite tiles — MCS's cache, and three ways to get it wrong

`reviewer/tiles.py` reads the Mission Control Station's tile cache: a **flat** directory of
files at `~/.config/blueboat_mcs/tile_cache/` (relocatable — the app reads MCS's
`config.json`), Esri World Imagery, no index and no expiry. Since every mission is flown from
MCS, the area a log covers is normally already cached and the app draws offline. A miss is
fetched in the background and a hole is simply never drawn — never an error box.

Three traps, each of which fails **silently**:

* **The URL is `{z}/{y}/{x}` and the filename is `{z}_{x}_{y}`.** Esri puts row before column;
  MCS's cache name puts column before row. Swap them and you get a valid tile of somewhere
  else on Earth, drawn confidently under the track. (Verified the right way round: a tile
  fetched by this app is byte-identical to the one MCS cached for the same key.)
* **The files are not PNGs.** Esri serves JPEG and MCS writes the body to a `.png` name
  without looking. Anything that decodes by *extension* — `matplotlib.image.imread`
  special-cases `.png` — raises `SyntaxError: not a PNG file` on every tile in the cache.
  Decode by content, through PIL.
* **Writes must be atomic.** MCS writes tiles with a plain `write_bytes`, so this app writes
  `<name>.part` and `os.replace`s it; the two can share the cache with MCS running.

The three slippy-map functions are **copied** from `BlueBoat-MCS/mcs/core/geo.py`, not
imported — CM-3, no module reaches into a neighbour's package, and BlueBoat-Control carries no
path dependency on BlueBoat-MCS. If MCS ever changes that projection maths, the copies are
wrong and must be re-copied.

Tiles are placed with `imshow(extent=[lon_w, lon_e, lat_s, lat_n])` using each tile's **own**
corners in degrees, so the track panel keeps the PNG's familiar lon/lat axes and the
Mercator-on-equirectangular error stays within one tile — well under a pixel at survey scale.

## 9.4 Layout and the pieces worth knowing

| File | What it owns |
|---|---|
| `run.py` | entry point, argument parsing |
| `reviewer/poslog_bridge.py` | the reuse above, `crop_run`, `summary_rows`, `wall_clock` |
| `reviewer/figures.py` | the four painters, the table, `build_report_figure`, `DEFAULT_TEXTS` |
| `reviewer/track_view.py` | the interactive track: zoom, pan, blitted replay, hover |
| `reviewer/pinger_bag.py` | the pinger recovered from the run's ros2 bag: MCAP + CDR reader, time alignment, the filter, the export columns (§9.7) |
| `reviewer/replay_video.py` | the export's ×5–×20 replay GIF and its timing (§9.5) |
| `reviewer/cursor.py` | `Overlay` (per-canvas blitting) and `LinkedCursor` (the hover dot) |
| `reviewer/timeline.py` | the two-handled range slider (Qt ships none) |
| `reviewer/tiles.py` | §9.3 |
| `reviewer/export.py` | the `Processed_Robot_data/` writer |
| `reviewer/app.py` | the window |

* **`crop_run` does not re-base mission time.** A cropped panel keeps the seconds it had in the
  whole run, so you can still see where you are. Every derived series is recomputed *from the
  crop* rather than sliced out of the full-run result: the crop is the dataset on screen, and a
  mean over a window has to be that window's mean.
* **No pyplot anywhere.** Figures are `matplotlib.figure.Figure` handed to a canvas (Qt on
  screen, Agg for the export), so the app never touches the global backend.
* **`show_headline=False` on screen.** The title and description *are* the entries above each
  canvas; drawing them inside it as well printed every heading twice. Only the export draws
  them.
* **Replay and the linked cursor are blitted, through ONE overlay per canvas.** Their artists
  are `animated=True`, so they are skipped by normal draws; the clean background is captured
  once per real draw and restored per frame. A frame therefore costs the same whether or not
  64 satellite tiles are underneath it — which is what makes a cursor that follows the mouse
  affordable at all.
  **Two blitters on one canvas erase each other**: each restores a background captured without
  the other's artists, so whichever blits second wins. The track's four replay markers and its
  two hover dots therefore share the canvas's single `Overlay`. If a third overlay-drawn thing
  is ever added to a canvas, it joins that `Overlay` — it does not make its own.
* **The linked cursor is one instant on four panels.** Hovering any plot broadcasts a mission
  time; every panel marks the nearest logged row, and the status bar reads the row out. Two
  details are load-bearing: the cursor plots the **same masked arrays the panels plot**, so a
  dot vanishes exactly where its curve does (no target, or a sample excluded as a pose jump)
  rather than claiming a value that was never drawn; and `LinkedCursor` positions its dot **by
  row index**, not by x, which is why the identical class serves both the time-series panels
  (x = time) and the track (x = longitude). Hovering the track finds the nearest sample in
  **metres** — an un-scaled lon/lat distance would snap east-west sooner than north-south —
  and marks nothing beyond a tolerance, so open water is not attributed to a row.
* **`ax.clear()` throws cursor artists away.** Every re-plot must call `cursor.set_data(...)`
  again; an artist left on a cleared axes draws nothing while still looking alive.
* **The scale bar is redrawn on every limit change.** The PNG computes it once from
  `get_xlim()`, which is correct for a static picture and stale the moment anyone zooms.
* **The export CSV is copied field by field**, never re-formatted from the parsed floats — a
  round trip through `float()` and `"%f"` would quietly rewrite field data at lower precision.
* **Nothing numpy reaches PyYAML.** `export.plain()` converts every numpy scalar before
  `safe_dump`, which otherwise raises at the LAST export step (a folder with no manifest).
  numpy leaks in invisibly (`ax.get_xlim()`, `round()` on one); gate §[3] exports with numpy
  limits on purpose. Inside `plain`, **bool is tested after `.item()` and before int**
  (`bool` subclasses `int`, `np.bool_` neither).
* **`export()` resolves its root at call time** (`root=None` → `DEFAULT_ROOT`), never as a
  default argument (bound at import, the overwrite check would guard the wrong folder);
  `app._export` passes `root=E.DEFAULT_ROOT` explicitly.
* **The export's error path ends in a dialog, not a traceback**: `_export` catches broadly
  and names the type, message and folder that may hold partial files.
* **The controller comes from the `.npy` filename, never its contents.** Neither poslog nor
  sidecar records it; `poslog_bridge.find_controller` lists `<root>/{PID,LoS,MPC}_data/` for
  `<run-start stamp>-<ctrl>_<real|simulation>_data.npy` (same stamp as the poslog) and shows
  "unknown" when there is no match (older or renamed logs). A sidecar `controller:` key wins.
* **The replay robot is an arrow** (`figures.arrow_marker`), rotated by the row's
  `relative_psi` (absolute ENU yaw). The track's aspect is metric in both frames, so the
  yaw is the on-screen angle; it holds the last valid yaw across NaN rows.

## 9.5 What an export contains

`~/ros2_ws/data/Processed_Robot_data/<name>/` — `<name>` from the toolbar entry, sanitised
only for path separators:

| File | Content |
|---|---|
| `<name>.csv` | the two legend rows (description, unit), the source header, then the rows inside the timeline, every column, verbatim — or, with **Pinger** ticked, the with-pinger layout (§9.7). The legend is copied from a source written since 2026-10-08, built from `robot_log_schema` for an older one (`export.yaml` `source.legend`: `source` / `schema`) |
| `<name>.png` | the report **as framed in the app**: edited text, current zoom, current tile setting, no actuation strip |
| `<name>-origin.yaml` | a **copy** of the run's origin sidecar, so the world-frame columns stay georeferenceable |
| `<name>.gif` | the track panel **as framed in the app** (zoom, tiles) replayed over the whole selection at the speed chosen in the export dialog (**×5–×20**, default ×10), ~576×504 px |
| `export.yaml` | `schema: blueboat_processed_log/1` — source csv/stem/sidecar/layout/target and row count, the crop (`t_start_s`, `t_end_s`, first/last source row, rows, wall-clock span), the track framing, the video (`speed`, `frame_ms`, `frames`, `duration_s`, or why there is none), and every edited string |

The replay video (`reviewer/replay_video.py`) — what is not obvious:

* **GIF, not MP4, because there is no ffmpeg** on this machine and matplotlib's only other
  writer is Pillow, which both interpreters already carry. No new dependency.
* **It draws with the on-screen replay's own artists.** `figures.replay_series`,
  `make_replay_artists` and `set_replay_frame` are shared by `TrackCanvas` and the video, so
  the two cannot drift apart. The background (tiles included) is drawn once and blitted.
* **The export dialog chooses the name and the speed.** "Export log" opens `ExportDialog`:
  the name (written back to the toolbar entry on accept — that entry is also the picture's
  title) and a ×5–×20 slider, remembered for the session. Its duration preview comes from
  `replay_video.video_timing`, the writer's own arithmetic, so it is the file's length, not an
  estimate; the gate checks the two agree.
* **The speed is exact; the frame count is bounded instead** (`MAX_FRAMES` 600). A long
  window gets longer frames, in 10 ms steps (GIF centiseconds) — a 1471 s run at ×10 plays at
  4 fps for 147 s. Pillow holds every frame until it writes, so the count is the memory bound.
* **Every frame after the first carries only what moved**: one palette for all frames (taken
  from the finished track, so no flicker), unchanged pixels written as a reserved transparent
  index over `disposal=1`. Without it Pillow re-encodes the box between the clock and the
  boat — over satellite imagery most of the panel — every frame: a 100 s field run went from
  14 MB to 0.9 MB.
* **A video failure never costs the export its manifest.** It is caught and recorded in
  `export.yaml` (`video: {error: …}` or `{skipped: …}`), and a stale GIF from an earlier
  export of the same name is removed rather than left to contradict it.

Note for re-opening an export: `poslog_report.sidecar_path` derives the sidecar name from the
`-poslog.csv` suffix, which a renamed export no longer has, so `poslog_bridge.load` falls back
to `<stem>-origin.yaml`. Without that fallback a re-opened export silently loses its origin.

## 9.6 Simulation is drawn in world coordinates, never in GPS

A run recorded in Gazebo is plotted in **local ENU metres** — east/north of the world origin —
in the archived `poslog_report` PNG and in the app alike, and **no latitude or longitude is printed
anywhere for it**: not on the track axes, not in the summary's origin cell. A simulated
position is not a surveyed one, and drawing it in degrees invites reading it as one.

**This holds even when the run carries GPS.** A GPS-anchored Gazebo mission has the Mission
Control Station synthesising `/mavros/global_position/global` from sim odom about an arbitrary
anchor, so `gps_latitude`/`gps_longitude` are populated and look exactly like field data. They
are a re-encoding of the world coordinates, not a measurement, so the frame decision **ignores
whether fixes exist** and asks what produced the run.

**The detector is the CLOCK, not the GPS** (`poslog_report.is_simulation`). `Sim_launch.py`
sets `use_sim_time=True`, so `simulation_interface` stamps every row from a clock that starts
at zero and a simulated log reads **1970** (§6) — `Year < 2000`. No real run can, and no CSV
column had to be added, which matters because the poslog layout is frozen field-record schema
(N7 / CM-7). A sidecar may also declare it outright (`frame: simulation`, or `simulation:
true`) and that wins, so a future writer can be explicit without fighting the heuristic.

Three consequences worth knowing:

* **A real run that lost its fix falls to the same world frame**, which beats the empty "no GPS
  fix in this run" panel it used to get. The default blurb distinguishes the two cases — one
  says the run is simulated, the other says it recorded no fix — so the picture never blames
  the wrong thing.
* **The frame is PINNED from the whole run** (`run["frame_mode"]`, set in `poslog_bridge.load`
  and inherited by `crop_run`'s dict copy). Without it, trimming the timeline into a stretch
  that happens to hold no fix would flip the panel from degrees to metres mid-session, which
  reads as a bug rather than as a frame change.
* **Satellite tiles are off in the world frame** and the checkbox is *disabled with a reason*
  rather than hidden — tiles are georeferenced in degrees, and a greyed box with a tooltip
  stops a simulated log looking like a broken tile cache. The scale bar goes too: in metres the
  ticks are the scale. The north arrow stays, and the aspect is simply 1.

**Field reports are untouched by all of this** — verified, not assumed: a real log rendered
through the current `poslog_report` is **byte-identical** to the same log rendered through the
committed one. The gate's `[1b]` section carries the simulated-frame checks, including that
`poslog_report` and the app agree on the frame for the same file.

## 9.7 The pinger from the run's ros2 bag

A path-mode run logs the no-pinger CSV layout, so its poslog has no pinger columns, but the
run's bag (`<date>-<note>-bag/`, filed beside the poslog) still holds `/uw_gps_data` and
`/blueboat/pinger_coordinates` (every run session tracks the pinger whatever the follow
mode). `reviewer/pinger_bag.py` reads them; the toolbar's **Pinger** box (unticked on open,
enabled only when the bag gave a detection, tooltip says why otherwise) draws them on the
track, in the replay and in the export.

* **No new dependency.** No `mcap` / `rosbag2_py` here: the reader parses MCAP directly (run
  bags have uncompressed chunks; zstd/lz4 only if the module happens to be installed) and the
  Float32MultiArray CDR by hand. A bag is opened with `os.listdir`, not a `*.mcap` glob — a
  bag filed from staging keeps a hidden `.run-bag-…_0.mcap` name. Reading a run takes ~0.1 s.
* **Timestamps.** Bag **receive time** (MCAP `log_time`) only — the date fields inside
  `/uw_gps_data` are the Water Linked unit's own clock (9 s off on 2026-10-28) and mavros
  stamps read 0. The poslog's naive wall clock is read in *this machine's* zone and tied to the
  bag's first message; a gap of a whole multiple of 30 min is a time-zone difference and is
  removed, any other gap > 120 s means the bag is not this run's and nothing is drawn. Samples
  are interpolated against the pose at their own time, never matched to the nearest 3 Hz row.
* **Position.** `/blueboat/pinger_coordinates` (body frame, all-zero = not yet detected)
  rotated into the local-ENU world frame with the logged pose by
  `custom_functions.transform_body_to_world`'s formula, i.e. what `robot_interface` writes as
  `corrected_pinger`. Not the `/uw_gps_data` lat/lon: float32 steps of ~1.4 m east-west here.
* **Filter** (`pinger_bag.FILTER`, recorded in `export.yaml`): drop samples taken while the boat
  turns faster than 0.35 rad/s (a yaw error is multiplied by the range), with a pose gap > 1.5 s,
  or beyond 30 m; drop samples > 3 m from the median of ±10 s of trusted samples (the plateaus of
  a bad fix that dead reckoning carries unchanged); then a 1 m/s speed gate (re-anchoring after
  3 s) and a ±4 s median. On the 2026-10-28 run this keeps 122 of 431 detections. It bounds the
  jumps; it does not make the USBL accurate — the 2026-10-28 12:52 pool run is still scattered
  over ~20 m, which is the data, not the filter.
* **Held, not extrapolated.** A shown estimate holds for `HOLD_S` (8 s) past its last accepted
  sample, then disappears.
* **Export shape.** With Pinger ticked the CSV is a **with-pinger log**: `COLUMNS_PINGER` (the
  target block = the pinger, `aco..filaco` = the latest raw `/uw_gps_data` packet, 0.0 before the
  first detection exactly as the writer does), followed by every source column the pinger
  layout lacks — the path `target_x/y` and `target_latitude/longitude` — so nothing is dropped.
  Shared columns are still copied verbatim; the legend is rebuilt from the schema. The app
  therefore re-opens it with no second code path (and as a pinger log, the pinger is its target
  track). `export.yaml` gains a `pinger:` block (bag, topics, counts, filter, zone shift).
* **A log that logged the pinger itself (with-pinger layout)** needs no bag: its own
  `corrected_pinger_x/y` (already world-frame, stamped with the pose of the same row) go through the
  same filter (`pinger_bag.estimate_from_log`; the range gate uses the boat-to-pinger distance). Its
  track line is labelled **raw_pinger** in place of "target", the filtered pinger is the "pinger"
  diamonds beside it, and the box opens ticked. Its CSV is exported verbatim (the raw columns are
  field record); only the PNG and GIF gain the filtered pinger (`export.yaml` `pinger.columns_added:
  false`).
* **Where it does not apply:** a no-pinger log with no bag beside it (an exported or renamed CSV),
  an unreadable or unaligned bag, or a pinger log that never detected the pinger.
* **The gate's corpus assumption.** Several sections treat the *last* log as a pre-2026-10-08
  one (header on line 1) and build the new format from it; `find_logs` now guarantees that
  instead of assuming the newest log is old.
