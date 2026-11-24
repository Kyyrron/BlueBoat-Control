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

## 9.3–9.7 (moved verbatim on 2026-11-18 to `BlueBoat-Control/.claude/log-reviewer-detail.md`)

Read it before touching any of these:
- **9.3 Satellite tiles:** MCS's cache, and three ways to get it wrong.
- **9.4 Layout and the pieces worth knowing.**
- **9.5 What an export contains.**
- **9.6 Simulation is drawn in world coordinates, never in GPS.**
- **9.7 The pinger from the run's ros2 bag.**
