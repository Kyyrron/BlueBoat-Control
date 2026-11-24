# BlueBoat log reviewer

A desktop app to read the position log (`*-poslog.csv`) that the boat writes during a run.
Open a log, trim it on a timeline, replay the run over satellite imagery, edit the panel titles,
and export the trimmed run with a new picture.

It works for real runs and for Gazebo runs. It is **not a ROS package**: no colcon, no sourced
workspace. The folder has a `COLCON_IGNORE`, so `colcon build` skips it.

## Requirements

Python 3 with PySide6, matplotlib, numpy, PyYAML and requests. All are in
`BlueBoat-Control/requirements.txt`. It runs with `/usr/bin/python3` or `~/ros2_ws/.venv`.

## Run

From the `BlueBoat-Control/` folder:

```bash
python3 log_reviewer/run.py                                              # then "Open log..."
python3 log_reviewer/run.py ~/ros2_ws/data/Robot_data/<run>/<run>.csv    # open a log at start
python3 log_reviewer/run.py --no-fetch-tiles                             # offline: cached tiles only
```

## What the window does

| Feature | What it does |
|---|---|
| **Timeline** | Two handles over the run time. Every panel and number is recomputed live for the selected window. Type exact seconds, or **Reset** for the whole run. |
| **Track** | Wheel to zoom, drag to pan, double-click or **Fit** to frame the selection. **Satellite** draws imagery underneath. |
| **Linked cursor** | Hover any curve or the track: the same instant is marked on all panels. The status bar shows time, distance to target, speed and both thrusts. |
| **Replay** | Plays the selection at x1, x2, x4 or x8. Drag the scrub bar to jump. |
| **Editable text** | Every title and description can be edited. The exported picture uses your text. Edits are lost when you reopen a log. |
| **Name** | Sets the name of the export. The original log is never renamed. |
| **Export log** | Choose the name and the replay video speed (x5 to x20), then write the export (below). |

**Simulation runs** are drawn in metres east/north of the world origin, without satellite tiles
(tiles need latitude/longitude). A real run that lost its GPS fix is drawn the same way.

**Satellite tiles** come from the Mission Control Station's cache,
`~/.config/blueboat_mcs/tile_cache/`. Missing tiles are downloaded unless you pass
`--no-fetch-tiles`.

## Export

Written to `~/ros2_ws/data/Processed_Robot_data/<name>/`:

| File | Content |
|---|---|
| `<name>.csv` | The rows inside the timeline, all columns, with the description and unit rows |
| `<name>.png` | The report as framed on screen: your text, zoom and tiles |
| `<name>-origin.yaml` | Copy of the run's world-frame origin file |
| `<name>-bag/` | Copy of the run's full ros2 bag, not trimmed (only if the run has one) |
| `<name>.gif` | The track replayed at the chosen speed |
| `export.yaml` | Source log, crop, time span and every edited text |

## Your field data is safe

`~/ros2_ws/data/Robot_data/` holds the original field records. The app opens it **read-only**:
it never renames, moves, rewrites or deletes anything there. Everything it writes goes to
`Processed_Robot_data/`.

## Test

```bash
QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py
```

It checks reading, cropping, the simulation frame, the numbers, rendering, export, the tile
cache and the window, and that `Robot_data/` is byte-for-byte unchanged. It needs at least one
poslog under `~/ros2_ws/data/Robot_data/` and skips (exit 0) without one.
