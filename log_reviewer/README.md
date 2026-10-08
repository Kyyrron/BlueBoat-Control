# BlueBoat log reviewer

An interactive reader for the position CSVs the boat writes during a mission.
Open a log, trim it on a timeline, watch the run play back over satellite
imagery, retitle the panels, and export the trimmed run with a fresh picture.

It is a **standalone desktop app**, not a ROS package: no colcon, no
`package.xml`, no sourced workspace. The folder carries a `COLCON_IGNORE` so a
workspace build skips it.

```bash
python3 log_reviewer/run.py                       # then "Open log…"
python3 log_reviewer/run.py ~/ros2_ws/data/Robot_data/Bathymetrie/<run>/<run>.csv
python3 log_reviewer/run.py --no-fetch-tiles      # never touch the network
```

Runs under `/usr/bin/python3` and under `~/ros2_ws/.venv` alike. Everything it
needs — PySide6, matplotlib, numpy, PyYAML, requests — is already in the
repository's `requirements.txt`.

## What the window does

* **Timeline** — two handles over mission time. Every panel and every number in
  the summary is recomputed from the rows inside the window, live as you drag.
  Type exact seconds in the two boxes, or press **Reset** for the whole run.
* **Track** — wheel to zoom about the cursor, drag to pan, double-click (or
  **Fit**) to frame the selection. One metre east is one metre north at every
  zoom, and the scale bar re-labels itself as you go. **Satellite** draws the
  Mission Control Station's cached imagery underneath.
* **Linked cursor** — put the pointer on any curve and the same instant is
  marked on all four panels at once: a dot riding each curve, a time rule down
  the three time-series panels, and the boat's and the target's positions on
  the track. The status bar reads out that moment — mission time, wall clock,
  distance to target, speed and both thrusts. Hovering the track works the same
  way round: it snaps to the nearest logged position and marks its time
  everywhere else.
* **Replay** — plays the selected window back at ×1, ×2, ×4 or ×8, drawing the
  trail as it goes. The view stays where you put it; drag the scrub bar to jump.
* **Editable text** — every title and description is an entry, not a label.
  What you type is what the exported picture carries. Edits last for the
  session; re-opening a log starts from the defaults again.
* **Simulation** — a Gazebo run is plotted in metres east/north of the world
  origin, with no latitude or longitude anywhere, even when the run carries
  GPS (an anchored mission synthesises its fixes). Satellite tiles are
  disabled there, since they are georeferenced in degrees. A real run that
  lost its fix gets the same world-frame track rather than an empty panel.
* **Name** — renames the *export*. The original log is never renamed.
* **Export log** — opens a small dialog to confirm the name and choose the replay
  video's speed (×5 to ×20, with the resulting GIF length shown live), then writes
  the selection to `~/ros2_ws/data/Processed_Robot_data/<name>/`:

  | file | what it is |
  |---|---|
  | `<name>.csv` | a description row and a unit row, the column names, then the rows inside the timeline, every column, copied field for field |
  | `<name>.png` | the report as you framed it — your text, your zoom, your tiles |
  | `<name>-origin.yaml` | a copy of the run's world-frame origin sidecar |
  | `<name>.gif` | the track replayed at the chosen speed over the whole selection, framed as on screen |
  | `export.yaml` | the source, the crop, the wall-clock span and every edited string |

## The rule that matters

`~/ros2_ws/data/Robot_data/` is **primary field record** and this app opens it
**read-only**. It never renames, moves, rewrites or deletes anything there —
not the CSV, not the sidecar, not the per-run PNG. Everything it writes goes
under `Processed_Robot_data/`. The smoke test asserts this by hashing the whole
tree before and after a full session.

## Gate

```bash
QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py
```

121 checks: reading, cropping, the simulation frame, that the numbers still match `poslog_report`'s
own, rendering, export contents, the tile cache, the window, and that the field
data is byte-for-byte untouched. It needs at least one real poslog under
`Robot_data/` and skips cleanly (exit 0) when there is none.
