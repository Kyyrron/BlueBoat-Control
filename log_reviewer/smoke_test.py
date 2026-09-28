#!/usr/bin/env python3

r"""
The log reviewer's gate. One plain script, an exit code, no test framework -
the same shape as `.claude/tools/interface_inventory.py`'s gate.

    QT_QPA_PLATFORM=offscreen python3 log_reviewer/smoke_test.py

Runs under the system python3 and under ~/ros2_ws/.venv. It needs no ROS, no
network and no display. It DOES need at least one real poslog under
`~/ros2_ws/data/Robot_data/`; with none it skips cleanly (exit 0) rather than
failing, because a fresh clone has no field data - the same rule the sonar
modules' corpus-gated tests use.

The load-bearing assertion is the last section: after a full session, every
byte and every mtime under Robot_data/ is unchanged. This app reads primary
field record and must never write it (CM-7 / N7).
"""

import hashlib
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                                # noqa: E402

ROBOT_DATA = os.path.join(os.path.expanduser("~"), "ros2_ws", "data", "Robot_data")

PASS, FAIL = [], []


def check(name, condition, detail=""):
    (PASS if condition else FAIL).append(name)
    mark = "ok  " if condition else "FAIL"
    print("  [%s] %s%s" % (mark, name, (" - " + detail) if detail and not condition else ""))
    return condition


def find_logs(limit=6):
    found = []
    for root, _dirs, files in os.walk(ROBOT_DATA):
        for name in sorted(files):
            if name.endswith(".csv") and "-poslog" in name:
                found.append(os.path.join(root, name))
    found.sort(key=os.path.getsize)
    if len(found) <= limit:
        return found
    # Smallest and largest, so both the short-run and long-run paths are hit.
    return found[:limit // 2] + found[-(limit - limit // 2):]


def digest(path):
    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def snapshot(root):
    state = {}
    for base, _dirs, files in os.walk(root):
        for name in files:
            path = os.path.join(base, name)
            try:
                state[path] = (digest(path), os.stat(path).st_mtime_ns)
            except OSError:
                pass
    return state


def main():
    print("BlueBoat log reviewer - smoke test")

    logs = find_logs()
    if not logs:
        print("\nSKIP: no *-poslog.csv under %s - nothing to test against." % ROBOT_DATA)
        return 0
    print("\n%d log(s) under test, %s\n" % (len(logs), ROBOT_DATA))

    before = snapshot(ROBOT_DATA)
    workspace = tempfile.mkdtemp(prefix="log_reviewer_smoke_")
    try:
        section_data(logs)
        section_frames(logs, workspace)
        section_figures(logs, workspace)
        section_export(logs, workspace)
        section_tiles()
        section_gui(logs, workspace)
    finally:
        shutil.rmtree(workspace, ignore_errors=True)

    print("\n[6] field data is untouched")
    after = snapshot(ROBOT_DATA)
    check("no file added or removed under Robot_data",
          set(before) == set(after),
          "added/removed: %s" % (set(before) ^ set(after)))
    changed = [p for p in before if p in after and before[p] != after[p]]
    check("no file modified under Robot_data (content or mtime)", not changed,
          "changed: %s" % changed[:3])

    print("\n%d passed, %d failed" % (len(PASS), len(FAIL)))
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
        return 1
    print("ALL CHECKS PASSED")
    return 0


def section_data(logs):
    print("[1] reading and cropping")
    from reviewer import poslog_bridge as pb
    import poslog_report as pr

    path = logs[-1]
    run = pb.load(path)
    check("load returns rows", run["n_rows"] > 0)
    check("mission time is monotonic", bool(np.all(np.diff(run["t"]) >= -1e-9)))
    check("row_index is identity at load",
          bool(np.array_equal(run["row_index"], np.arange(run["n_rows"]))))

    # The whole point of reusing poslog_report: same file, same numbers.
    mine = pb.compute_metrics(run)
    theirs = pr.compute_metrics(pr.read_poslog(path))
    same = all(_close(mine[k], theirs[k]) for k in theirs if k != "act_fraction")
    check("uncropped metrics match poslog_report exactly", same)

    t0, t1 = run["t_full"]
    lo, hi = t0 + 0.25 * (t1 - t0), t0 + 0.6 * (t1 - t0)
    crop = pb.crop_run(run, lo, hi)
    check("crop keeps fewer rows", 0 < crop["n_rows"] < run["n_rows"])
    check("crop stays inside the window",
          bool(crop["t"][0] >= lo - 1e-9 and crop["t"][-1] <= hi + 1e-9))
    check("crop keeps mission time un-rebased", bool(crop["t"][0] > t0))
    check("crop row_index points back at the source",
          bool(np.array_equal(run["data"]["relative_x"][crop["row_index"]],
                              crop["data"]["relative_x"])))
    check("crop does not mutate the full run", run["n_rows"] == run["n_rows_full"])

    metrics = pb.compute_metrics(crop)
    check("cropped duration is the window", abs(metrics["duration_s"] - (hi - lo)) < 2.0)
    check("cropped row count agrees", metrics["rows"] == crop["n_rows"])

    empty = pb.crop_run(run, t1 + 100.0, t1 + 200.0)
    check("a window catching no row still renders", empty["n_rows"] == 1)

    rows = pb.summary_rows(crop, metrics)
    labels = [label for label, _ in rows]
    check("summary has 21 entries", len(rows) == 21, str(len(rows)))
    check("summary keeps the world origin (the PNG drops it)",
          any("World origin" in label for label in labels))

    for path in logs:
        try:
            other = pb.load(path)
            pb.compute_metrics(other)
            pb.summary_rows(other, pb.compute_metrics(other))
        except Exception as exc:                                   # noqa: BLE001
            check("every log loads: %s" % os.path.basename(path), False, str(exc))
            return
    check("every log on disk loads and summarises", True)


def make_sim_log(real_csv, workspace):
    """A simulation-shaped copy of a real log: same rows, sim clock.

    `Sim_launch.py` sets use_sim_time, so `simulation_interface` stamps rows
    from a clock that starts at zero and a simulated log reads 1970. The GPS
    columns are left FILLED on purpose - that is the GPS-anchored case, the one
    that must still be drawn in metres.
    """
    out = os.path.join(workspace, "1970_01_01-00_00_00-Sim-poslog.csv")
    import csv
    with open(real_csv, newline="") as source, open(out, "w", newline="") as target:
        reader = csv.DictReader(source)
        writer = csv.DictWriter(target, fieldnames=reader.fieldnames)
        writer.writeheader()
        for row in reader:
            row["Year"], row["Month"], row["Day"] = "1970.0", "1.0", "1.0"
            writer.writerow(row)
    with open(os.path.join(workspace, "1970_01_01-00_00_00-Sim-origin.yaml"), "w") as fh:
        fh.write("latitude: 33.930593\nlongitude: 130.7312318\nyaw0_rad: 1.84\n")
    return out


def section_frames(logs, workspace):
    print("\n[1b] simulation is drawn in world coordinates")
    from reviewer import figures as F
    from reviewer import poslog_bridge as pb

    real = pb.load(logs[-1])
    check("a field log is not taken for a simulation", not pb.is_simulation(real))
    check("a field log with fixes is drawn in WGS84",
          pb.track_series(real)["mode"] == "wgs84")

    sim_csv = make_sim_log(logs[-1], workspace)
    sim = pb.load(sim_csv)
    check("a sim-clock log is detected as simulation", pb.is_simulation(sim))
    check("its GPS columns are still populated (the anchored case)",
          bool(np.any(sim["data"]["gps_latitude"] != 0.0)))

    series = pb.track_series(sim)
    check("a GPS-anchored simulation is STILL drawn in metres",
          series["mode"] == "world", series["mode"])
    check("its axes are metres, not degrees",
          "m)" in series["xlabel"] and "\u00b0" not in series["xlabel"], series["xlabel"])
    check("its track is the world-frame pair",
          bool(np.array_equal(series["x"], sim["data"]["relative_x"])))
    check("no latitude or longitude reaches the summary",
          "Gazebo world origin" in dict(pb.summary_rows(
              sim, pb.compute_metrics(sim)))["World origin (lat, lon)"])
    check("the subtitle says simulation",
          "simulation" in pb.subtitle_for(sim, pb.compute_metrics(sim)))
    check("the default track blurb stops promising GPS",
          "GPS degrees" in F.default_texts("x", sim)["track_desc"])

    # The frame is pinned from the whole run, so a crop cannot flip it.
    t0, t1 = real["t_full"]
    crop = pb.crop_run(real, t0, t0 + 5.0)
    check("a crop inherits the pinned frame",
          pb.track_series(crop)["mode"] == "wgs84")
    sim_crop = pb.crop_run(sim, t0, t0 + 5.0)
    check("a simulated crop stays in metres",
          pb.track_series(sim_crop)["mode"] == "world")

    # A real run that never got a fix also falls to the world frame - better
    # than the empty panel it used to get.
    blind = pb.load(logs[-1])
    blind["data"]["gps_latitude"] = np.zeros_like(blind["data"]["gps_latitude"])
    blind["data"]["gps_longitude"] = np.zeros_like(blind["data"]["gps_longitude"])
    blind.pop("frame_mode", None)
    fallback = pb.track_series(blind)
    check("a real run with no fix falls back to world coordinates",
          fallback["mode"] == "world" and not fallback["simulated"])
    check("and says so without blaming simulation",
          "no GPS fix" in F.default_texts("x", blind)["track_desc"])

    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure = F.build_report_figure(sim, F.default_texts(sim["stem"], sim), None)
    FigureCanvasAgg(figure)
    out = os.path.join(workspace, "sim_report.png")
    figure.savefig(out)
    check("the simulated report renders", os.path.getsize(out) > 40_000)

    # And the archived poslog_report PNG makes the same call, from the same code.
    import poslog_report as pr
    check("poslog_report agrees the run is simulated", pr.is_simulation(sim))
    check("poslog_report draws it in the same frame",
          pr.track_series(sim)["mode"] == "world")


def section_figures(logs, workspace):
    print("\n[2] rendering")
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from reviewer import figures as F
    from reviewer import poslog_bridge as pb

    run = pb.load(logs[-1])
    texts = F.default_texts(run["stem"])
    check("default speed title is not the old parenthetical definition",
          "1 s smoothed" not in texts["speed_title"], texts["speed_title"])
    check("the smoothing is explained in the description",
          "second" in texts["speed_desc"])

    figure = F.build_report_figure(run, texts, None)
    FigureCanvasAgg(figure)
    out = os.path.join(workspace, "report.png")
    figure.savefig(out, bbox_inches="tight", pad_inches=0.28)
    check("report renders to a non-trivial PNG", os.path.getsize(out) > 50_000,
          "%d bytes" % os.path.getsize(out))

    # 21 entries into 7 rows x 3 pairs. With per_col=6 the last one falls off,
    # which is the bug in poslog_report._plot_table this must not inherit.
    axes = figure.add_subplot(9, 1, 9)
    rows = pb.summary_rows(run, pb.compute_metrics(run))
    F.plot_table(axes, rows)
    drawn = sum(1 for (_r, c), cell in axes.tables[0].get_celld().items()
                if c % 2 == 0 and cell.get_text().get_text())
    check("every summary entry reaches the table", drawn == len(rows),
          "%d of %d" % (drawn, len(rows)))

    short = pb.crop_run(run, run["t_full"][0], run["t_full"][0] + 2.0)
    figure2 = F.build_report_figure(short, texts, None)
    FigureCanvasAgg(figure2)
    figure2.savefig(os.path.join(workspace, "short.png"))
    check("a two-second crop still renders", True)


def section_export(logs, workspace):
    print("\n[3] export")
    from reviewer import export as E
    from reviewer import figures as F
    from reviewer import poslog_bridge as pb

    source = logs[-1]
    run = pb.load(source)
    t0, t1 = run["t_full"]
    crop = pb.crop_run(run, t0 + 5.0, t0 + 0.5 * (t1 - t0))
    root = os.path.join(workspace, "Processed_Robot_data")
    name = "smoke export"

    folder = E.export(source, name, crop, F.default_texts(name), root=root)
    files = sorted(os.listdir(folder))
    check("four files written", len(files) == 4, str(files))
    for suffix in (".csv", ".png", "-origin.yaml"):
        check("export carries %s" % suffix,
              any(f.endswith(suffix) for f in files), str(files))
    check("export carries export.yaml", "export.yaml" in files)

    exported_csv = os.path.join(folder, name + ".csv")
    with open(source) as handle:
        source_lines = handle.read().splitlines()
    with open(exported_csv) as handle:
        exported_lines = handle.read().splitlines()
    check("header is byte-identical to the source",
          exported_lines[0] == source_lines[0])
    check("row count equals the crop",
          len(exported_lines) - 1 == crop["n_rows"],
          "%d vs %d" % (len(exported_lines) - 1, crop["n_rows"]))
    first_source_row = int(crop["row_index"][0])
    check("rows are copied verbatim, not re-formatted",
          exported_lines[1] == source_lines[first_source_row + 1])

    reopened = pb.load(exported_csv)
    check("an export re-opens in the app", reopened["n_rows"] == crop["n_rows"])
    check("a renamed export still finds its origin sidecar",
          reopened["origin"] is not None and "latitude" in (reopened["origin"] or {}))

    try:
        E.export(source, name, crop, F.default_texts(name), root=root)
        check("re-exporting refuses to overwrite silently", False)
    except E.ExportExists:
        check("re-exporting refuses to overwrite silently", True)
    E.export(source, name, crop, F.default_texts(name), root=root, overwrite=True)
    check("an explicit overwrite is allowed", True)

    import yaml
    with open(os.path.join(folder, "export.yaml")) as handle:
        manifest = yaml.safe_load(handle)
    # Regression: exporting after a pan or zoom hands numpy scalars straight
    # from ax.get_xlim() into the manifest, and PyYAML's SafeDumper refuses
    # them - which used to kill the export AFTER the CSV and PNG were written.
    import numpy as np_
    limits = (np_.float64(130.7302415), np_.float64(130.7310), np_.float64(33.9305),
              np_.float64(33.9312))
    E.export(source, "smoke numpy limits", crop, F.default_texts("x"), root=root,
             track_limits=limits)
    with open(os.path.join(root, "smoke numpy limits", "export.yaml")) as handle:
        numpy_manifest = yaml.safe_load(handle)
    check("a manual framing survives the manifest (numpy scalars)",
          abs(numpy_manifest["track_view"]["lon_min"] - 130.7302415) < 1e-9)
    check("plain() keeps types YAML can write",
          all(isinstance(v, (float, int, str, bool, type(None)))
              for v in numpy_manifest["track_view"].values()))
    check("plain() keeps a bool a bool, not a 1",
          E.plain({"b": np_.bool_(True)})["b"] is True)

    check("manifest names the source", manifest["source"]["csv"] == os.path.abspath(source))
    check("manifest records the crop", manifest["crop"]["rows"] == crop["n_rows"])
    check("manifest records the texts", "speed_desc" in manifest["texts"])

    sim_csv = make_sim_log(source, workspace)
    sim = pb.load(sim_csv)
    sim_crop = pb.crop_run(sim, sim["t_full"][0], sim["t_full"][0] + 30.0)
    sim_folder = E.export(sim_csv, "smoke sim", sim_crop,
                          F.default_texts("smoke sim", sim), root=root)
    check("a simulated run exports", os.path.isdir(sim_folder))
    with open(os.path.join(sim_folder, "export.yaml")) as handle:
        sim_manifest = yaml.safe_load(handle)
    check("the exported simulation keeps world-frame text",
          "GPS degrees" in sim_manifest["texts"]["track_desc"])
    check("safe_name strips path separators", "/" not in E.safe_name("a/b"))


def section_tiles():
    print("\n[4] tiles (offline, the MCS cache)")
    from reviewer.tiles import TileProvider, latlon_to_tile_xy, tile_xy_to_latlon

    provider = TileProvider(fetch=False)
    check("fetching is off in this test", provider.fetch_enabled is False)
    check("cache path is MCS's flat {z}_{x}_{y}.png",
          provider.cache_path(19, 1, 2).endswith("19_1_2.png"))

    lat, lon, zoom = 33.9305353, 130.7313307, 19
    x, y = latlon_to_tile_xy(lat, lon, zoom)
    back_lat, back_lon = tile_xy_to_latlon(x, y, zoom)
    check("slippy-map round trip", abs(back_lat - lat) < 1e-6 and abs(back_lon - lon) < 1e-6)

    if not os.path.isdir(provider.cache_dir) or not os.listdir(provider.cache_dir):
        print("  [skip] no MCS tile cache on this machine")
        return
    tiles = provider.tiles_for(lon - 0.0015, lon + 0.0015, lat - 0.001, lat + 0.001, 900)
    check("cached tiles decode (JPEG bytes under a .png name)", len(tiles) > 0,
          "found %d" % len(tiles))
    if tiles:
        extent, image = tiles[0]
        check("tile image is 256x256 RGB", image.shape[:2] == (256, 256))
        check("tile extent is west<east, south<north",
              extent[0] < extent[1] and extent[2] < extent[3])
    check("open ocean with nothing cached returns nothing, without raising",
          provider.tiles_for(0.0, 0.002, 0.0, 0.002, 900) == [])


def section_gui(logs, workspace):
    print("\n[5] the window (offscreen)")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication

    from reviewer.app import ReviewerWindow

    app = QApplication.instance() or QApplication([])
    window = ReviewerWindow(csv_path=logs[-1], fetch_tiles=False)
    window.resize(1500, 900)
    window.show()
    app.processEvents()
    check("a log opens", window.run is not None)
    check("the name entry seeds from the stem",
          window.name_edit.text() == window.run["stem"])
    check("the summary table is filled", window.table.rowCount() == 21)

    t0, t1 = window.run["t_full"]
    window.slider.set_values(t0 + 10.0, t0 + 0.5 * (t1 - t0))
    window._rebuild()
    app.processEvents()
    check("moving the timeline crops the run",
          window.crop["n_rows"] < window.run["n_rows_full"])
    check("the table follows the crop",
          window.table.item(1, 1).text().startswith(str(window.crop["n_rows"])))

    window.play_button.setChecked(True)
    for _ in range(5):
        window._tick()
    app.processEvents()
    check("replay advances inside the window",
          window.slider.values()[0] <= window._playhead <= window.slider.values()[1])
    check("replay draws a trail", len(window.track._replay[1].get_xdata()) > 0)
    window._set_speed(8)
    before = window._playhead
    window._tick()
    check("x8 advances eight times faster",
          abs((window._playhead - before) - 8 * 0.033) < 0.02)
    window.play_button.setChecked(False)
    check("pausing stops the frame timer", not window._frame.isActive())

    import types
    speed_panel = window.panels["speed"]
    middle = 0.5 * (window.crop["t"][0] + window.crop["t"][-1])
    speed_panel._on_motion(types.SimpleNamespace(
        inaxes=speed_panel.ax, xdata=middle, ydata=0.5))
    app.processEvents()
    check("hovering a panel marks every time-series panel",
          all(any(dot.get_visible() for dot in window.panels[k].cursor.dots)
              for k in ("distance", "speed", "thrust")))
    check("hovering a panel marks the track too",
          any(dot.get_visible() for dot in window.track.cursor.dots))
    check("the thrust panel marks both thrusters",
          sum(dot.get_visible() for dot in window.panels["thrust"].cursor.dots) == 2)
    check("the cursor reads out the instant", "to target" in window.readout.text())
    check("all panels mark the SAME row",
          len({window.panels[k].cursor.index_at(middle)
               for k in ("distance", "speed", "thrust")}) == 1)

    row = min(len(window.crop["t"]) - 1, 40)
    at_row = window.track._series
    reverse = window.track._hover_time_at(types.SimpleNamespace(
        xdata=float(at_row["lon"][row]), ydata=float(at_row["lat"][row])))
    check("hovering the track maps back to that row's time",
          reverse is not None and abs(reverse - window.crop["t"][row]) < 1e-6)
    check("hovering open water marks nothing",
          window.track._hover_time_at(types.SimpleNamespace(xdata=0.0, ydata=0.0)) is None)

    window._hover(None)
    app.processEvents()
    check("leaving a panel clears every cursor",
          not any(dot.get_visible() for k in ("distance", "speed", "thrust")
                  for dot in window.panels[k].cursor.dots)
          and window.readout.text() == "")

    window.track._set_limits(*(window.track.ax.get_xlim() + window.track.ax.get_ylim()))
    check("panning records a manual framing", window.track.limits() is not None)
    window.track.fit()
    check("Fit forgets it again", window.track.limits() is None)

    window.headers["speed"].title.setText("Vitesse")
    window._texts_changed()
    check("editing a title reaches the export texts",
          window.texts["speed_title"] == "Vitesse")

    window.open_log(logs[0])
    app.processEvents()
    check("a second log opens in place without restarting",
          window.run["path"] == logs[0])
    check("the texts reset for the new log",
          window.texts["speed_title"] != "Vitesse")
    window.close()


def _close(a, b):
    try:
        if a != a and b != b:          # both NaN
            return True
        return abs(float(a) - float(b)) < 1e-9
    except (TypeError, ValueError):
        return a == b


if __name__ == "__main__":
    sys.exit(main())
