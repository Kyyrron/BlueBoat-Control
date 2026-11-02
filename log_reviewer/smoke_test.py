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

    def legacy(path):               # written before the legend rows of 2026-10-08
        with open(path) as handle:
            return handle.readline().startswith("Year")

    if len(found) > limit:
        # Smallest and largest, so both the short-run and long-run paths are hit.
        found = found[:limit // 2] + found[-(limit - limit // 2):]
    # Several sections treat the LAST log as an old-format one (they build the
    # new format from it), so make sure the last is - whatever the corpus's
    # newest logs look like.
    everything = sorted((p for p in _all_logs() if legacy(p)), key=os.path.getsize)
    if everything and not legacy(found[-1]):
        found = [p for p in found if p != everything[-1]][:limit - 1] + [everything[-1]]
    return found


def _all_logs():
    for root, _dirs, files in os.walk(ROBOT_DATA):
        for name in files:
            if name.endswith(".csv") and "-poslog" in name:
                yield os.path.join(root, name)


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
        section_legend(logs, workspace)
        section_figures(logs, workspace)
        section_pinger(logs, workspace)
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
    check("summary has 22 entries", len(rows) == 22, str(len(rows)))
    check("summary names the controller",
          dict(rows)["Controller"] in ("PID", "LoS", "MPC", "unknown"))
    with tempfile.TemporaryDirectory() as root:
        os.makedirs(os.path.join(root, "LoS_data"))
        open(os.path.join(root, "LoS_data",
                          "2026_01_02-03_04_05-LoS_real_data.npy"), "wb").close()
        fake = {"stem": "2026_01_02-03_04_05-x-poslog", "origin": {}}
        check("find_controller reads the .npy name",
              pb.find_controller(fake, root) == "LoS")
        fake["stem"] = "2026_01_02-03_04_06-x-poslog"
        check("find_controller: no match is unknown", pb.find_controller(fake, root) is None)
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
    from reviewer import poslog_bridge as pb
    with open(real_csv, newline="") as source:
        table = list(csv.reader(source))
    # Either format: legend rows, if the source has them, are kept as they are.
    _legend, top = pb.rls.split_header(table)
    header = table[top]
    date = [header.index(c) for c in ("Year", "Month", "Day")]
    with open(out, "w", newline="") as target:
        writer = csv.writer(target, lineterminator="\n")
        writer.writerows(table[:top + 1])
        for row in table[top + 1:]:
            if row:
                for i, value in zip(date, ("1970.0", "1.0", "1.0")):
                    row[i] = value
                writer.writerow(row)
    with open(os.path.join(workspace, "1970_01_01-00_00_00-Sim-origin.yaml"), "w") as fh:
        fh.write("latitude: 33.930593\nlongitude: 130.7312318\nyaw0_rad: 1.84\n")
    return out


def make_legend_log(old_csv, workspace):
    """A copy of a pre-2026-10-08 log in today's format.

    The two legend rows are written exactly as robot_interface and
    simulation_interface write them - `','.join` over
    `robot_log_schema.legend_rows(columns)` - and every original line follows
    byte for byte, so the two files differ in the legend and nothing else.
    """
    from reviewer import poslog_bridge as pb
    stem = os.path.basename(old_csv)[:-len("-poslog.csv")]
    out = os.path.join(workspace, stem + "-legend-poslog.csv")
    with open(old_csv) as handle:
        lines = handle.read().splitlines()
    columns = lines[0].split(",")
    with open(out, "w") as handle:
        for line in pb.rls.legend_rows(columns):
            handle.write(",".join(line) + "\n")
        handle.write("\n".join(lines) + "\n")
    sidecar = pb.sidecar_path(old_csv)
    if os.path.isfile(sidecar):
        shutil.copy2(sidecar, os.path.join(workspace, stem + "-legend-origin.yaml"))
    return out


def section_legend(logs, workspace):
    print("\n[1c] legend rows (CSVs written since 2026-10-08)")
    from reviewer import poslog_bridge as pb
    rls = pb.rls

    known = set(rls.COLUMNS_NO_PINGER) | set(rls.COLUMNS_PINGER)
    check("the legend covers every column of both layouts",
          known <= set(rls.LEGEND), str(sorted(known - set(rls.LEGEND))))
    long = [c for c, (d, _u) in rls.LEGEND.items() if not 1 <= len(d.split()) <= 4]
    check("every description is 1 to 4 words", not long, str(long))
    bad = [c for c, pair in rls.LEGEND.items()
           if any("," in v or not v.isascii() or not v for v in pair)]
    check("legend text is ASCII, non-empty and comma-free", not bad, str(bad))
    check("an unknown column gets empty legend cells, so the rows still line up",
          rls.legend_rows(["relative_x", "quat_w"]) == ([rls.LEGEND["relative_x"][0], ""], ["m", ""]))

    # Needs a log from BEFORE the legend rows (header on line 1); a corpus that
    # holds only newer logs has nothing to upgrade.
    old = next((p for p in logs if open(p).readline().startswith("Year")), None)
    if old is None:
        print("  (no pre-2026-10-08 log in the corpus - the upgrade checks are skipped)")
        return
    new = make_legend_log(old, workspace)
    with open(new) as handle:
        top = [handle.readline().rstrip("\n").split(",") for _ in range(3)]
    check("a new log opens with description, unit, then column names",
          top[0][7] == rls.LEGEND["relative_x"][0] and top[1][7] == "m"
          and top[2][7] == "relative_x", str([r[7] for r in top]))
    import csv
    with open(old, newline="") as handle:
        old_rows = list(csv.reader(handle))[:3]
    check("split_header: old format has its names on row 1",
          rls.split_header(old_rows) == (None, 0))
    legend, index = rls.split_header(top)
    check("split_header: new format has them on row 3, under its legend",
          index == 2 and legend == (top[0], top[1]))
    check("split_header: a file with no relative_x is not a poslog",
          rls.split_header([["a", "b"], ["1", "2"]]) == (None, None))

    a, b = pb.load(old), pb.load(new)
    check("both formats read to the same layout and row count",
          (a["layout"], a["n_rows"]) == (b["layout"], b["n_rows"]))
    check("both formats read to identical columns",
          set(a["data"]) == set(b["data"]) and all(
              np.array_equal(a["data"][k], b["data"][k], equal_nan=True)
              for k in a["data"]))
    check("both formats give the same metrics",
          str(pb.compute_metrics(a)) == str(pb.compute_metrics(b)))
    check("the reader exposes the legend of a new log, None for an old one",
          a["legend"] is None and b["legend"] is not None
          and b["legend"][1][7] == "m")


def mcap_file(path, topics):
    """A minimal valid MCAP (uncompressed chunk) holding `topics`:
    {name: [(log_time_ns, [float32 values]), ...]} - enough for the reader
    without ROS, mcap or a real bag."""
    import struct

    def rec(op, body):
        return bytes([op]) + struct.pack("<Q", len(body)) + body

    def string(text):
        raw = text.encode()
        return struct.pack("<I", len(raw)) + raw

    def cdr(values):        # Float32MultiArray: no dims, offset 0, then data
        return (b"\x00\x01\x00\x00" + struct.pack("<II", 0, 0)
                + struct.pack("<I", len(values))
                + struct.pack("<%df" % len(values), *values))

    inner = b""
    stamps = []
    for cid, (_name, messages) in enumerate(topics.items(), start=1):
        inner += rec(0x04, struct.pack("<HH", cid, 0) + string(_name) + string("cdr")
                     + struct.pack("<I", 0))
    for cid, (_name, messages) in enumerate(topics.items(), start=1):
        for seq, (when, values) in enumerate(messages):
            stamps.append(when)
            inner += rec(0x05, struct.pack("<HIQQ", cid, seq, when, when) + cdr(values))
    chunk = (struct.pack("<QQQI", min(stamps), max(stamps), len(inner), 0)
             + string("") + struct.pack("<Q", len(inner)) + inner)
    magic = b"\x89MCAP0\r\n"
    with open(path, "wb") as handle:
        handle.write(magic + rec(0x01, string("") + string("smoke")) + rec(0x06, chunk)
                     + rec(0x0F, struct.pack("<I", 0))
                     + rec(0x02, struct.pack("<QQI", 0, 0, 0)) + magic)


def synthetic_pinger_log(logs, workspace, shift_s=0.0, spike=True):
    """A copy of the smallest real no-pinger log with a bag beside it holding a
    STATIC pinger at a known world point, seen from the logged pose.

    Returns (csv path, (px, py)). `shift_s` moves every bag time, to rehearse a
    machine in another time zone (a whole number of half hours) or a bag that
    is not this run's.
    """
    from reviewer import pinger_bag as PB
    from reviewer import poslog_bridge as pb
    source = None
    for candidate in logs:
        loaded = pb.load(candidate)
        if loaded["layout"] == "no_pinger" and loaded["n_rows"] >= 150:
            source = candidate
            break
    if source is None:
        return None, None
    stem = "2026_01_02-03_04_05-synth"
    name = "%s-poslog%s.csv" % (stem, "" if not shift_s else "-%d" % abs(int(shift_s)))
    out = os.path.join(workspace, name)
    shutil.copy(source, out)
    sidecar = pb.sidecar_path(source)
    if os.path.isfile(sidecar):
        shutil.copy(sidecar, pb.sidecar_path(out))
    run = pb.load(out)
    pt, px, py, psi = PB._pose(run)
    target = (float(np.median(px)) + 4.0, float(np.median(py)) - 3.0)
    base = PB.first_row_epoch(run) + shift_s
    times = np.arange(pt[0] + 0.2, pt[-1] - 0.2, 0.2)
    body = []
    for when in times:
        x, y, yaw = (np.interp(when, pt, q) for q in (px, py, psi))
        dx, dy = target[0] - x, target[1] - y
        body.append([np.cos(yaw) * dx + np.sin(yaw) * dy,
                     -np.sin(yaw) * dx + np.cos(yaw) * dy, 0.4])
    first = len(times) // 5                        # not detected before this
    messages = [(int((base + w) * 1e9), [0.0, 0.0, 0.0] if i < first else b)
                for i, (w, b) in enumerate(zip(times, body))]
    if spike:
        k = first + 3 * (len(times) - first) // 4
        messages[k] = (messages[k][0], [18.0, -12.0, 0.4])
    ugps = [(int((base + w) * 1e9),
             [2026, 1, 2, 3, 4, 5, 0] + [1.0, 2.0, 3.0, 0, 0, 0, 0, 0, 0.5] + list(b))
            for w, b in zip(times[first::5], body[first::5])]
    bag = os.path.join(workspace, pb.pr.bag_name(out))
    os.makedirs(bag, exist_ok=True)
    mcap_file(os.path.join(bag, ".run-bag-1-1_0.mcap"),
              {PB.BODY_TOPIC: messages, PB.UGPS_TOPIC: ugps})
    return out, target, first


def section_pinger(logs, workspace):
    print("\n[2b] the pinger from the run's ros2 bag")
    from reviewer import export as E
    from reviewer import figures as F
    from reviewer import pinger_bag as PB
    from reviewer import poslog_bridge as pb

    made = synthetic_pinger_log(logs, workspace)
    if made[0] is None:
        print("  (no no-pinger log in the corpus - skipped)")
        return
    csv_path, target, first = made
    before = digest(csv_path)
    run = pb.load(csv_path)
    PB.attach(run)
    found = run["pinger"]
    check("a bag beside the log is found", PB.find_bag(csv_path) is not None)
    check("the pinger is detected", found is not None, run["pinger_note"])
    if found is None:
        return
    err = np.hypot(found["x"] - target[0], found["y"] - target[1])
    check("a static pinger is recovered at its true place", err.max() < 0.01,
          "max error %.3f m" % err.max())
    check("a 20 m spike is rejected, not smoothed in",
          found["n_accepted"] < found["n_detected"] and err.max() < 0.01)
    check("the all-zero vector is 'not detected', so nothing is drawn before it",
          found["first_t"] > run["t"][0] + 0.2 * (run["t"][-1] - run["t"][0]) - 1.0)

    # Time: a machine in another zone, and a bag that is not this run's.
    for shift, ok in ((9 * 3600.0, True), (-5.5 * 3600.0, True), (3600.0 + 420.0, False)):
        other, _t, _f = synthetic_pinger_log(logs, workspace, shift_s=shift)
        again = pb.load(other)
        PB.attach(again)
        if ok:
            check("a bag a whole zone away still aligns (%+.1f h)" % (shift / 3600.0),
                  again["pinger"] is not None and abs(again["pinger"]["shift_s"] - shift) < 1e-6
                  and np.hypot(again["pinger"]["x"] - target[0],
                               again["pinger"]["y"] - target[1]).max() < 0.01,
                  again["pinger_note"])
        else:
            check("a bag 67 minutes off is refused, not shifted",
                  again["pinger"] is None and "not this run" in again["pinger_note"],
                  again["pinger_note"])

    # What the panels take.
    run["pinger_show"] = False
    check("hidden: nothing joins the replay series", "px" not in F.replay_series(run))
    run["pinger_show"] = True
    series = F.replay_series(run)
    check("shown: the pinger joins the replay series, aligned with the rows",
          "px" in series and len(series["px"]) == run["n_rows"])
    shown = np.isfinite(series["px"])
    early = run["t"] < found["t"][0]
    after = run["t"] > found["t"][-1] + PB.HOLD_S
    check("held from its first detection, not before, and not past HOLD_S of the last",
          shown.any() and not shown[early].any() and not shown[after].any()
          and shown[np.searchsorted(run["t"], found["t"][len(found["t"]) // 2])])
    fig = F.Figure()
    artists = F.plot_track(fig.add_subplot(111), run, F.default_texts(run["stem"], run), None,
                           None, 800, show_headline=False)
    check("the track draws the pinger", artists.get("pinger_line") is not None)
    box = F.track_extent(run)
    px, py = PB.in_panel_frame(run, np.array([target[0]]), np.array([target[1]]))
    check("the frame includes the pinger",
          box[0] <= px[0] <= box[1] and box[2] <= py[0] <= box[3])

    # The export: shaped as a with-pinger log, readable by this same app.
    crop = pb.crop_run(run, run["t"][0], run["t"][-1])
    crop["pinger_show"] = True
    out = os.path.join(workspace, "pinger-export.csv")
    E.write_cropped_csv(csv_path, out, crop["row_index"], PB.export_columns(crop))
    shutil.copy(pb.sidecar_path(csv_path), os.path.join(workspace, "pinger-export-origin.yaml"))
    back = pb.load(out)
    rls = pb.rls
    check("the export reads back as a with-pinger log",
          back["layout"] == "pinger" and back["n_rows"] == run["n_rows"])
    check("its columns are the with-pinger layout, the path target kept after them",
          list(back["data"])[:len(rls.COLUMNS_PINGER)] == list(rls.COLUMNS_PINGER)
          and {"target_x", "target_y", "target_latitude", "target_longitude"}
          <= set(back["data"]))
    check("the path target is not lost",
          np.array_equal(back["data"]["target_x"], run["data"]["target_x"]))
    with open(csv_path, newline="") as a, open(out, newline="") as b:
        import csv as _csv
        src = list(_csv.reader(a))
        dst = list(_csv.reader(b))
    _legend, si = rls.split_header(src)
    _legend, di = rls.split_header(dst)
    sh, dh = src[si], dst[di]
    same = all(src[si + 1 + k][sh.index(c)] == dst[di + 1 + k][dh.index(c)]
               for k in (0, len(back["data"]["Year"]) - 1)
               for c in ("relative_x", "relative_psi", "right_thr_in", "roll"))
    check("the columns both layouts share are copied verbatim", same)
    px_col = back["data"]["corrected_pinger_x"]
    seen = px_col != 0.0
    check("corrected_pinger is 0.0 before the first detection and the pinger after",
          not seen[0] and seen.any()
          and np.hypot(px_col[seen][-1] - target[0],
                       back["data"]["corrected_pinger_y"][seen][-1] - target[1]) < 0.01)
    check("the raw packet fills aco..filaco from the latest message",
          back["data"]["aco_x"][seen][-1] == 1.0 and back["data"]["dep"][seen][-1] == 0.5)
    check("two legend rows open the export",
          dst[0][dh.index("relative_x")] == rls.LEGEND["relative_x"][0]
          and dst[1][dh.index("relative_x")] == "m")
    check("the source log is untouched", digest(csv_path) == before)
    # A log that logged the pinger itself: filtered from its own columns, no bag.
    pinger_run = pb.load(out)
    PB.attach(pinger_run)
    mine = pinger_run["pinger"]
    check("a pinger log is filtered from its own raw_pinger columns, no bag needed",
          mine is not None and mine["bag"] is None
          and np.hypot(mine["x"] - target[0], mine["y"] - target[1]).max() < 0.01,
          pinger_run["pinger_note"])
    pinger_run["pinger_show"] = True
    fig = F.Figure()
    drawn = F.plot_track(fig.add_subplot(111), pinger_run, F.default_texts("x", pinger_run),
                         None, None, 800, show_headline=False)
    labels = [line.get_label() for line in fig.axes[0].get_lines()]
    check("its target track is called raw_pinger and the filtered pinger sits beside it",
          "raw_pinger" in labels and "pinger" in labels and "target" not in labels, str(labels))
    check("a no-pinger log still calls it target", "target" in [
        line.get_label() for line in (lambda a: (F.plot_track(
            a, run, F.default_texts("x", run), None, None, 800, show_headline=False), a.get_lines())[1])(
                F.Figure().add_subplot(111))])
    crop2 = pb.crop_run(pinger_run, pinger_run["t"][0], pinger_run["t"][-1])
    crop2["pinger_show"] = True
    folder2 = E.export(out, "pinger-of-pinger", crop2, F.default_texts("x"),
                       root=os.path.join(workspace, "pexp2"), video_speed=20)
    with open(out) as a, open(os.path.join(folder2, "pinger-of-pinger.csv")) as b:
        check("a pinger log's CSV is exported verbatim (raw columns are field record)",
              a.read().splitlines()[2:] == b.read().splitlines()[2:])


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
    arrow = F.make_replay_artists(F.Figure().add_subplot(111))
    F.set_replay_frame(arrow, {"t": np.array([0.0]), "lon": np.array([0.0]),
                               "lat": np.array([0.0]), "tlon": np.array([np.nan]),
                               "tlat": np.array([np.nan]), "psi": np.array([np.pi / 2])}, 0.0)
    rot = arrow[3]._marker.get_transform().transform([[1.0, 0.0]])[0]
    check("replay arrow points north for psi=pi/2", abs(rot[0]) < 1e-9 and rot[1] > 0)
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
    check("five files written", len(files) == 5, str(files))
    for suffix in (".csv", ".png", "-origin.yaml", ".gif"):
        check("export carries %s" % suffix,
              any(f.endswith(suffix) for f in files), str(files))
    check("export carries export.yaml", "export.yaml" in files)

    exported_csv = os.path.join(folder, name + ".csv")
    with open(source) as handle:
        source_lines = handle.read().splitlines()
    with open(exported_csv) as handle:
        exported_lines = handle.read().splitlines()
    # Exports always open with the legend; the source may or may not have it.
    top = pb.rls.split_header([line.split(",") for line in source_lines[:3]])[1]
    descriptions, units = pb.rls.legend_rows(source_lines[top].split(","))
    check("an old log's export gains the legend rows",
          exported_lines[0] == ",".join(descriptions)
          and exported_lines[1] == ",".join(units))
    check("header is byte-identical to the source",
          exported_lines[2] == source_lines[top])
    check("row count equals the crop",
          len(exported_lines) - 3 == crop["n_rows"],
          "%d vs %d" % (len(exported_lines) - 3, crop["n_rows"]))
    first_source_row = int(crop["row_index"][0])
    check("rows are copied verbatim, not re-formatted",
          exported_lines[3] == source_lines[top + first_source_row + 1])

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

    # The replay video: bounded, and as long as the window and speed say.
    from PIL import Image
    from reviewer import replay_video as V

    def gif_timing(path):
        gif = Image.open(path)
        total = 0
        for k in range(gif.n_frames):
            gif.seek(k)
            total += gif.info.get("duration", 0)
        return gif.n_frames, total

    frames, total_ms = gif_timing(os.path.join(folder, name + ".gif"))
    span = float(crop["t"][-1] - crop["t"][0])
    video = manifest.get("video") or {}
    check("the replay video is animated", frames > 1, str(frames))
    check("the replay video stays under its frame bound",
          frames <= V.MAX_FRAMES + 1, str(frames))
    check("the default replay video plays the window at x10",
          abs(total_ms - V.END_HOLD_MS - 1000.0 * span / V.DEFAULT_SPEED)
          <= video.get("frame_ms", 0),
          "%d ms for %.1f s" % (total_ms, span))
    check("manifest records the video", video.get("speed") == V.DEFAULT_SPEED
          and video.get("frames") == frames, str(video))

    fast = E.export(source, "smoke x20", crop, F.default_texts("x"), root=root,
                    video_speed=20)
    with open(os.path.join(fast, "export.yaml")) as handle:
        fast_video = yaml.safe_load(handle)["video"]
    _, fast_ms = gif_timing(os.path.join(fast, "smoke x20.gif"))
    preview_s, _ = V.video_timing(crop["t"][0], crop["t"][-1], 20)
    check("a chosen speed reaches the video and the manifest",
          fast_video.get("speed") == 20, str(fast_video))
    check("the duration preview is the file's length",
          abs(fast_ms / 1000.0 - preview_s) < 0.011, "%d ms vs %.2f s" % (fast_ms, preview_s))
    _, _, times = V.frame_timing(0.0, 1471.0)
    check("a long window gets longer frames, not more",
          len(times) <= V.MAX_FRAMES + 1, str(len(times)))

    check("manifest says where the legend came from",
          manifest["source"].get("legend") == "schema", str(manifest["source"].get("legend")))

    new_source = make_legend_log(source, workspace)
    new_run = pb.load(new_source)
    new_crop = pb.crop_run(new_run, t0 + 5.0, t0 + 0.5 * (t1 - t0))
    new_folder = E.export(new_source, "smoke legend", new_crop,
                          F.default_texts("x"), root=root)
    with open(new_source) as handle:
        new_source_lines = handle.read().splitlines()
    with open(os.path.join(new_folder, "smoke legend.csv")) as handle:
        new_export_lines = handle.read().splitlines()
    check("a new log's export copies its legend and header verbatim",
          new_export_lines[:3] == new_source_lines[:3])
    check("a new log's export rows are the old log's export rows",
          new_export_lines[3:] == exported_lines[3:])
    with open(os.path.join(new_folder, "export.yaml")) as handle:
        check("…and the manifest says the legend came from the source",
              yaml.safe_load(handle)["source"].get("legend") == "source")
    check("an export with a legend re-opens in the app",
          pb.load(os.path.join(new_folder, "smoke legend.csv"))["n_rows"] == new_crop["n_rows"])

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
    check("a simulated run gets a replay video",
          os.path.isfile(os.path.join(sim_folder, "smoke sim.gif"))
          and "file" in (sim_manifest.get("video") or {}), str(sim_manifest.get("video")))
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
    check("the summary table is filled", window.table.rowCount() == 22)

    from reviewer import replay_video as V
    from reviewer.app import ExportDialog, fmt_duration
    dialog = ExportDialog("dialog test", 0.0, 1000.0, parent=window)
    check("the export dialog starts at x10", dialog.speed() == V.DEFAULT_SPEED)
    dialog.speed_slider.setValue(V.MIN_SPEED)
    slow = dialog.preview_text()
    check("the preview quotes the writer's own duration",
          fmt_duration(V.video_timing(0.0, 1000.0, V.MIN_SPEED)[0]) in slow, slow)
    dialog.speed_slider.setValue(V.MAX_SPEED)
    check("the preview follows the speed",
          V.video_timing(0.0, 1000.0, V.MIN_SPEED)[0]
          > V.video_timing(0.0, 1000.0, V.MAX_SPEED)[0]
          and dialog.preview_text() != slow)
    check("the slider stops at x5 and x20",
          (dialog.speed_slider.minimum(), dialog.speed_slider.maximum()) == (5, 20))
    dialog.name_edit.setText("   ")
    check("a blank name cannot be exported", not dialog.export_button.isEnabled())
    dialog.deleteLater()

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

    # The pinger box: off for a log with no bag, on for one whose bag has it.
    check("the Pinger box is enabled exactly when the bag gave a pinger, and always explains",
          window.pinger_box.isEnabled() == (window.run["pinger"] is not None)
          and bool(window.pinger_box.toolTip()), window.run["pinger_note"])
    made = synthetic_pinger_log(logs, workspace)
    if made[0] is not None:
        window.open_log(made[0])
        app.processEvents()
        check("a bag with a pinger enables the box, unticked",
              window.pinger_box.isEnabled() and not window.pinger_box.isChecked())
        check("unticked: the crop carries no pinger", not window.crop["pinger_show"])
        window.pinger_box.setChecked(True)
        app.processEvents()
        check("ticking it puts the pinger on the map",
              window.crop["pinger_show"] and window.track._track_lines
              and any(line.get_label() == "pinger" for line in window.track._track_lines))
        window.play_button.setChecked(True)
        for _ in range(3):
            window._tick()
        window._playhead = window.crop["t"][-1] * 0.8
        window._show_playhead()
        check("the replay moves the pinger marker",
              len(window.track._replay) == 6
              and len(window.track._replay[5].get_xdata()) == 1)
        window.play_button.setChecked(False)
        from reviewer import export as E2
        from reviewer import poslog_bridge as pb
        folder = E2.export(window.source_csv, "smoke pinger", window.crop,
                           window.texts, root=os.path.join(workspace, "pexp"),
                           video_speed=20)
        import yaml as _yaml
        with open(os.path.join(folder, "export.yaml")) as handle:
            manifest = _yaml.safe_load(handle)
        check("the export manifest records the pinger source and filter",
              manifest.get("pinger", {}).get("csv_layout") == "pinger"
              and "filter" in manifest["pinger"])
        check("the exported CSV re-opens in the app as a pinger log",
              pb.load(os.path.join(folder, "smoke pinger.csv"))["layout"] == "pinger")
        window.pinger_box.setChecked(False)
        window.open_log(logs[0])
        app.processEvents()
        check("a new log resets the box", not window.pinger_box.isChecked())
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
