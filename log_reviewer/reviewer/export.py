#!/usr/bin/env python3

r"""
Export: the selected window of a run, filed under `Processed_Robot_data/`.

THE WRITE-ONCE BOUNDARY (superproject CM-7 / BlueBoat-Control N7). The poslog
CSV, its `-origin.yaml` sidecar and the per-run PNG in `data/Robot_data/` are
PRIMARY FIELD RECORD. This app opens them read-only and never renames, moves,
rewrites or deletes one - not even the file the operator has just renamed in
the toolbar, which renames only the EXPORT. Everything written goes into

    ~/ros2_ws/data/Processed_Robot_data/<name>/
        <name>.csv            the rows inside the timeline, every column, verbatim
        <name>.png            the report as framed in the app
        <name>-origin.yaml    a COPY of the run's origin sidecar
        export.yaml           what was cropped, from what, with which text

which is derived data: re-exporting the same window is expected, and
overwriting an export is the operator's call (the app asks first).

The cropped CSV is copied out of the source FIELD BY FIELD, never re-formatted
from the parsed floats. `1.8421408096936085` must land in the export as those
exact digits: a round-trip through `float()` and `"%f"` would quietly re-write
primary data at reduced precision.
"""

import csv
import datetime as dt
import os
import re
import shutil

import yaml
from matplotlib.backends.backend_agg import FigureCanvasAgg

from . import figures as F
from . import poslog_bridge as pb

DEFAULT_ROOT = os.path.join(os.path.expanduser("~"), "ros2_ws", "data",
                            "Processed_Robot_data")
SCHEMA = "blueboat_processed_log/1"


def plain(value):
    """Strip numpy types out of anything bound for YAML.

    PyYAML's SafeDumper represents Python scalars only: handed an
    `np.float64` it raises `RepresenterError: cannot represent an object` and
    the export dies AFTER the CSV and the picture are already on disk. numpy
    leaks in easily and invisibly - `ax.get_xlim()` returns numpy scalars, and
    `round()` on one gives a numpy scalar back, so the value looks like a float
    everywhere except at the dumper. Sanitising the whole manifest once is the
    only version of this that stays fixed; converting the fields known to be
    numpy today just moves the next occurrence.
    """
    if isinstance(value, dict):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if value is None or isinstance(value, str):
        return value
    if hasattr(value, "item"):          # any numpy scalar
        value = value.item()
    # bool BEFORE int: bool is a subclass of int, and np.bool_ is not a
    # subclass of bool, so an unordered check writes `true` out as `1`.
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, float):
        return float(value)
    if isinstance(value, int):
        return int(value)
    return value


class ExportExists(Exception):
    """The destination folder is already there and overwrite was not asked for."""


def safe_name(name, fallback="log"):
    """A folder name from operator text.

    Only path-dangerous characters are touched - spaces, dashes and dots are
    kept, because an operator who types 'Bath2 run 3' should get a folder
    called exactly that.
    """
    name = (name or "").strip().strip(".")
    name = re.sub(r"[/\\\x00]+", "_", name)
    return name or fallback


def export(source_csv, name, crop, texts, root=None, tile_provider=None,
           track_limits=None, overwrite=False):
    """Write the four files. Returns the folder path.

    `root` resolves at CALL time, not at import time. A default argument of
    `DEFAULT_ROOT` binds the value once, so anything that redirects the root
    afterwards (a test, a future setting) would have the caller's
    already-exists check look at one folder while the write went to another -
    and the overwrite prompt would then guard the wrong directory.
    """
    root = root or DEFAULT_ROOT
    source_csv = os.path.abspath(os.path.expanduser(str(source_csv)))
    name = safe_name(name, os.path.basename(source_csv)[:-4])
    folder = os.path.join(os.path.expanduser(root), name)

    if os.path.isdir(folder) and not overwrite:
        raise ExportExists(folder)
    os.makedirs(folder, exist_ok=True)

    csv_out = os.path.join(folder, name + ".csv")
    png_out = os.path.join(folder, name + ".png")
    written_rows = write_cropped_csv(source_csv, csv_out, crop["row_index"])

    figure = F.build_report_figure(crop, texts, tile_provider, track_limits)
    FigureCanvasAgg(figure)
    figure.savefig(png_out, bbox_inches="tight", pad_inches=0.28)

    sidecar_out = None
    sidecar_in = pb.sidecar_path(source_csv)
    if not os.path.isfile(sidecar_in):
        sidecar_in = source_csv[:-4] + "-origin.yaml"
    if os.path.isfile(sidecar_in):
        sidecar_out = os.path.join(folder, name + "-origin.yaml")
        shutil.copy2(sidecar_in, sidecar_out)      # copied, never moved

    write_manifest(os.path.join(folder, "export.yaml"), source_csv, sidecar_in,
                   crop, texts, written_rows, track_limits, name)
    return folder


def write_cropped_csv(source_csv, out_csv, row_index):
    """Copy the header and the selected data rows, field for field.

    `row_index` counts DATA rows, ignoring the header - the same numbering
    `poslog_report.read_poslog` produces, which is what the crop carries.
    """
    wanted = set(int(i) for i in row_index)
    written = 0
    with open(source_csv, newline="") as source, \
            open(out_csv, "w", newline="") as target:
        reader = csv.reader(source)
        writer = csv.writer(target, lineterminator="\n")
        try:
            writer.writerow(next(reader))
        except StopIteration:
            return 0
        for number, row in enumerate(reader):
            if number in wanted:
                writer.writerow(row)
                written += 1
    return written


def write_manifest(path, source_csv, sidecar_in, crop, texts, written_rows,
                   track_limits, name):
    """Everything needed to say what this export is and redo it."""
    t = crop["t"]
    rows = crop["row_index"]
    window = crop.get("window", (float(t[0]), float(t[-1])) if len(t) else (0.0, 0.0))
    manifest = {
        "schema": SCHEMA,
        "name": name,
        "exported_utc": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "source": {
            "csv": source_csv,
            "stem": crop["stem"],
            "origin_sidecar": sidecar_in if os.path.isfile(sidecar_in) else None,
            "layout": crop["layout"],
            "target": crop["spec"]["target_name"],
            "rows": int(crop.get("n_rows_full", crop["n_rows"])),
        },
        "crop": {
            "t_start_s": round(float(window[0]), 3),
            "t_end_s": round(float(window[1]), 3),
            "first_t_s": round(float(t[0]), 3) if len(t) else None,
            "last_t_s": round(float(t[-1]), 3) if len(t) else None,
            "first_source_row": int(rows[0]) if len(rows) else None,
            "last_source_row": int(rows[-1]) if len(rows) else None,
            "rows": int(written_rows),
            "wall_clock_start": pb.wall_clock(crop, 0) if len(t) else None,
            "wall_clock_end": pb.wall_clock(crop, len(t) - 1) if len(t) else None,
        },
        "track_view": ({"framed": "manual",
                        "lon_min": round(float(track_limits[0]), 8),
                        "lon_max": round(float(track_limits[1]), 8),
                        "lat_min": round(float(track_limits[2]), 8),
                        "lat_max": round(float(track_limits[3]), 8)}
                       if track_limits else {"framed": "fit to selection"}),
        "texts": dict(texts),
    }
    with open(path, "w") as handle:
        yaml.safe_dump(plain(manifest), handle, sort_keys=False,
                       allow_unicode=True, default_flow_style=False)
