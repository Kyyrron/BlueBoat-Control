#!/usr/bin/env python3

r"""
The panels. Adapted from `poslog_report`'s renderers, NOT imported from them.

The maths is reused (see poslog_bridge); the painting is the app's own, because
four things differ from the archived PNG and every one of them is a deliberate
divergence:

  * every panel's title and description are OPERATOR TEXT, not literals;
  * the track carries satellite imagery and must survive pan and zoom;
  * there is no actuation-state strip (the numbers stay in the summary);
  * the summary lays 21 entries into 7 rows x 3 pairs. The PNG's `per_col = 6`
    against 3 columns is 18 slots for 19 entries, so its last row - the world
    origin - is silently dropped. Do not copy that back.

No pyplot anywhere. Figures are built as `matplotlib.figure.Figure` and handed
to whichever canvas wants them (Qt on screen, Agg for the export), so the app
never touches the global backend and can never collide with `poslog_report`'s
own `matplotlib.use("Agg")`.
"""

import math
import textwrap

import matplotlib
import matplotlib.patheffects as patheffects
import numpy as np
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec

from . import poslog_bridge as pb

SURFACE, INK, INK2, MUTED, GRID = pb.SURFACE, pb.INK, pb.INK2, pb.MUTED, pb.GRID
ROBOT, TARGET, REF = pb.ROBOT, pb.TARGET, pb.REF
EARTH_R = pb.EARTH_R

matplotlib.rcParams.update({
    "figure.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.titlelocation": "left",
    "axes.titlesize": 10,
    "axes.titlecolor": INK,
    "axes.labelsize": 9,
    "axes.labelcolor": INK2,
    "font.size": 9,
    "legend.frameon": False,
    "legend.fontsize": 8,
})

# Defaults for every editable string. The speed panel is the one that changed
# wording: the PNG's title read "Ground speed (|v|, 1 s smoothed, differenced
# from pose)", which is a definition wearing a title's clothes. The title now
# names the quantity and the description says, in words, what was done to it.
DEFAULT_TEXTS = {
    "report_title": "",
    "track_title": "Track (WGS84) — robot and target",
    "track_desc": "Where the boat went and where it was being sent, as recorded "
                  "by GPS. Rows with no fix are left out.",
    "distance_title": "Distance robot → target",
    "distance_desc": "How far the boat was from the point it was being commanded to.",
    "speed_title": "Ground speed",
    "speed_desc": "How fast the boat was moving over the ground, worked out from how "
                  "far it travelled between one log row and the next, then averaged "
                  "across a second so the 3 Hz logging does not make it jitter.",
    "thrust_title": "Commanded thrust",
    "thrust_desc": "What the controller asked each thruster for, in Newtons, as "
                   "published on /thruster_input.",
    "summary_title": "Summary",
}

WRAP_CHARS = 95

# Anything drawn INSIDE the track axes may land on dark satellite imagery, so
# it carries a thin halo in the surface colour. Without it the start/end labels
# and the scale bar vanish over deep water.
HALO = [patheffects.withStroke(linewidth=2.2, foreground=SURFACE)]


def default_texts(stem="", run=None):
    """The starting text for a run. Frame-dependent, because the default track
    blurb must not promise GPS on a panel drawn in metres."""
    texts = dict(DEFAULT_TEXTS)
    texts["report_title"] = stem
    if run is None:
        return texts
    series = pb.track_series(run)
    if series["mode"] == "world":
        texts["track_title"] = series["title"]
        texts["track_desc"] = (
            "Where the boat went and where it was being sent, in metres east "
            "and north of the world origin. A simulated run is plotted in its "
            "own frame, not in GPS degrees."
            if series["simulated"] else
            "Where the boat went and where it was being sent, in metres east "
            "and north of the launch point. This run recorded no GPS fix, so "
            "the track is shown in the boat's own frame.")
    return texts


# ---------------------------------------------------------------------------
# Shared styling
# ---------------------------------------------------------------------------

def style(ax, grid="both"):
    """`poslog_report._style`, unchanged."""
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8, length=3, color=GRID)
    ax.grid(True, axis=grid if grid != "both" else "both",
            color=GRID, linewidth=0.7)
    ax.set_axisbelow(True)


def headline(ax, title, description, wrap=WRAP_CHARS, show=True):
    """Title above, operator description between it and the axes.

    The description is wrapped rather than clipped, and the title's pad grows
    with it, so a long sentence pushes the title up instead of landing on it.

    `show=False` draws neither. On screen the title and the description ARE the
    text entries above the canvas - drawing them inside it as well would print
    every heading twice and spend a third of a small panel doing it. They are
    drawn only into the exported picture, which has no entries of its own.
    """
    if not show:
        ax.set_title("")
        return
    lines = textwrap.wrap(description.strip(), wrap) if description.strip() else []
    ax.set_title(title, pad=8 + 11 * len(lines))
    if lines:
        ax.text(0.0, 1.008, "\n".join(lines), transform=ax.transAxes,
                ha="left", va="bottom", fontsize=8, color=MUTED,
                linespacing=1.35)


# ---------------------------------------------------------------------------
# Track
# ---------------------------------------------------------------------------

def track_extent(run, margin=0.08):
    """The box enclosing both tracks, padded, in whatever frame they are in.

    Returns None when the run has nothing drawable at all.
    """
    series = pb.track_series(run)
    fix, tfix = series["ok"], series["tok"]
    if not np.any(fix):
        return None
    lons = [series["x"][fix]]
    lats = [series["y"][fix]]
    if np.any(tfix):
        lons.append(series["tx"][tfix])
        lats.append(series["ty"][tfix])
    lon_all, lat_all = np.concatenate(lons), np.concatenate(lats)
    lon_lo, lon_hi = float(np.min(lon_all)), float(np.max(lon_all))
    lat_lo, lat_hi = float(np.min(lat_all)), float(np.max(lat_all))
    # A stationary run has zero span; give it something to draw inside. The
    # floor is frame-dependent: 2e-5 deg is about 2 m, so metres need metres.
    floor = 2.0 if series["mode"] == "world" else 2e-5
    span = max(lon_hi - lon_lo, lat_hi - lat_lo, floor)
    pad = span * margin
    cx, cy = 0.5 * (lon_lo + lon_hi), 0.5 * (lat_lo + lat_hi)
    half = 0.5 * span + pad
    return (cx - half, cx + half, cy - half, cy + half)


def plot_track(ax, run, texts, tile_provider=None, limits=None, canvas_px=900, show_headline=True):
    """The track panel. Returns the artists the replay animates.

    The frame comes from `poslog_bridge.track_series`, shared with the archived
    PNG: WGS84 degrees for a real run that has fixes, local ENU metres for a
    simulation (even a GPS-anchored one, whose fixes are synthesised from sim
    odom) and for a real run that lost its fix. Satellite tiles are georeferenced
    in degrees, so they are drawn in WGS84 mode only.
    """
    ax.clear()
    style(ax)
    headline(ax, texts["track_title"], texts["track_desc"], show=show_headline)

    series = pb.track_series(run)
    world = series["mode"] == "world"
    ax.set_xlabel(series["xlabel"])
    ax.set_ylabel(series["ylabel"])

    x, y = series["x"], series["y"]
    tx, ty = series["tx"], series["ty"]
    fix, tfix = series["ok"], series["tok"]

    if not np.any(fix):
        ax.text(0.5, 0.5, "no usable track in this selection",
                transform=ax.transAxes, ha="center", va="center",
                color=MUTED, fontsize=11)
        ax.set_xticks([])
        ax.set_yticks([])
        return {}

    box = limits or track_extent(run)
    lat0 = 0.0 if world else float(np.mean(y[fix]))

    # True metric aspect either way: in metres that is simply 1, in degrees it
    # is the longitude foreshortening, so the shape on screen is the shape on
    # the water at every zoom.
    ax.set_aspect(1.0 if world else 1.0 / max(math.cos(math.radians(lat0)), 1e-6))
    ax.set_xlim(box[0], box[1])
    ax.set_ylim(box[2], box[3])

    if tile_provider is not None and not world:
        draw_tiles(ax, tile_provider, canvas_px)

    target_line = None
    if np.any(tfix):
        target_line, = ax.plot(tx[tfix], ty[tfix], "-", color=TARGET,
                               linewidth=2.0, label="target", zorder=2)
    robot_line, = ax.plot(x[fix], y[fix], "-", color=ROBOT, linewidth=2.0,
                          label="robot", zorder=3)

    # Start / end, ringed in the surface colour so they stay legible over the line.
    ax.plot(x[fix][0], y[fix][0], "o", markersize=9, color=ROBOT,
            markeredgecolor=SURFACE, markeredgewidth=2.0, zorder=4)
    ax.plot(x[fix][-1], y[fix][-1], "s", markersize=9, color=ROBOT,
            markeredgecolor=SURFACE, markeredgewidth=2.0, zorder=4)
    for label, index in (("start", 0), ("end", -1)):
        ax.annotate(label, (x[fix][index], y[fix][index]),
                    textcoords="offset points", xytext=(9, 5), color=INK2,
                    fontsize=8, path_effects=HALO, zorder=5)

    ax.legend(loc="best")
    if not world:
        ax.ticklabel_format(useOffset=False, style="plain")
        ax.tick_params(axis="x", labelrotation=20)
    decorate_track(ax, lat0, world)

    return {"robot_line": robot_line, "target_line": target_line,
            "lat0": lat0, "fix": fix, "tfix": tfix, "world": world}


def draw_tiles(ax, provider, canvas_px):
    """Satellite imagery under the current view. Silent when nothing is cached."""
    for image in list(getattr(ax, "_tile_artists", [])):
        try:
            image.remove()
        except (ValueError, NotImplementedError):
            pass
    ax._tile_artists = []

    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    for extent, image in provider.tiles_for(x0, x1, y0, y1, canvas_px):
        artist = ax.imshow(image, extent=extent, origin="upper", zorder=0,
                           interpolation="bilinear")
        # imshow forces its own aspect and limits; the track owns both.
        artist.set_clip_on(True)
        ax._tile_artists.append(artist)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)


def decorate_track(ax, lat0, world=False):
    """Scale bar and north arrow, RE-DRAWN for the current limits.

    In metres the ticks ARE the scale, so only the north arrow is drawn.

    The PNG computes its bar once from `get_xlim()` at render time, which is
    fine for a static picture and wrong the moment anybody zooms - the bar
    would keep its old length and its old label. Every limit change calls this.
    """
    for artist in list(getattr(ax, "_decor_artists", [])):
        try:
            artist.remove()
        except (ValueError, NotImplementedError):
            pass
    ax._decor_artists = []

    arrow = ax.annotate("N", xy=(0.965, 0.93), xytext=(0.965, 0.80),
                        xycoords="axes fraction", textcoords="axes fraction",
                        ha="center", va="bottom", color=INK2, fontsize=9,
                        arrowprops=dict(arrowstyle="-|>", color=INK2, linewidth=1.4))
    ax._decor_artists = [arrow]
    if world:
        return

    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    span_m = abs(x1 - x0) * math.radians(1.0) * EARTH_R * math.cos(math.radians(lat0))
    if span_m <= 0 or not math.isfinite(span_m):
        return

    nice = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000]
    length_m = min(nice, key=lambda v: abs(v - span_m * 0.25))
    length_deg = length_m / (math.radians(1.0) * EARTH_R * math.cos(math.radians(lat0)))

    bx = x0 + 0.06 * (x1 - x0)
    by = y0 + 0.07 * (y1 - y0)
    bar, = ax.plot([bx, bx + length_deg], [by, by], "-", color=INK2,
                   linewidth=2.5, solid_capstyle="butt", zorder=5,
                   path_effects=HALO)
    label = ax.text(bx + length_deg / 2, by + 0.018 * (y1 - y0), "%d m" % length_m,
                    ha="center", va="bottom", color=INK2, fontsize=8, zorder=5,
                    path_effects=HALO)
    ax._decor_artists = [arrow, bar, label]


# ---------------------------------------------------------------------------
# Time-series panels
# ---------------------------------------------------------------------------

def plot_distance(ax, run, metrics, texts, dist=None, have=None, show_headline=True):
    ax.clear()
    style(ax)
    headline(ax, texts["distance_title"], texts["distance_desc"], show=show_headline)
    ax.set_xlabel("mission time (s)")
    ax.set_ylabel("distance (m)")
    t = run["t"]
    if dist is None:
        dist, have = pb.distance_series(run)
    if not np.any(have):
        ax.text(0.5, 0.5, "no target in this selection", transform=ax.transAxes,
                ha="center", va="center", color=MUTED)
        return
    ax.plot(t, np.where(have, dist, np.nan), "-", color=TARGET, linewidth=2.0)
    _span(ax, t)
    if math.isfinite(metrics["dist_mean"]):
        ax.axhline(metrics["dist_mean"], color=REF, linewidth=1.2, linestyle="--")
        ax.annotate("mean %.2f m" % metrics["dist_mean"],
                    xy=(t[-1], metrics["dist_mean"]), textcoords="offset points",
                    xytext=(-4, 5), ha="right", color=MUTED, fontsize=8)
    ax.set_ylim(bottom=0)


def plot_speed(ax, run, metrics, texts, speed=None, jumps=None, show_headline=True):
    ax.clear()
    style(ax)
    headline(ax, texts["speed_title"], texts["speed_desc"], show=show_headline)
    ax.set_xlabel("mission time (s)")
    ax.set_ylabel("speed (m/s)")
    t = run["t"]
    if speed is None:
        speed, jumps = pb.speed_series(run)
    ax.plot(t, np.where(jumps, np.nan, speed), "-", color=ROBOT, linewidth=2.0)
    if np.any(jumps):
        # Named, not hidden: the reader must see that samples were removed.
        ax.plot(t[jumps], np.zeros(int(np.sum(jumps))), "|", color=pb.BAD,
                markersize=9, markeredgewidth=1.2, zorder=4,
                label="pose jump (>%.0f m/s), excluded" % pb.MAX_PLAUSIBLE_SPEED_MS)
        ax.legend(loc="upper left")
    _span(ax, t)
    if math.isfinite(metrics["speed_mean"]):
        ax.axhline(metrics["speed_mean"], color=REF, linewidth=1.2, linestyle="--")
        ax.annotate("mean %.2f m/s" % metrics["speed_mean"],
                    xy=(t[-1], metrics["speed_mean"]), textcoords="offset points",
                    xytext=(-4, 5), ha="right", color=MUTED, fontsize=8)
    ax.set_ylim(bottom=0)


def plot_thrusters(ax, run, texts, show_headline=True):
    ax.clear()
    style(ax)
    headline(ax, texts["thrust_title"], texts["thrust_desc"], show=show_headline)
    ax.set_ylabel("thrust (N)")
    # The x label lives here now; in the PNG it belonged to the actuation strip
    # underneath, which this app does not draw.
    ax.set_xlabel("mission time (s)")
    t, d = run["t"], run["data"]
    ax.plot(t, d["right_thr_in"], "-", color=ROBOT, linewidth=1.8, label="right")
    ax.plot(t, d["left_thr_in"], "-", color=TARGET, linewidth=1.8, label="left")
    ax.axhline(0.0, color=GRID, linewidth=1.0)
    _span(ax, t)
    ax.margins(y=0.12)
    ax.legend(loc="lower right", ncol=2)


def _span(ax, t):
    """Every time axis spans the whole SELECTION, so the panels read against
    one another rather than each against its own window."""
    if len(t) > 1 and t[-1] > t[0]:
        ax.set_xlim(t[0], t[-1])


# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------

def plot_table(ax, rows, title="Summary", per_col=7):
    """21 entries as 7 rows of 3 label/value pairs, filled down then across.

    `per_col` x 3 must be >= len(rows) or entries fall off the bottom - which
    is exactly the bug in the PNG this panel is adapted from.
    """
    ax.clear()
    ax.axis("off")
    per_col = max(per_col, -(-len(rows) // 3))

    cell_text = []
    for r in range(per_col):
        line = []
        for c in range(3):
            index = c * per_col + r
            line.extend(rows[index] if index < len(rows) else ("", ""))
        cell_text.append(line)

    table = ax.table(cellText=cell_text, cellLoc="left", loc="upper center",
                     colWidths=[0.135, 0.1975] * 3)
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.55)
    for (_row, col), cell in table.get_celld().items():
        cell.set_edgecolor(GRID)
        cell.set_linewidth(0.6)
        cell.set_facecolor(SURFACE)
        text = cell.get_text()
        if col % 2 == 0:
            text.set_color(MUTED)
        else:
            text.set_color(INK)
            text.set_fontweight("bold")
        if not text.get_text():
            cell.set_edgecolor(SURFACE)

    ax.set_title(title, loc="left", color=INK, fontsize=10, pad=10)


# ---------------------------------------------------------------------------
# The whole report, for the export
# ---------------------------------------------------------------------------

def build_report_figure(run, texts, tile_provider=None, track_limits=None,
                        figsize=(16.0, 12.0), dpi=130):
    """The exported picture: the PNG's layout minus the actuation strip.

    Built on a bare `Figure`, so the caller attaches whatever canvas it wants
    and nothing here touches the global matplotlib backend.
    """
    metrics = pb.compute_metrics(run)
    dist, have = pb.distance_series(run)
    speed, jumps = pb.speed_series(run)

    fig = Figure(figsize=figsize, dpi=dpi, facecolor=SURFACE)
    gs = GridSpec(4, 4, figure=fig,
                  height_ratios=[2.3, 2.3, 1.5, 1.75],
                  hspace=0.75, wspace=0.32,
                  left=0.055, right=0.975, top=0.880, bottom=0.045)

    ax_track = fig.add_subplot(gs[0:2, 0:2])
    ax_dist = fig.add_subplot(gs[0, 2:4])
    ax_speed = fig.add_subplot(gs[1, 2:4])
    ax_thr = fig.add_subplot(gs[2, 0:4])
    ax_table = fig.add_subplot(gs[3, 0:4])

    track_px = figsize[0] * dpi * 0.45
    plot_track(ax_track, run, texts, tile_provider, track_limits, track_px)
    plot_distance(ax_dist, run, metrics, texts, dist, have)
    plot_speed(ax_speed, run, metrics, texts, speed, jumps)
    plot_thrusters(ax_thr, run, texts)
    plot_table(ax_table, pb.summary_rows(run, metrics), texts["summary_title"])

    fig.suptitle(texts["report_title"] or run["stem"], x=0.055, y=0.972,
                 ha="left", fontsize=15, color=INK, fontweight="bold")
    fig.text(0.055, 0.945, pb.subtitle_for(run, metrics), ha="left",
             fontsize=9.5, color=MUTED)
    return fig
