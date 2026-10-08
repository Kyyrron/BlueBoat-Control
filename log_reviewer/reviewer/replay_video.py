#!/usr/bin/env python3

r"""
The replay video: the track panel as framed in the app, played at x5 to x20.

An ANIMATED GIF, written with Pillow, because that is the one encoder every
interpreter this app runs under already has - matplotlib's ffmpeg writer needs
an ffmpeg binary, and this machine has none. A GIF of a static background with
a moving trail is also small, provided every frame after the first carries only
what moved: unchanged pixels are written transparent over the previous frame,
so the satellite imagery is paid for once.

It is drawn like the on-screen replay and with the same artists
(`figures.make_replay_artists`): the full tracks dimmed, the trails and dots
blitted over a background captured once. Tiles are drawn once, not per frame.

Two bounds keep it cheap:
  * the speed is exact - a frame advances `frame_ms / 1000 * speed` mission
    seconds - so the video's length always says how long the run was;
  * a long window does not get more frames past MAX_FRAMES, it gets LONGER
    frames (in 10 ms steps, the GIF's centisecond resolution). Pillow holds every
    frame in memory until the file is written, so the frame count is the memory
    bound; the speed stays exact either way.
"""

import math
import os

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from PIL import Image

from . import figures as F
from . import poslog_bridge as pb

DEFAULT_SPEED = 10
MIN_SPEED, MAX_SPEED = 5, 20        # what the export dialog offers
FRAME_MS = 100          # 10 fps unless the window is long
MAX_FRAMES = 600
END_HOLD_MS = 2000      # the finished track stays up before the loop restarts
FIGSIZE = (6.4, 5.6)    # the on-screen TrackCanvas's, so the layout matches
DPI = 90                # ~576 x 504 px: readable, deliberately not detailed
COLOURS = 128          # palette entries; index TRANSPARENT is reserved
TRANSPARENT = 255


def frame_timing(t0, t1, speed=DEFAULT_SPEED):
    """(frame_ms, mission seconds per frame, playhead times) for a window."""
    span = max(float(t1) - float(t0), 0.0)
    video_ms = span * 1000.0 / speed
    frame_ms = max(FRAME_MS, int(math.ceil(video_ms / MAX_FRAMES / 10.0)) * 10)
    step = frame_ms / 1000.0 * speed
    times = list(np.arange(float(t0), float(t1), step)) if step > 0 else []
    times.append(float(t1))               # always end on the finished track
    return frame_ms, step, times


def video_timing(t0, t1, speed=DEFAULT_SPEED):
    """(duration in seconds, frame_ms) of the GIF a window would give.

    The same arithmetic `write_replay_gif` writes, so the export dialog's
    preview is the file's length, not an estimate of it.
    """
    frame_ms, _step, times = frame_timing(t0, t1, speed)
    return ((len(times) - 1) * frame_ms + END_HOLD_MS) / 1000.0, frame_ms


def write_replay_gif(path, run, texts, tile_provider=None, track_limits=None,
                     speed=DEFAULT_SPEED):
    """Write the replay of `run` to `path`.

    Returns what the manifest records, or None when the selection has no
    drawable track (the PNG says so; a video of an empty panel says nothing).
    """
    t = run["t"]
    if not len(t):
        return None

    figure = Figure(figsize=FIGSIZE, dpi=DPI, facecolor=pb.SURFACE)
    canvas = FigureCanvasAgg(figure)
    ax = figure.add_subplot(111)
    figure.subplots_adjust(left=0.155, right=0.985, top=0.965, bottom=0.16)

    artists = F.plot_track(ax, run, texts, tile_provider, track_limits,
                           max(1.0, ax.bbox.width), show_headline=False)
    if not artists:
        return None
    for line in (artists.get("robot_line"), artists.get("target_line")):
        if line is not None:
            line.set_alpha(0.25)          # as on screen while replaying

    series = F.replay_series(run)
    replay = F.make_replay_artists(ax, visible=True)
    clock = ax.text(0.03, 0.965, "", transform=ax.transAxes, ha="left", va="top",
                    color=pb.INK, fontsize=9, zorder=10,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor=pb.SURFACE,
                              edgecolor=pb.GRID, alpha=0.9))
    moving = replay + [clock]
    for artist in moving:
        artist.set_animated(True)         # left out of the background draw

    canvas.draw()
    background = canvas.copy_from_bbox(figure.bbox)

    def render(when):
        canvas.restore_region(background)
        F.set_replay_frame(replay, series, when)
        clock.set_text("t %.0f s  \u00b7  \u00d7%d" % (when, speed))
        for artist in moving:
            ax.draw_artist(artist)
        return Image.frombuffer("RGBA", canvas.get_width_height(),
                                bytes(canvas.buffer_rgba()), "raw", "RGBA", 0, 1
                                ).convert("RGB")

    frame_ms, _step, times = frame_timing(t[0], t[-1], speed)
    # One palette for every frame, taken from the LAST one (it holds the full
    # trail), rendered first so each frame can be reduced as it is drawn
    # rather than all of them held in RGB. A shared palette also means no
    # colour flicker, and an unchanged pixel keeps the same index.
    palette_image = render(times[-1]).quantize(colors=COLOURS)
    palette = (palette_image.getpalette() or [])[:3 * COLOURS]
    palette += [0] * (3 * 256 - len(palette))

    # Every pixel equal to the previous frame's becomes TRANSPARENT, so a frame
    # carries only what moved. Without it Pillow re-encodes the whole box
    # between the clock and the boat - over satellite imagery that is most of
    # the panel, every frame (a 100 s field run came out at 14 MB).
    frames, previous = [], None
    for when in times:
        index = np.asarray(render(when).quantize(palette=palette_image,
                                                 dither=Image.Dither.NONE))
        delta = index.copy()
        if previous is not None:
            delta[index == previous] = TRANSPARENT
        previous = index
        frame = Image.fromarray(delta, mode="P")
        frame.putpalette(palette)
        frames.append(frame)

    durations = [frame_ms] * (len(frames) - 1) + [END_HOLD_MS]
    # disposal=1: each frame is drawn over the last, which is what makes
    # "transparent" mean "unchanged".
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=durations, loop=0, optimize=False, disposal=1,
                   transparency=TRANSPARENT)

    return {"file": os.path.basename(path), "speed": int(speed),
            "frame_ms": frame_ms, "frames": len(frames),
            "duration_s": round(sum(durations) / 1000.0, 2)}
