#!/usr/bin/env python3

r"""
The linked cursor: one instant, marked on every panel at once.

Hovering any plot answers "what was happening at this moment?" everywhere -
a dot rides the curve under the pointer, and the same instant is marked on the
other three panels, including the boat's and the target's positions on the
track. The panels are drawn against a shared mission-time axis precisely so
they can be read against one another; this makes that reading a gesture
instead of an eye-measurement.

BLITTING, AND WHY THERE IS AN `Overlay` AT ALL. A mouse move must not redraw a
panel - on the track that would mean re-drawing up to 64 satellite tiles per
motion event. So every cursor artist is `animated=True`, which excludes it from
normal draws, the clean background is captured once per real draw, and a move
restores that background and draws only the handful of markers on top.

One canvas must have exactly ONE overlay, though. Two independent blitters on
the same canvas fight: each restores a background captured without the other's
artists, so whichever blits second erases the first. That is why the track's
replay markers and its hover dot share the canvas's single `Overlay` rather
than owning one each.
"""

import numpy as np
from matplotlib.lines import Line2D

from . import poslog_bridge as pb


class Overlay:
    """Every animated artist on one canvas, and the blitting that draws them."""

    def __init__(self, canvas):
        self.canvas = canvas
        self.artists = []
        self._background = None
        canvas.mpl_connect("draw_event", self._on_draw)

    def add(self, artist):
        artist.set_animated(True)
        self.artists.append(artist)
        return artist

    def discard(self, artist):
        if artist in self.artists:
            self.artists.remove(artist)
        try:
            artist.remove()
        except (ValueError, NotImplementedError, AttributeError):
            pass

    def invalidate(self):
        """The figure changed underneath us; the next flush re-captures it."""
        self._background = None

    def _on_draw(self, _event):
        # Animated artists are absent from this draw, so the capture is clean.
        self._background = self.canvas.copy_from_bbox(self.canvas.figure.bbox)

    def flush(self):
        if self._background is None:
            self.canvas.draw()             # _on_draw captures, artists follow
            return
        self.canvas.restore_region(self._background)
        for artist in self.artists:
            if artist.get_visible() and artist.axes is not None:
                artist.axes.draw_artist(artist)
        self.canvas.blit(self.canvas.figure.bbox)


class LinkedCursor:
    """The dot (and optional time rule) marking one instant on one axes.

    `set_data` takes the mission-time base and one (x, y, colour) track per dot
    - for a time-series panel that is (t, values); for the track panel it is
    (lon, lat), which is why the dot is positioned by INDEX rather than by the
    x coordinate. One mechanism, both kinds of panel.
    """

    def __init__(self, canvas, ax, overlay, with_rule=True):
        self.canvas = canvas
        self.ax = ax
        self.overlay = overlay
        self.with_rule = with_rule
        self.rule = None
        self.dots = []
        self._t = None
        self._tracks = []

    def set_data(self, t, tracks):
        """Rebuild the artists. Call after every re-plot: `ax.clear()` has
        thrown the old ones away, and an artist on a cleared axes draws
        nothing while still looking alive."""
        for artist in self.dots + ([self.rule] if self.rule else []):
            self.overlay.discard(artist)
        self.dots, self.rule = [], None

        self._t = np.asarray(t)
        self._tracks = [(np.asarray(x), np.asarray(y)) for x, y, _c, _s in tracks]

        if self.with_rule:
            rule = Line2D([], [], color=pb.INK2, linewidth=0.9, alpha=0.55,
                          zorder=11, visible=False)
            self.ax.add_line(rule)
            self.rule = self.overlay.add(rule)

        for _x, _y, colour, size in tracks:
            dot = Line2D([], [], marker="o", markersize=size, color=colour,
                         markeredgecolor=pb.SURFACE, markeredgewidth=1.6,
                         linestyle="none", zorder=12, visible=False)
            self.ax.add_line(dot)
            self.dots.append(self.overlay.add(dot))

    def index_at(self, when):
        """The nearest logged row to a mission time, or None if out of range."""
        t = self._t
        if t is None or not len(t) or when is None:
            return None
        if when < t[0] - 1e-9 or when > t[-1] + 1e-9:
            return None
        after = int(np.searchsorted(t, when))
        if after <= 0:
            return 0
        if after >= len(t):
            return len(t) - 1
        return after if (t[after] - when) < (when - t[after - 1]) else after - 1

    def set_time(self, when, flush=True):
        """Mark this instant, or hide the cursor when `when` is None."""
        index = self.index_at(when)
        if index is None:
            self.hide(flush=flush)
            return None
        for dot, (xs, ys) in zip(self.dots, self._tracks):
            x, y = xs[index], ys[index]
            # A NaN is a real answer - no fix, or a sample excluded as a pose
            # jump. The dot vanishes there rather than lying about a position.
            visible = bool(np.isfinite(x) and np.isfinite(y))
            dot.set_data([x] if visible else [], [y] if visible else [])
            dot.set_visible(visible)
        if self.rule is not None:
            low, high = self.ax.get_ylim()
            self.rule.set_data([self._t[index]] * 2, [low, high])
            self.rule.set_visible(True)
        if flush:
            self.overlay.flush()
        return index

    def hide(self, flush=True):
        for artist in self.dots + ([self.rule] if self.rule else []):
            artist.set_visible(False)
        if flush:
            self.overlay.flush()
