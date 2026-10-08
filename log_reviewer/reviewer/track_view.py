#!/usr/bin/env python3

r"""
The interactive track: zoom, pan, satellite imagery and replay.

Zoom and pan are implemented directly on the axes limits rather than through
matplotlib's navigation toolbar, for two reasons: the panel's metric aspect
lock (one metre east = one metre north, at every zoom) has to survive the
interaction, and every limit change has to re-request tiles and re-draw the
scale bar - which the toolbar knows nothing about.

REPLAY AND THE HOVER CURSOR ARE BLITTED, THROUGH ONE SHARED OVERLAY. Redrawing
the whole figure 30 times a second - or on every mouse move - would re-draw up
to 64 satellite images each time. Instead the markers are `animated=True`, the
clean background is captured once per real draw, and a frame restores it and
draws only the markers. The replay and the cursor deliberately share this
canvas's SINGLE `Overlay`: two independent blitters on one canvas erase each
other (see cursor.py).
"""

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PySide6.QtCore import QTimer, Signal

from . import figures as F
from . import poslog_bridge as pb
from .cursor import LinkedCursor, Overlay

ZOOM_STEP = 1.3
TILE_REFRESH_MS = 90


class TrackCanvas(FigureCanvasQTAgg):
    """The track panel as a live widget."""

    viewChanged = Signal()
    hoverTime = Signal(object)          # mission seconds, or None on leaving

    def __init__(self, parent=None):
        figure = Figure(figsize=(6.4, 5.6), dpi=100, facecolor=pb.SURFACE)
        super().__init__(figure)
        self.setParent(parent)
        self.ax = figure.add_subplot(111)
        figure.subplots_adjust(left=0.155, right=0.985, top=0.965, bottom=0.16)

        self.run = None
        self.texts = F.default_texts()
        self.provider = None
        self.tiles_enabled = True
        self.user_limits = None       # set once the operator frames it themselves

        self._lat0 = 0.0
        self._world = False
        self._replay = []
        self._replay_on = False
        self._track_lines = []
        self._pan = None
        self._series = None

        self.overlay = Overlay(self)
        self.cursor = LinkedCursor(self, self.ax, self.overlay, with_rule=False)

        self._tile_timer = QTimer(self)
        self._tile_timer.setSingleShot(True)
        self._tile_timer.setInterval(TILE_REFRESH_MS)
        self._tile_timer.timeout.connect(self._refresh_tiles)

        self.mpl_connect("scroll_event", self._on_scroll)
        self.mpl_connect("button_press_event", self._on_press)
        self.mpl_connect("motion_notify_event", self._on_motion)
        self.mpl_connect("button_release_event", self._on_release)
        self.mpl_connect("axes_leave_event", self._on_leave)
        self.mpl_connect("figure_leave_event", self._on_leave)

    # -- content ----------------------------------------------------------

    def set_provider(self, provider):
        self.provider = provider

    def render(self, run, texts, keep_view=True):
        """Draw a (possibly cropped) run. Keeps the operator's framing."""
        self.run = run
        self.texts = texts
        limits = self.user_limits if (keep_view and self.user_limits) else None
        artists = F.plot_track(self.ax, run, texts,
                               self.provider if self.tiles_enabled else None,
                               limits, self._canvas_px(), show_headline=False)
        self._lat0 = artists.get("lat0", 0.0)
        self._world = bool(artists.get("world", False))
        self._track_lines = [a for a in (artists.get("robot_line"),
                                         artists.get("target_line")) if a]
        self._series = F.replay_series(run)
        self._build_replay_artists()
        # ax.clear() threw the old cursor artists away; rebuild them onto the
        # fresh axes, robot first so it draws over the target.
        # Deliberately smaller than the replay markers (9 and 11): on the
        # track both are round dots in the same two colours, so size is the
        # only thing telling "where the playback is" from "where the pointer
        # is asking about". On the time-series panels there is no such clash.
        self.cursor.set_data(run["t"], [
            (self._series["tlon"], self._series["tlat"], pb.TARGET, 5),
            (self._series["lon"], self._series["lat"], pb.ROBOT, 6)])
        self.overlay.invalidate()
        self.draw_idle()

    def _canvas_px(self):
        return max(1.0, self.ax.bbox.width)

    def _build_replay_artists(self):
        for artist in self._replay:
            self.overlay.discard(artist)
        self._replay = F.make_replay_artists(self.ax)
        for artist in self._replay:
            self.overlay.add(artist)

    # -- replay -----------------------------------------------------------

    def set_replay_active(self, active):
        """Dim the full tracks while replaying, so the trail reads as the front."""
        self._replay_on = bool(active)
        for line in self._track_lines:
            line.set_alpha(0.25 if active else 1.0)
        for artist in self._replay:
            artist.set_visible(bool(active))
        self.overlay.invalidate()
        self.draw_idle()

    def set_playhead(self, t):
        """Advance the trail to mission time `t`. Cheap: blitted, four artists."""
        if not self._replay_on or self._series is None:
            return
        F.set_replay_frame(self._replay, self._series, t)
        self.overlay.flush()

    # -- the linked cursor ------------------------------------------------

    def set_hover_time(self, when):
        """Mark an instant sent by another panel."""
        self.cursor.set_time(when)

    def _hover_time_at(self, event):
        """The mission time of the logged position nearest the pointer.

        Measured in metres, not degrees: a degree of longitude is shorter than
        a degree of latitude, so an un-scaled distance would snap east-west
        sooner than north-south. In the world frame both axes are already
        metres, so the correction is 1. Beyond a tolerance the pointer is over
        open water rather than over the track, and nothing is marked.
        """
        series = self._series
        if series is None or event.xdata is None or event.ydata is None:
            return None
        import math
        scale = 1.0 if self._world else (math.cos(math.radians(self._lat0)) or 1.0)
        best, best_d2 = None, None
        for lon, lat in ((series["lon"], series["lat"]),
                         (series["tlon"], series["tlat"])):
            dx = (lon - event.xdata) * scale
            dy = lat - event.ydata
            d2 = dx * dx + dy * dy
            if not np.any(np.isfinite(d2)):
                continue
            index = int(np.nanargmin(d2))
            if best_d2 is None or d2[index] < best_d2:
                best, best_d2 = index, d2[index]
        if best is None:
            return None
        y0, y1 = self.ax.get_ylim()
        tolerance = 0.05 * abs(y1 - y0)
        if best_d2 > tolerance * tolerance:
            return None
        return float(series["t"][best])

    def _on_leave(self, _event):
        self.hoverTime.emit(None)

    # -- navigation -------------------------------------------------------

    def fit(self):
        """Frame the whole selection again and forget the operator's zoom."""
        self.user_limits = None
        if self.run is not None:
            self.render(self.run, self.texts, keep_view=False)
        self.viewChanged.emit()

    def _on_scroll(self, event):
        if event.inaxes is not self.ax or self.run is None:
            return
        factor = 1.0 / ZOOM_STEP if event.button == "up" else ZOOM_STEP
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        cx = event.xdata if event.xdata is not None else 0.5 * (x0 + x1)
        cy = event.ydata if event.ydata is not None else 0.5 * (y0 + y1)
        # Zoom about the cursor: the point under the pointer stays put.
        self._set_limits(cx + (x0 - cx) * factor, cx + (x1 - cx) * factor,
                         cy + (y0 - cy) * factor, cy + (y1 - cy) * factor)

    def _on_press(self, event):
        if event.inaxes is not self.ax or self.run is None:
            return
        if event.dblclick:
            self.fit()
            return
        if event.button == 1:
            self._pan = (event.x, event.y, self.ax.get_xlim(), self.ax.get_ylim())

    def _on_motion(self, event):
        if self._pan is None:
            # Not dragging: the pointer is asking "what happened here?"
            self.hoverTime.emit(
                self._hover_time_at(event) if event.inaxes is self.ax else None)
            return
        if event.x is None:
            return
        px, py, (x0, x1), (y0, y1) = self._pan
        width = max(self.ax.bbox.width, 1.0)
        height = max(self.ax.bbox.height, 1.0)
        # Pixel delta converted through the limits at press time, so the point
        # grabbed stays under the pointer however far the drag goes.
        dx = (event.x - px) * (x1 - x0) / width
        dy = (event.y - py) * (y1 - y0) / height
        self._set_limits(x0 - dx, x1 - dx, y0 - dy, y1 - dy)

    def _on_release(self, _event):
        self._pan = None

    def _set_limits(self, x0, x1, y0, y1):
        self.ax.set_xlim(x0, x1)
        self.ax.set_ylim(y0, y1)
        self.user_limits = (x0, x1, y0, y1)
        F.decorate_track(self.ax, self._lat0, self._world)
        self.overlay.invalidate()
        self.draw_idle()
        self._tile_timer.start()
        self.viewChanged.emit()

    def _refresh_tiles(self):
        # Tiles are georeferenced in degrees; in the world frame there is
        # nothing sensible to place them against.
        if (self.run is None or not self.tiles_enabled or self.provider is None
                or self._world):
            return
        F.draw_tiles(self.ax, self.provider, self._canvas_px())
        self.overlay.invalidate()
        self.draw_idle()

    def set_tiles_enabled(self, enabled):
        self.tiles_enabled = bool(enabled)
        if self.run is not None:
            self.render(self.run, self.texts)

    def is_world_frame(self):
        """True when the panel is drawing local ENU metres rather than WGS84."""
        return self._world

    def limits(self):
        """What the export should frame, or None for 'fit the selection'."""
        return self.user_limits

