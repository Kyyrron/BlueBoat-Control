#!/usr/bin/env python3

r"""
The timeline: a two-handled range over mission time.

Qt ships no range slider, so this is a small painted widget. It works in
MISSION SECONDS, the same axis every panel is drawn against, and emits
`rangeChanged(t0, t1)` while you drag. The window it selects is the only thing
the rest of the app knows about: every figure and every number in the summary
is recomputed from the rows inside it.
"""

from PySide6.QtCore import QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from . import poslog_bridge as pb

MARGIN = 12          # px of dead space each side, so a handle at an end is whole
HANDLE_W = 11
GROOVE_H = 8


class RangeSlider(QWidget):
    """Two handles over a continuous float range."""

    rangeChanged = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._minimum = 0.0
        self._maximum = 1.0
        self._low = 0.0
        self._high = 1.0
        self._grab = None          # 'low' | 'high' | 'band'
        self._grab_x = 0.0
        self._grab_low = 0.0
        self._grab_high = 0.0
        self.setMinimumHeight(34)
        self.setCursor(Qt.SizeHorCursor)

    # -- state ------------------------------------------------------------

    def bounds(self):
        return self._minimum, self._maximum

    def values(self):
        return self._low, self._high

    def set_bounds(self, minimum, maximum, reset=True):
        self._minimum = float(minimum)
        self._maximum = float(max(maximum, minimum + 1e-6))
        if reset:
            self._low, self._high = self._minimum, self._maximum
        else:
            self._low = min(max(self._low, self._minimum), self._maximum)
            self._high = min(max(self._high, self._low), self._maximum)
        self.update()

    def set_values(self, low, high, emit=True):
        low, high = float(low), float(high)
        low = min(max(low, self._minimum), self._maximum)
        high = min(max(high, self._minimum), self._maximum)
        if high < low:
            low, high = high, low
        changed = (low, high) != (self._low, self._high)
        self._low, self._high = low, high
        self.update()
        if changed and emit:
            self.rangeChanged.emit(self._low, self._high)

    # -- geometry ---------------------------------------------------------

    def _track_px(self):
        return max(1, self.width() - 2 * MARGIN)

    def _x_of(self, value):
        span = self._maximum - self._minimum
        frac = 0.0 if span <= 0 else (value - self._minimum) / span
        return MARGIN + frac * self._track_px()

    def _value_of(self, x):
        frac = (x - MARGIN) / float(self._track_px())
        return self._minimum + min(max(frac, 0.0), 1.0) * (self._maximum - self._minimum)

    # -- interaction ------------------------------------------------------

    def mousePressEvent(self, event):
        x = event.position().x()
        x_low, x_high = self._x_of(self._low), self._x_of(self._high)
        self._grab_x, self._grab_low, self._grab_high = x, self._low, self._high
        if abs(x - x_low) <= HANDLE_W and abs(x - x_low) <= abs(x - x_high):
            self._grab = "low"
        elif abs(x - x_high) <= HANDLE_W:
            self._grab = "high"
        elif x_low < x < x_high:
            self._grab = "band"          # drag the whole window, keeping its width
        else:
            # A click outside the window moves the nearer handle to it, so a
            # long run can be trimmed without dragging across the whole widget.
            self._grab = "low" if x < x_low else "high"
            self._apply(x)
        self.update()

    def mouseMoveEvent(self, event):
        if self._grab:
            self._apply(event.position().x())

    def mouseReleaseEvent(self, _event):
        self._grab = None
        self.update()

    def _apply(self, x):
        value = self._value_of(x)
        if self._grab == "low":
            self.set_values(min(value, self._high), self._high)
        elif self._grab == "high":
            self.set_values(self._low, max(value, self._low))
        elif self._grab == "band":
            shift = self._value_of(x) - self._value_of(self._grab_x)
            width = self._grab_high - self._grab_low
            low = min(max(self._grab_low + shift, self._minimum), self._maximum - width)
            self.set_values(low, low + width)

    # -- painting ---------------------------------------------------------

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        mid = self.height() / 2.0
        x_low, x_high = self._x_of(self._low), self._x_of(self._high)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(pb.GRID)))
        painter.drawRoundedRect(
            QRectF(MARGIN, mid - GROOVE_H / 2, self._track_px(), GROOVE_H), 4, 4)

        painter.setBrush(QBrush(QColor(pb.ROBOT)))
        painter.drawRoundedRect(
            QRectF(x_low, mid - GROOVE_H / 2, max(x_high - x_low, 1), GROOVE_H), 4, 4)

        painter.setPen(QPen(QColor(pb.SURFACE), 2))
        for x, name in ((x_low, "low"), (x_high, "high")):
            painter.setBrush(QBrush(QColor(pb.INK if self._grab == name else pb.INK2)))
            painter.drawRoundedRect(
                QRectF(x - HANDLE_W / 2, mid - 11, HANDLE_W, 22), 3, 3)
        painter.end()
