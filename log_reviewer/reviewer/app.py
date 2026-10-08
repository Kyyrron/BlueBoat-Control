#!/usr/bin/env python3

r"""
The window.

One log is open at a time. The timeline owns the selection; everything else -
four panels, the summary, the replay, the export - is a function of it, and is
rebuilt whenever it moves. Rebuilding all of it on a 1500-row log is about a
millisecond of numpy plus the draw, so the panels follow the handles live
rather than waiting for the drag to end; a 50 ms coalescing timer keeps a fast
drag from queueing more redraws than the screen can show.
"""

import os

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QDialog,
                               QDialogButtonBox, QDoubleSpinBox, QFileDialog,
                               QFormLayout, QFrame, QHBoxLayout, QHeaderView,
                               QLabel, QLineEdit, QMainWindow, QMessageBox,
                               QPlainTextEdit, QPushButton, QScrollArea,
                               QSizePolicy, QSlider, QSplitter, QTableWidget,
                               QTableWidgetItem, QVBoxLayout, QWidget)

from . import export as E
from . import figures as F
from . import poslog_bridge as pb
from . import replay_video as V
from .cursor import LinkedCursor, Overlay
from .tiles import TileProvider
from .timeline import RangeSlider
from .track_view import TrackCanvas

ROBOT_DATA = os.path.join(os.path.expanduser("~"), "ros2_ws", "data", "Robot_data")
REDRAW_MS = 50
FRAME_MS = 33
REPLAY_SPEEDS = (1, 2, 4, 8)


class PanelCanvas(FigureCanvasQTAgg):
    """One small matplotlib panel, with a linked-cursor dot on its curves."""

    hoverTime = Signal(object)          # mission seconds, or None on leaving

    def __init__(self, parent=None, height=215):
        figure = Figure(figsize=(6.0, 2.5), dpi=100, facecolor=pb.SURFACE)
        super().__init__(figure)
        self.setParent(parent)
        self.ax = figure.add_subplot(111)
        figure.subplots_adjust(left=0.085, right=0.98, top=0.94, bottom=0.21)
        self.setMinimumHeight(height)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.overlay = Overlay(self)
        self.cursor = LinkedCursor(self, self.ax, self.overlay)
        self.mpl_connect("motion_notify_event", self._on_motion)
        self.mpl_connect("axes_leave_event", self._on_leave)
        self.mpl_connect("figure_leave_event", self._on_leave)

    def _on_motion(self, event):
        # The x axis IS mission time here, so the pointer needs no lookup.
        self.hoverTime.emit(event.xdata if event.inaxes is self.ax else None)

    def _on_leave(self, _event):
        self.hoverTime.emit(None)

    def set_hover_time(self, when):
        self.cursor.set_time(when)


class TextHeader(QWidget):
    """The editable title and description of one panel.

    Nothing here is a label: every string the picture will carry is an entry,
    so a panel can be renamed for a report without touching the code.
    """

    changed = Signal()

    def __init__(self, title, description, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 6, 2, 0)
        layout.setSpacing(2)
        self.title = QLineEdit(title)
        self.title.setStyleSheet("font-weight: 600; font-size: 13px; border: none;"
                                 " background: transparent;")
        # A plain text edit, not a line edit: these sentences are long enough
        # that a single line shows the tail and hides the beginning.
        self.description = QPlainTextEdit(description)
        self.description.setStyleSheet(
            "color: %s; border: none; background: transparent;" % pb.MUTED)
        self.description.setFixedHeight(42)
        self.description.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.description.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.title.setToolTip("Panel title - appears on the exported picture")
        self.description.setToolTip("Panel description - appears on the exported picture")
        layout.addWidget(self.title)
        layout.addWidget(self.description)
        self.title.textChanged.connect(self.changed)
        self.description.textChanged.connect(self.changed)

    def values(self):
        return self.title.text(), self.description.toPlainText()

    def set_values(self, title, description):
        self.title.blockSignals(True)
        self.title.setText(title)
        self.title.setCursorPosition(0)
        self.title.blockSignals(False)
        self.description.blockSignals(True)
        self.description.setPlainText(description)
        self.description.blockSignals(False)


def fmt_duration(seconds):
    """`1 min 44 s` above a minute, `9.4 s` below."""
    if seconds >= 60.0:
        whole = int(round(seconds))
        return "%d min %02d s" % (whole // 60, whole % 60)
    return "%.1f s" % seconds


class ExportDialog(QDialog):
    """The last word before an export: its name, and how fast the GIF plays.

    The duration preview is `replay_video.video_timing` - the arithmetic the
    writer itself uses - so what it says is the length of the file, not an
    estimate of it.
    """

    def __init__(self, name, t0, t1, speed=V.DEFAULT_SPEED, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Export log")
        self._t0, self._t1 = float(t0), float(t1)

        form = QFormLayout()
        self.name_edit = QLineEdit(name)
        self.name_edit.setMinimumWidth(380)
        self.name_edit.setToolTip(
            "Names the export folder and its files, and titles the picture.\n"
            "The original log in Robot_data/ is never renamed.")
        form.addRow("Name", self.name_edit)

        row = QHBoxLayout()
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(V.MIN_SPEED, V.MAX_SPEED)
        self.speed_slider.setSingleStep(1)
        self.speed_slider.setPageStep(5)
        self.speed_slider.setTickInterval(5)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setValue(int(speed))
        self.speed_label = QLabel()
        self.speed_label.setFixedWidth(40)
        row.addWidget(self.speed_slider, 1)
        row.addWidget(self.speed_label)
        form.addRow("Replay video", row)

        self.preview = QLabel()
        self.preview.setStyleSheet("color: %s;" % pb.MUTED)
        form.addRow("", self.preview)

        self.buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        self.export_button = self.buttons.addButton("Export", QDialogButtonBox.AcceptRole)
        self.export_button.setDefault(True)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.buttons)

        self.speed_slider.valueChanged.connect(self._update_preview)
        self.name_edit.textChanged.connect(self._update_button)
        self._update_preview()
        self._update_button()

    def name(self):
        return self.name_edit.text()

    def speed(self):
        return int(self.speed_slider.value())

    def preview_text(self):
        return self.preview.text()

    def _update_preview(self):
        speed = self.speed()
        self.speed_label.setText("\u00d7%d" % speed)
        duration, frame_ms = V.video_timing(self._t0, self._t1, speed)
        self.preview.setText("Selection %s  \u2192  GIF %s  \u00b7  %.1f fps" % (
            fmt_duration(self._t1 - self._t0), fmt_duration(duration),
            1000.0 / frame_ms))

    def _update_button(self):
        self.export_button.setEnabled(bool(self.name_edit.text().strip()))


class ReviewerWindow(QMainWindow):

    def __init__(self, csv_path=None, fetch_tiles=True):
        super().__init__()
        self.setWindowTitle("BlueBoat log reviewer")
        self.resize(1720, 1000)

        self.run = None
        self.crop = None
        self.metrics = None
        self._hover_series = None
        self.source_csv = None
        self.texts = F.default_texts()

        self.provider = TileProvider(fetch=fetch_tiles, parent=self)
        try:
            self.provider.tileReady.connect(self._tiles_arrived)
        except AttributeError:
            pass

        self._redraw = QTimer(self)
        self._redraw.setSingleShot(True)
        self._redraw.setInterval(REDRAW_MS)
        self._redraw.timeout.connect(self._rebuild)

        self._frame = QTimer(self)
        self._frame.setInterval(FRAME_MS)
        self._frame.timeout.connect(self._tick)
        self._playhead = 0.0
        self._speed = 1
        self._video_speed = V.DEFAULT_SPEED

        self._build_ui()
        self._set_enabled(False)
        if csv_path:
            self.open_log(csv_path)

    # -- construction -----------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(10, 8, 10, 8)
        outer.setSpacing(8)
        outer.addLayout(self._build_toolbar())
        outer.addWidget(self._rule())
        outer.addLayout(self._build_timeline())
        outer.addWidget(self._rule())

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_left())
        splitter.addWidget(self._build_right())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 4)
        splitter.setSizes([760, 900])
        outer.addWidget(splitter, 1)

        self.setCentralWidget(central)
        self.readout = QLabel("")
        # Padded off the status message beside it, which is a different thing:
        # that one names the file, this one names the instant under the pointer.
        self.readout.setStyleSheet(
            "color: %s; padding-left: 18px; border-left: 1px solid %s;"
            % (pb.INK, pb.GRID))
        self.statusBar().addPermanentWidget(self.readout)
        self.statusBar().showMessage("Open a poslog CSV to begin.")

    def _rule(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color: %s;" % pb.GRID)
        return line

    def _build_toolbar(self):
        row = QHBoxLayout()
        self.open_button = QPushButton("Open log…")
        self.open_button.clicked.connect(self._choose_log)
        row.addWidget(self.open_button)

        row.addSpacing(14)
        row.addWidget(QLabel("Name"))
        self.name_edit = QLineEdit()
        self.name_edit.setMinimumWidth(420)
        self.name_edit.setToolTip(
            "Names the export folder and its files, and titles the picture.\n"
            "The original log in Robot_data/ is never renamed.")
        self.name_edit.textChanged.connect(self._name_changed)
        row.addWidget(self.name_edit, 1)

        self.export_button = QPushButton("Export log")
        self.export_button.clicked.connect(self._export)
        self.export_button.setToolTip(
            "Write the selected window to ~/ros2_ws/data/Processed_Robot_data/,\n"
            "with a \u00d75\u2013\u00d720 replay video of the track as framed here")
        row.addWidget(self.export_button)
        return row

    def _build_timeline(self):
        row = QHBoxLayout()
        self.slider = RangeSlider()
        self.slider.rangeChanged.connect(self._range_changed)
        row.addWidget(self.slider, 1)

        self.start_spin = QDoubleSpinBox()
        self.end_spin = QDoubleSpinBox()
        for spin, tip in ((self.start_spin, "Start of the selection, mission seconds"),
                          (self.end_spin, "End of the selection, mission seconds")):
            spin.setDecimals(2)
            spin.setSuffix(" s")
            spin.setSingleStep(1.0)
            spin.setFixedWidth(110)
            spin.setToolTip(tip)
            spin.valueChanged.connect(self._spin_changed)
        row.addWidget(QLabel("from"))
        row.addWidget(self.start_spin)
        row.addWidget(QLabel("to"))
        row.addWidget(self.end_spin)

        self.window_label = QLabel("")
        self.window_label.setStyleSheet("color: %s;" % pb.MUTED)
        self.window_label.setMinimumWidth(330)
        row.addWidget(self.window_label)

        reset = QPushButton("Reset")
        reset.setToolTip("Select the whole run again")
        reset.clicked.connect(self._reset_range)
        row.addWidget(reset)
        return row

    def _build_left(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 6, 0)
        layout.setSpacing(4)

        self.track_header = TextHeader(self.texts["track_title"],
                                       self.texts["track_desc"])
        self.track_header.changed.connect(self._texts_changed)
        layout.addWidget(self.track_header)

        self.track = TrackCanvas(panel)
        self.track.set_provider(self.provider)
        self.track.hoverTime.connect(self._hover)
        layout.addWidget(self.track, 1)
        layout.addLayout(self._build_replay_bar())
        return panel

    def _build_replay_bar(self):
        row = QHBoxLayout()
        self.play_button = QPushButton("▶  Replay")
        self.play_button.setCheckable(True)
        self.play_button.setFixedWidth(110)
        self.play_button.toggled.connect(self._toggle_replay)
        row.addWidget(self.play_button)

        self.speed_group = QButtonGroup(self)
        for factor in REPLAY_SPEEDS:
            button = QPushButton("×%d" % factor)
            button.setCheckable(True)
            button.setFixedWidth(46)
            button.setChecked(factor == 1)
            self.speed_group.addButton(button, factor)
            row.addWidget(button)
        self.speed_group.idClicked.connect(self._set_speed)

        self.scrub = QSlider(Qt.Horizontal)
        self.scrub.setRange(0, 1000)
        self.scrub.setToolTip("Replay position inside the selected window")
        self.scrub.sliderMoved.connect(self._scrub_moved)
        row.addWidget(self.scrub, 1)

        self.clock_label = QLabel("0.0 s")
        self.clock_label.setFixedWidth(78)
        self.clock_label.setStyleSheet("color: %s;" % pb.MUTED)
        row.addWidget(self.clock_label)

        self.satellite_box = QCheckBox("Satellite")
        self.satellite_box.setChecked(True)
        self.satellite_box.setToolTip(
            "Draw the Mission Control Station's cached satellite tiles under the track")
        self.satellite_box.toggled.connect(self.track.set_tiles_enabled)
        row.addWidget(self.satellite_box)

        fit = QPushButton("Fit")
        fit.setToolTip("Frame the whole selection (double-click the track does this too)")
        fit.setFixedWidth(56)
        fit.clicked.connect(self.track.fit)
        row.addWidget(fit)
        return row

    def _build_right(self):
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setFrameShape(QFrame.NoFrame)
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setContentsMargins(6, 0, 0, 0)
        layout.setSpacing(4)

        self.headers = {}
        self.panels = {}
        for key, title_key, desc_key in (
                ("distance", "distance_title", "distance_desc"),
                ("speed", "speed_title", "speed_desc"),
                ("thrust", "thrust_title", "thrust_desc")):
            header = TextHeader(self.texts[title_key], self.texts[desc_key])
            header.changed.connect(self._texts_changed)
            canvas = PanelCanvas(inner)
            canvas.hoverTime.connect(self._hover)
            layout.addWidget(header)
            layout.addWidget(canvas, 1)
            self.headers[key] = header
            self.panels[key] = canvas

        self.summary_title = QLineEdit(self.texts["summary_title"])
        self.summary_title.setStyleSheet("font-weight: 600; font-size: 13px;"
                                         " border: none; background: transparent;")
        self.summary_title.textChanged.connect(self._texts_changed)
        layout.addWidget(self.summary_title)

        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["Metric", "Value"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.setMinimumHeight(300)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        layout.addWidget(self.table)

        area.setWidget(inner)
        return area

    def _set_enabled(self, enabled):
        for widget in (self.name_edit, self.export_button, self.slider,
                       self.start_spin, self.end_spin, self.play_button,
                       self.scrub, self.satellite_box):
            widget.setEnabled(enabled)

    # -- opening ----------------------------------------------------------

    def _choose_log(self):
        start = ROBOT_DATA if os.path.isdir(ROBOT_DATA) else os.path.expanduser("~")
        path, _ = QFileDialog.getOpenFileName(
            self, "Open a poslog CSV", start, "Poslog CSV (*.csv);;All files (*)")
        if path:
            self.open_log(path)

    def open_log(self, path):
        """Load a log in place - the window is never rebuilt for a new file."""
        try:
            run = pb.load(path)
        except (pb.PoslogError, OSError, ValueError) as exc:
            QMessageBox.critical(self, "Cannot read this log", str(exc))
            return

        self._stop_replay()
        self.run = run
        self.source_csv = os.path.abspath(path)
        self.texts = F.default_texts(run["stem"], run)
        self.track.user_limits = None

        self.name_edit.blockSignals(True)
        self.name_edit.setText(run["stem"])
        self.name_edit.blockSignals(False)

        self.track_header.set_values(self.texts["track_title"], self.texts["track_desc"])
        for key in ("distance", "speed", "thrust"):
            self.headers[key].set_values(self.texts[key + "_title"],
                                         self.texts[key + "_desc"])
        self.summary_title.blockSignals(True)
        self.summary_title.setText(self.texts["summary_title"])
        self.summary_title.blockSignals(False)

        t0, t1 = run["t_full"]
        for spin in (self.start_spin, self.end_spin):
            spin.blockSignals(True)
            spin.setRange(t0, t1)
            spin.blockSignals(False)
        self.slider.blockSignals(True)
        self.slider.set_bounds(t0, t1, reset=True)
        self.slider.blockSignals(False)
        self._sync_spins(t0, t1)

        self._set_enabled(True)
        self.setWindowTitle("BlueBoat log reviewer — %s" % run["stem"])
        self._rebuild(fit=True)

    # -- selection --------------------------------------------------------

    def _range_changed(self, _t0, _t1):
        self._sync_spins(*self.slider.values())
        self._redraw.start()

    def _spin_changed(self, _value):
        low, high = self.start_spin.value(), self.end_spin.value()
        if high < low:
            high = low
        self.slider.blockSignals(True)
        self.slider.set_values(low, high, emit=False)
        self.slider.blockSignals(False)
        self._redraw.start()

    def _reset_range(self):
        if self.run is None:
            return
        t0, t1 = self.run["t_full"]
        self.slider.set_values(t0, t1, emit=False)
        self._sync_spins(t0, t1)
        self._rebuild()

    def _sync_spins(self, low, high):
        for spin, value in ((self.start_spin, low), (self.end_spin, high)):
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)

    def _name_changed(self, text):
        self.texts["report_title"] = text

    def _texts_changed(self):
        """Panel text is not data, so only the affected panels are redrawn."""
        if self.run is None:
            return
        self.texts["track_title"], self.texts["track_desc"] = self.track_header.values()
        for key in ("distance", "speed", "thrust"):
            title, description = self.headers[key].values()
            self.texts[key + "_title"] = title
            self.texts[key + "_desc"] = description
        self.texts["summary_title"] = self.summary_title.text()
        self._redraw.start()

    # -- rebuilding -------------------------------------------------------

    def _rebuild(self, fit=False):
        if self.run is None:
            return
        low, high = self.slider.values()
        self.crop = pb.crop_run(self.run, low, high)
        self.metrics = pb.compute_metrics(self.crop)

        distance, have = pb.distance_series(self.crop)
        speed, jumps = pb.speed_series(self.crop)

        # show_headline=False everywhere on screen: the title and description
        # are the entries above each canvas. Only the export draws them.
        F.plot_distance(self.panels["distance"].ax, self.crop, self.metrics,
                        self.texts, distance, have, show_headline=False)
        F.plot_speed(self.panels["speed"].ax, self.crop, self.metrics,
                     self.texts, speed, jumps, show_headline=False)
        F.plot_thrusters(self.panels["thrust"].ax, self.crop, self.texts,
                         show_headline=False)

        # Rebuild the cursor artists onto the freshly cleared axes, over the
        # SAME masked arrays the panels plot - so a dot sits on the curve, and
        # vanishes where the curve does (no target, or an excluded pose jump).
        t, data = self.crop["t"], self.crop["data"]
        self.panels["distance"].cursor.set_data(
            t, [(t, np.where(have, distance, np.nan), pb.TARGET, 7)])
        self.panels["speed"].cursor.set_data(
            t, [(t, np.where(jumps, np.nan, speed), pb.ROBOT, 7)])
        self.panels["thrust"].cursor.set_data(
            t, [(t, data["right_thr_in"], pb.ROBOT, 7),
                (t, data["left_thr_in"], pb.TARGET, 7)])
        self._hover_series = {"distance": np.where(have, distance, np.nan),
                              "speed": np.where(jumps, np.nan, speed),
                              "right": data["right_thr_in"],
                              "left": data["left_thr_in"]}
        for canvas in self.panels.values():
            canvas.overlay.invalidate()
            canvas.draw_idle()

        if fit:
            self.track.user_limits = None
        self.track.render(self.crop, self.texts, keep_view=not fit)
        self._sync_satellite_box()
        self._fill_table()
        self._update_window_label()
        self._clamp_playhead()

    def _fill_table(self):
        rows = pb.summary_rows(self.crop, self.metrics)
        self.table.setRowCount(len(rows))
        for index, (label, value) in enumerate(rows):
            self.table.setItem(index, 0, QTableWidgetItem(label))
            item = QTableWidgetItem(value)
            font = item.font()
            font.setBold(True)
            item.setFont(font)
            self.table.setItem(index, 1, item)
        self.table.resizeRowsToContents()

    def _update_window_label(self):
        rows = self.crop["row_index"]
        first = int(rows[0]) if len(rows) else 0
        last = int(rows[-1]) if len(rows) else 0
        self.window_label.setText(
            "%d of %d rows · %s → %s" % (
                self.crop["n_rows"], self.run["n_rows_full"],
                pb.wall_clock(self.crop, 0).split(" ")[-1],
                pb.wall_clock(self.crop, self.crop["n_rows"] - 1).split(" ")[-1]))
        self.statusBar().showMessage(
            "%s · source rows %d–%d · %s" % (
                self.source_csv, first, last, pb.subtitle_for(self.crop, self.metrics)))

    def _hover(self, when):
        """One instant, marked on every panel at once.

        Whichever plot the pointer is over is the source; all four - the three
        time series and the track - mark the same mission time, so the boat's
        position, its speed, its distance to target and what the thrusters were
        being asked for are read off together.
        """
        if self.crop is None:
            return
        index = None
        for canvas in self.panels.values():
            index = canvas.cursor.set_time(when) if when is not None else None
            if when is None:
                canvas.cursor.hide()
        self.track.set_hover_time(when)
        self._show_readout(index if when is not None else None)

    def _show_readout(self, index):
        if index is None or self._hover_series is None:
            self.readout.setText("")
            return
        series = self._hover_series
        parts = ["t %.1f s" % self.crop["t"][index],
                 pb.wall_clock(self.crop, index).split(" ")[-1]]
        for label, key, unit in (("to target", "distance", " m"),
                                 ("|v|", "speed", " m/s")):
            value = series[key][index]
            parts.append("%s %s" % (label, pb.fmt(float(value), unit)))
        parts.append("thrust %s / %s N" % (pb.fmt(float(series["right"][index])),
                                           pb.fmt(float(series["left"][index]))))
        self.readout.setText("   \u00b7   ".join(parts))

    def _sync_satellite_box(self):
        """Satellite imagery is georeferenced; the world frame has no use for it.

        The box is disabled rather than hidden, and says why, so a simulation
        log does not look like a broken tile cache.
        """
        world = self.track.is_world_frame()
        self.satellite_box.setEnabled(not world)
        self.satellite_box.setToolTip(
            "Not available in the local-ENU frame - satellite tiles are "
            "georeferenced in degrees, and a simulated run is plotted in metres"
            if world else
            "Draw the Mission Control Station's cached satellite tiles under the track")

    def _tiles_arrived(self):
        """A background tile landed; fold it in without disturbing the view."""
        if self.run is not None and self.satellite_box.isChecked():
            self.track._tile_timer.start()

    # -- replay -----------------------------------------------------------

    def _toggle_replay(self, on):
        if self.run is None:
            return
        self.play_button.setText("⏸  Pause" if on else "▶  Replay")
        self.track.set_replay_active(on)
        if on:
            low, high = self.slider.values()
            if not low <= self._playhead < high:
                self._playhead = low
            self._frame.start()
        else:
            self._frame.stop()

    def _stop_replay(self):
        self._frame.stop()
        self.play_button.blockSignals(True)
        self.play_button.setChecked(False)
        self.play_button.setText("▶  Replay")
        self.play_button.blockSignals(False)
        self.track.set_replay_active(False)

    def _set_speed(self, factor):
        self._speed = int(factor)

    def _tick(self):
        low, high = self.slider.values()
        self._playhead += (FRAME_MS / 1000.0) * self._speed
        if self._playhead >= high:
            self._playhead = low               # loop rather than stop at the end
        self._show_playhead()

    def _scrub_moved(self, value):
        low, high = self.slider.values()
        self._playhead = low + (high - low) * value / 1000.0
        self._show_playhead()

    def _clamp_playhead(self):
        low, high = self.slider.values()
        self._playhead = min(max(self._playhead, low), high)
        self._show_playhead()

    def _show_playhead(self):
        low, high = self.slider.values()
        span = max(high - low, 1e-9)
        self.scrub.blockSignals(True)
        self.scrub.setValue(int(1000 * (self._playhead - low) / span))
        self.scrub.blockSignals(False)
        self.clock_label.setText("%.1f s" % self._playhead)
        self.track.set_playhead(self._playhead)

    # -- export -----------------------------------------------------------

    def _export(self):
        if self.crop is None:
            return
        t = self.crop["t"]
        dialog = ExportDialog(self.name_edit.text(), t[0] if len(t) else 0.0,
                              t[-1] if len(t) else 0.0, self._video_speed, self)
        if dialog.exec() != QDialog.Accepted:
            return
        # Written back to the toolbar: that entry is also the picture's title.
        self.name_edit.setText(dialog.name())
        self._video_speed = dialog.speed()

        name = E.safe_name(self.name_edit.text(), self.run["stem"])
        folder = os.path.join(E.DEFAULT_ROOT, name)
        overwrite = False
        if os.path.isdir(folder):
            answer = QMessageBox.question(
                self, "Already exported",
                "%s already exists.\n\nReplace its contents?" % folder,
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
            overwrite = True

        was_playing = self.play_button.isChecked()
        if was_playing:
            self.play_button.setChecked(False)
        try:
            # The replay video takes a few seconds on a long run. Restored
            # before any dialog, so an error box is not under a busy cursor.
            QApplication.setOverrideCursor(Qt.WaitCursor)
            try:
                written = E.export(
                    self.source_csv, name, self.crop, self.texts, root=E.DEFAULT_ROOT,
                    tile_provider=self.provider if self.satellite_box.isChecked() else None,
                    track_limits=self.track.limits(), overwrite=overwrite,
                    video_speed=self._video_speed)
            finally:
                QApplication.restoreOverrideCursor()
        except Exception as exc:                                # noqa: BLE001
            # Deliberately broad. An export writes five files through three
            # libraries; whatever one of them raises, the operator must be told
            # in the window rather than in a terminal they may not be watching,
            # and the app must survive to let them try again.
            QMessageBox.critical(
                self, "Export failed",
                "%s: %s\n\nPartial files may be left in\n%s"
                % (type(exc).__name__, exc, folder))
            return
        video = os.path.isfile(os.path.join(written, name + ".gif"))
        duration, _ = V.video_timing(t[0], t[-1], self._video_speed) if len(t) else (0, 0)
        QMessageBox.information(
            self, "Exported",
            "%d of %d rows written to\n\n%s\n\n%s" % (
                self.crop["n_rows"], self.run["n_rows_full"], written,
                "With a \u00d7%d replay video (%s)." % (self._video_speed,
                                                        fmt_duration(duration))
                if video else "No replay video (export.yaml says why)."))
        self.statusBar().showMessage("Exported to %s" % written)
