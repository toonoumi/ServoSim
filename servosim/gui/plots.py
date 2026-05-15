from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout

WINDOW_POINTS = 1000   # 10 s at 10ms per sample


class PlotPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        pg.setConfigOptions(antialias=True, background="#1a1a2e", foreground="#cccccc")

        self._glw = pg.GraphicsLayoutWidget()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._glw)

        self._t   = deque(maxlen=WINDOW_POINTS)
        self._pos = deque(maxlen=WINDOW_POINTS)
        self._tgt = deque(maxlen=WINDOW_POINTS)
        self._err = deque(maxlen=WINDOW_POINTS)
        self._cur = deque(maxlen=WINDOW_POINTS)
        self._torq = deque(maxlen=WINDOW_POINTS)

        def make_plot(title, unit):
            p = self._glw.addPlot(title=title)
            p.setLabel("left", unit)
            p.setLabel("bottom", "time (ms)")
            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend(offset=(5, 5))
            self._glw.nextRow()
            return p

        self._p_pos  = make_plot("Position", "degrees")
        self._p_err  = make_plot("Error", "degrees")
        self._p_cur  = make_plot("Current", "A")
        self._p_torq = make_plot("Torque", "Nm")

        # Link x-axes for synchronized scrolling
        for p in (self._p_err, self._p_cur, self._p_torq):
            p.setXLink(self._p_pos)

        self._c_pos    = self._p_pos.plot(pen=pg.mkPen("#f0c040", width=1.5), name="position")
        self._c_target = self._p_pos.plot(pen=pg.mkPen("#40c040", width=1, style=Qt.DashLine), name="target")
        self._c_err    = self._p_err.plot(pen=pg.mkPen("#e05050", width=1.5), name="error")
        self._c_cur    = self._p_cur.plot(pen=pg.mkPen("#40c0f0", width=1.5), name="current")
        self._c_torq   = self._p_torq.plot(pen=pg.mkPen("#c040e0", width=1.5), name="torque")

        self._frame_skip = 0

    def append(self, data: dict) -> None:
        ts = data["timestamp"]
        self._t.append(ts)
        self._pos.append(data["position"])
        self._tgt.append(data["target"])
        self._err.append(data["position_error"])
        self._cur.append(data["current"])
        self._torq.append(data["torque"])

        # Redraw every 3 samples (~33 Hz) to keep GUI responsive
        self._frame_skip += 1
        if self._frame_skip < 3:
            return
        self._frame_skip = 0

        t = np.array(self._t)
        self._c_pos.setData(t, np.array(self._pos))
        self._c_target.setData(t, np.array(self._tgt))
        self._c_err.setData(t, np.array(self._err))
        self._c_cur.setData(t, np.array(self._cur))
        self._c_torq.setData(t, np.array(self._torq))

        if len(t) >= 2:
            x_min = t[-1] - WINDOW_POINTS * 10  # 10ms per tick
            self._p_pos.setXRange(x_min, t[-1], padding=0)

    def clear_data(self) -> None:
        for buf in (self._t, self._pos, self._tgt, self._err, self._cur, self._torq):
            buf.clear()
        for curve in (self._c_pos, self._c_target, self._c_err, self._c_cur, self._c_torq):
            curve.setData([], [])
