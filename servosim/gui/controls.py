from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout,
    QLabel, QSlider, QDoubleSpinBox, QPushButton, QComboBox, QSizePolicy,
    QCheckBox, QSpinBox,
)

from ..physics import PayloadDirection


class HallIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._leds = {}
        for name in ("A", "B", "C"):
            lbl = QLabel()
            lbl.setFixedSize(18, 18)
            self._set_led(lbl, False)
            layout.addWidget(QLabel(f"{name}:"))
            layout.addWidget(lbl)
            layout.addSpacing(6)
            self._leds[name] = lbl

    @staticmethod
    def _set_led(lbl: QLabel, on: bool) -> None:
        color = "#00CC44" if on else "#444444"
        lbl.setStyleSheet(
            f"background:{color}; border-radius:9px; border:1px solid #222;"
        )

    def update_state(self, a: int, b: int, c: int) -> None:
        self._set_led(self._leds["A"], bool(a))
        self._set_led(self._leds["B"], bool(b))
        self._set_led(self._leds["C"], bool(c))


class StatusPanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Telemetry", parent)
        form = QFormLayout(self)
        form.setVerticalSpacing(3)
        self._fields = {}
        for key, label in [
            ("position",      "Position (°)"),
            ("target",        "Target (°)"),
            ("position_error","Error (°)"),
            ("voltage",       "Voltage (V)"),
            ("current",       "Current (A)"),
            ("power",         "Power (W)"),
            ("torque",        "Torque (Nm)"),
            ("kp",            "Kp"),
            ("ki",            "Ki"),
            ("kd",            "Kd"),
            ("timestamp",     "Tick"),
        ]:
            val = QLabel("—")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setMinimumWidth(90)
            form.addRow(label + ":", val)
            self._fields[key] = val

        hall_row = QHBoxLayout()
        self._hall = HallIndicator()
        hall_row.addWidget(QLabel("Hall A/B/C:"))
        hall_row.addWidget(self._hall)
        hall_row.addStretch()
        form.addRow(hall_row)

    def update_status(self, data: dict) -> None:
        for key, lbl in self._fields.items():
            if key in data:
                v = data[key]
                lbl.setText(f"{v:.4f}" if isinstance(v, float) else str(v))
        self._hall.update_state(
            data.get("hall_a", 0),
            data.get("hall_b", 0),
            data.get("hall_c", 0),
        )


class ControlPanel(QGroupBox):
    target_changed      = pyqtSignal(float)
    payload_changed     = pyqtSignal(float)
    direction_changed   = pyqtSignal(str, float)   # (direction_value, tilt_deg)
    gains_changed       = pyqtSignal(float, float, float)
    injection_changed   = pyqtSignal(dict)          # full InjectionParams dict
    cycle_start_clicked = pyqtSignal(float)         # hold_time_ms
    run_clicked         = pyqtSignal()
    pause_clicked       = pyqtSignal()
    reset_clicked       = pyqtSignal()

    _DIRECTION_LABELS = [
        ("Vertical (arm in vertical plane)",   PayloadDirection.VERTICAL),
        ("Horizontal (arm in horizontal plane)", PayloadDirection.HORIZONTAL),
        ("Rotational (flywheel / added inertia)", PayloadDirection.ROTATIONAL),
        ("Angled (tilted plane)",               PayloadDirection.ANGLED),
    ]

    def __init__(self, parent=None):
        super().__init__("Controls", parent)
        layout = QVBoxLayout(self)

        # --- Target position ---
        tgt_grp = QGroupBox("Target Position")
        tgt_lay = QVBoxLayout(tgt_grp)
        self._tgt_slider = QSlider(Qt.Horizontal)
        self._tgt_slider.setRange(0, 3600)
        self._tgt_slider.setValue(0)
        self._tgt_spin = QDoubleSpinBox()
        self._tgt_spin.setRange(0.0, 360.0)
        self._tgt_spin.setSingleStep(1.0)
        self._tgt_spin.setSuffix(" °")
        tgt_lay.addWidget(self._tgt_slider)
        tgt_lay.addWidget(self._tgt_spin)
        self._tgt_slider.valueChanged.connect(
            lambda v: self._tgt_spin.setValue(v / 10.0))
        self._tgt_spin.valueChanged.connect(
            lambda v: (self._tgt_slider.blockSignals(True),
                       self._tgt_slider.setValue(int(v * 10)),
                       self._tgt_slider.blockSignals(False),
                       self.target_changed.emit(v)))
        layout.addWidget(tgt_grp)

        # --- Payload ---
        pay_grp = QGroupBox("Payload")
        pay_form = QFormLayout(pay_grp)
        pay_form.setVerticalSpacing(4)

        # Mass
        self._pay_spin = QDoubleSpinBox()
        self._pay_spin.setRange(0.0, 10.0)
        self._pay_spin.setSingleStep(0.1)
        self._pay_spin.setSuffix(" kg")
        self._pay_slider = QSlider(Qt.Horizontal)
        self._pay_slider.setRange(0, 1000)
        self._pay_slider.setValue(0)
        self._pay_slider.valueChanged.connect(
            lambda v: self._pay_spin.setValue(v / 100.0))
        self._pay_spin.valueChanged.connect(
            lambda v: (self._pay_slider.blockSignals(True),
                       self._pay_slider.setValue(int(v * 100)),
                       self._pay_slider.blockSignals(False)))
        pay_form.addRow("Mass:", self._pay_slider)
        pay_form.addRow("", self._pay_spin)

        # Direction combo
        self._dir_combo = QComboBox()
        for label, _ in self._DIRECTION_LABELS:
            self._dir_combo.addItem(label)
        self._dir_combo.setCurrentIndex(0)
        self._dir_combo.currentIndexChanged.connect(self._on_direction_changed)
        pay_form.addRow("Direction:", self._dir_combo)

        # Tilt angle (only visible in ANGLED mode)
        self._tilt_label = QLabel("Tilt angle:")
        self._tilt_spin = QDoubleSpinBox()
        self._tilt_spin.setRange(0.0, 90.0)
        self._tilt_spin.setSingleStep(5.0)
        self._tilt_spin.setSuffix(" °")
        self._tilt_spin.setValue(45.0)
        self._tilt_slider = QSlider(Qt.Horizontal)
        self._tilt_slider.setRange(0, 900)
        self._tilt_slider.setValue(450)
        self._tilt_slider.valueChanged.connect(
            lambda v: self._tilt_spin.setValue(v / 10.0))
        self._tilt_spin.valueChanged.connect(
            lambda v: (self._tilt_slider.blockSignals(True),
                       self._tilt_slider.setValue(int(v * 10)),
                       self._tilt_slider.blockSignals(False),
                       self._emit_direction()))
        pay_form.addRow(self._tilt_label, self._tilt_slider)
        pay_form.addRow("", self._tilt_spin)
        self._tilt_label.setVisible(False)
        self._tilt_slider.setVisible(False)
        self._tilt_spin.setVisible(False)

        btn_apply_pay = QPushButton("Apply Mid-Motion")
        btn_apply_pay.clicked.connect(self._apply_payload)
        pay_form.addRow(btn_apply_pay)
        layout.addWidget(pay_grp)

        # --- PID gains ---
        pid_grp = QGroupBox("PID Gains")
        pid_form = QFormLayout(pid_grp)
        self._kp = self._make_gain_spin(10.0)
        self._ki = self._make_gain_spin(0.5)
        self._kd = self._make_gain_spin(0.1)
        pid_form.addRow("Kp:", self._kp)
        pid_form.addRow("Ki:", self._ki)
        pid_form.addRow("Kd:", self._kd)
        btn_gains = QPushButton("Apply Gains")
        btn_gains.clicked.connect(self._emit_gains)
        pid_form.addRow(btn_gains)
        layout.addWidget(pid_grp)

        # --- Injection molding load ---
        inj_grp = QGroupBox("Injection Load")
        inj_form = QFormLayout(inj_grp)
        inj_form.setVerticalSpacing(4)

        self._inj_enable = QCheckBox("Enable injection mode")
        inj_form.addRow(self._inj_enable)

        def make_inj_spin(lo, hi, default, decimals=4, step=0.01, suffix=""):
            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(decimals)
            s.setSingleStep(step)
            s.setValue(default)
            if suffix:
                s.setSuffix(suffix)
            return s

        self._inj_engagement = make_inj_spin(1.0, 180.0, 30.0, decimals=1, step=1.0, suffix=" °")
        self._inj_release    = make_inj_spin(0.5, 90.0, 5.0, decimals=1, step=0.5, suffix=" °")
        self._inj_k_approach = make_inj_spin(0.0, 5.0, 0.08, step=0.01, suffix=" Nm")
        self._inj_k_release  = make_inj_spin(0.0, 5.0, 0.04, step=0.01, suffix=" Nm")
        self._inj_holding    = make_inj_spin(0.0, 5.0, 0.02, step=0.005, suffix=" Nm")
        self._inj_shear      = make_inj_spin(0.0, 10.0, 0.5, step=0.05)

        inj_form.addRow("Engagement dist:", self._inj_engagement)
        inj_form.addRow("Release dist:", self._inj_release)
        inj_form.addRow("Approach resistance:", self._inj_k_approach)
        inj_form.addRow("Release stickiness:", self._inj_k_release)
        inj_form.addRow("Holding backpressure:", self._inj_holding)
        inj_form.addRow("Shear-thinning:", self._inj_shear)

        self._inj_hold_time = QSpinBox()
        self._inj_hold_time.setRange(50, 10000)
        self._inj_hold_time.setSingleStep(50)
        self._inj_hold_time.setValue(500)
        self._inj_hold_time.setSuffix(" ms")
        inj_form.addRow("Hold time:", self._inj_hold_time)

        self._btn_cycle = QPushButton("Start Cycle")
        self._btn_cycle.setEnabled(False)
        inj_form.addRow(self._btn_cycle)
        layout.addWidget(inj_grp)

        self._inj_param_widgets = [
            self._inj_engagement, self._inj_release, self._inj_k_approach,
            self._inj_k_release, self._inj_holding, self._inj_shear,
            self._inj_hold_time, self._btn_cycle,
        ]
        for w in self._inj_param_widgets:
            w.setEnabled(False)

        def _on_inj_enable(state):
            enabled = bool(state)
            for w in self._inj_param_widgets:
                w.setEnabled(enabled)
            self._emit_injection()

        self._inj_enable.stateChanged.connect(_on_inj_enable)
        for spin in (self._inj_engagement, self._inj_release, self._inj_k_approach,
                     self._inj_k_release, self._inj_holding, self._inj_shear):
            spin.valueChanged.connect(lambda _: self._emit_injection())
        self._btn_cycle.clicked.connect(
            lambda: self.cycle_start_clicked.emit(float(self._inj_hold_time.value())))

        # --- Simulation buttons ---
        btn_row = QHBoxLayout()
        self._btn_run   = QPushButton("Run")
        self._btn_pause = QPushButton("Pause")
        self._btn_reset = QPushButton("Reset")
        self._btn_run.clicked.connect(self.run_clicked)
        self._btn_pause.clicked.connect(self.pause_clicked)
        self._btn_reset.clicked.connect(self.reset_clicked)
        btn_row.addWidget(self._btn_run)
        btn_row.addWidget(self._btn_pause)
        btn_row.addWidget(self._btn_reset)
        layout.addLayout(btn_row)
        layout.addStretch()

    @staticmethod
    def _make_gain_spin(default: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 1000.0)
        spin.setDecimals(4)
        spin.setSingleStep(0.1)
        spin.setValue(default)
        return spin

    def _emit_gains(self) -> None:
        self.gains_changed.emit(
            self._kp.value(), self._ki.value(), self._kd.value())

    def _apply_payload(self) -> None:
        self.payload_changed.emit(self._pay_spin.value())
        self._emit_direction()

    def _on_direction_changed(self, index: int) -> None:
        is_angled = self._DIRECTION_LABELS[index][1] == PayloadDirection.ANGLED
        self._tilt_label.setVisible(is_angled)
        self._tilt_slider.setVisible(is_angled)
        self._tilt_spin.setVisible(is_angled)
        self._emit_direction()

    def _emit_direction(self) -> None:
        idx = self._dir_combo.currentIndex()
        direction = self._DIRECTION_LABELS[idx][1].value
        self.direction_changed.emit(direction, self._tilt_spin.value())

    def _emit_injection(self) -> None:
        self.injection_changed.emit({
            "enabled":             self._inj_enable.isChecked(),
            "engagement_distance": self._inj_engagement.value(),
            "release_distance":    self._inj_release.value(),
            "k_approach":          self._inj_k_approach.value(),
            "k_release":           self._inj_k_release.value(),
            "holding_force":       self._inj_holding.value(),
            "shear_thinning":      self._inj_shear.value(),
        })

    def update_gains_display(self, kp: float, ki: float, kd: float) -> None:
        for spin, val in [(self._kp, kp), (self._ki, ki), (self._kd, kd)]:
            spin.blockSignals(True)
            spin.setValue(val)
            spin.blockSignals(False)
