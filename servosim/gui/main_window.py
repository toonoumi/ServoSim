from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QSplitter, QStatusBar, QLabel,
)

from .plots import PlotPanel
from .controls import ControlPanel, StatusPanel
from ..simulator import SimulationLoop
from ..physics import ServoMotor
from ..socket_server import SocketServer


class MainWindow(QMainWindow):
    def __init__(self, simulator: SimulationLoop, motor: ServoMotor,
                 socket_server: SocketServer):
        super().__init__()
        self._sim = simulator
        self._motor = motor
        self._socket = socket_server
        self._running = False

        self.setWindowTitle("ServoSim")
        self.resize(1200, 700)

        # Build central layout
        central = QWidget()
        self.setCentralWidget(central)
        outer = QHBoxLayout(central)
        outer.setContentsMargins(4, 4, 4, 4)

        splitter = QSplitter(Qt.Horizontal)
        outer.addWidget(splitter)

        self._plots = PlotPanel()
        splitter.addWidget(self._plots)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(4, 4, 4, 4)
        right_layout.setSpacing(6)

        self._controls = ControlPanel()
        self._status   = StatusPanel()
        right_layout.addWidget(self._controls)
        right_layout.addWidget(self._status)
        right_layout.addStretch()
        splitter.addWidget(right)
        splitter.setSizes([780, 380])

        # Status bar
        self._sb_state   = QLabel("Stopped")
        self._sb_tick    = QLabel("t=0 ms")
        self._sb_socket  = QLabel(f"Socket: {socket_server.socket_path}")
        self._sb_clients = QLabel("Clients: 0")
        sb = QStatusBar()
        self.setStatusBar(sb)
        for w in (self._sb_state, self._sb_tick, self._sb_socket, self._sb_clients):
            sb.addWidget(w)
            sb.addWidget(_make_separator())

        # Wire signals
        self._controls.target_changed.connect(self._on_target_changed)
        self._controls.payload_changed.connect(self._on_payload_changed)
        self._controls.direction_changed.connect(self._on_direction_changed)
        self._controls.gains_changed.connect(self._on_gains_changed)
        self._controls.run_clicked.connect(self._on_run)
        self._controls.pause_clicked.connect(self._on_pause)
        self._controls.reset_clicked.connect(self._on_reset)
        self._controls.injection_changed.connect(self._on_injection_changed)
        self._controls.cycle_start_clicked.connect(self._on_cycle_start)
        simulator.telemetry_ready.connect(self._on_telemetry)

    @pyqtSlot(float)
    def _on_target_changed(self, deg: float) -> None:
        self._motor.set_target(deg)

    @pyqtSlot(float)
    def _on_payload_changed(self, mass_kg: float) -> None:
        self._motor.set_payload(mass_kg)

    @pyqtSlot(str, float)
    def _on_direction_changed(self, direction: str, tilt_deg: float) -> None:
        self._motor.set_payload_direction(direction, tilt_deg)

    @pyqtSlot(float, float, float)
    def _on_gains_changed(self, kp: float, ki: float, kd: float) -> None:
        self._motor.pid.set_gains(kp, ki, kd)

    @pyqtSlot(dict)
    def _on_injection_changed(self, params: dict) -> None:
        self._motor.set_injection(**params)

    @pyqtSlot(float)
    def _on_cycle_start(self, hold_time_ms: float) -> None:
        self._sim.start_cycle(hold_time_ms)

    @pyqtSlot()
    def _on_run(self) -> None:
        if not self._running:
            self._sim.start()
            self._running = True
        else:
            self._sim.resume()
        self._sb_state.setText("Running")

    @pyqtSlot()
    def _on_pause(self) -> None:
        self._sim.pause()
        self._sb_state.setText("Paused")

    @pyqtSlot()
    def _on_reset(self) -> None:
        self._sim.reset()
        self._plots.clear_data()
        self._sb_tick.setText("t=0 ms")
        self._sb_state.setText("Paused")

    @pyqtSlot(dict)
    def _on_telemetry(self, data: dict) -> None:
        self._plots.append(data)
        self._status.update_status(data)
        self._sb_tick.setText(f"t={data['timestamp']} ms")
        self._sb_clients.setText(f"Clients: {self._socket.client_count}")

        stage = data.get("cycle_stage", "idle")
        if stage != "idle":
            self._sb_state.setText(f"Running [{stage}]")

        # If gains changed externally (via socket), sync the spinboxes
        self._controls.update_gains_display(data["kp"], data["ki"], data["kd"])

    def closeEvent(self, event):
        self._sim.stop_loop()
        self._sim.wait(2000)
        self._socket.stop()
        event.accept()


def _make_separator() -> QLabel:
    sep = QLabel("|")
    sep.setStyleSheet("color: #666; margin: 0 4px;")
    return sep
