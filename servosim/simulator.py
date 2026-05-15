import time

from PyQt5.QtCore import QThread, pyqtSignal

from .physics import ServoMotor
from .socket_server import SocketServer

PHYSICS_DT = 0.001    # 1ms per physics tick
REPORT_EVERY = 10     # emit telemetry every 10 ticks = 10ms
SPIN_THRESHOLD = 0.0002  # spin-wait below 0.2ms for timing accuracy


class SimulationLoop(QThread):
    telemetry_ready = pyqtSignal(dict)

    def __init__(self, motor: ServoMotor, socket_server: SocketServer):
        super().__init__()
        self._motor = motor
        self._socket = socket_server
        self._paused = False
        self._stop_flag = False
        self._tick: int = 0

    def run(self) -> None:
        self._stop_flag = False
        self._tick = 0
        t_next = time.perf_counter()

        while not self._stop_flag:
            if self._paused:
                time.sleep(0.005)
                t_next = time.perf_counter()
                continue

            self._motor.step(PHYSICS_DT)
            self._tick += 1

            if self._tick % REPORT_EVERY == 0:
                data = self._build_telemetry()
                self._socket.push_telemetry(data)
                self.telemetry_ready.emit(data)

            t_next += PHYSICS_DT
            remaining = t_next - time.perf_counter()
            if remaining > SPIN_THRESHOLD:
                time.sleep(remaining - SPIN_THRESHOLD)
            # Spin-wait for the remainder for tighter timing
            while time.perf_counter() < t_next:
                pass

    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False

    def stop_loop(self) -> None:
        self._stop_flag = True

    def reset(self) -> None:
        was_paused = self._paused
        self._paused = True
        time.sleep(0.005)
        self._motor.reset()
        self._tick = 0
        if not was_paused:
            self._paused = False

    def _build_telemetry(self) -> dict:
        m = self._motor
        return {
            "timestamp": self._tick,
            "position": round(m.position_deg, 4),
            "target": round(m.target_deg, 4),
            "position_error": round(m.target_deg - m.position_deg, 4),
            "voltage": round(m.voltage_applied, 4),
            "current": round(m.current, 6),
            "power": round(m.power, 4),
            "torque": round(m.torque, 6),
            "gravity_torque": round(m.gravity_torque, 6),
            "kp": m.pid.kp,
            "ki": m.pid.ki,
            "kd": m.pid.kd,
            "hall_a": m.hall.hall_a,
            "hall_b": m.hall.hall_b,
            "hall_c": m.hall.hall_c,
            "payload_direction": m.payload_direction.value,
            "payload_tilt_deg": m.payload_tilt_deg,
        }
