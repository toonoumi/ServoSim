import sys

from PyQt5.QtWidgets import QApplication

from servosim.physics import MotorParams, ServoMotor
from servosim.pid import PIDController
from servosim.hall import HallSensor
from servosim.socket_server import SocketServer
from servosim.simulator import SimulationLoop
from servosim.gui.main_window import MainWindow


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("ServoSim")
    app.setStyle("Fusion")

    # Build simulation objects
    params = MotorParams()
    pid    = PIDController(kp=10.0, ki=0.5, kd=0.1)
    hall   = HallSensor(poles=7)
    motor  = ServoMotor(params, pid, hall)

    socket_server = SocketServer(motor)
    socket_server.start()

    simulator = SimulationLoop(motor, socket_server)

    window = MainWindow(simulator, motor, socket_server)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
