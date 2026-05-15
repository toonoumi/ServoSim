import math
import threading
from dataclasses import dataclass, field

from .pid import PIDController
from .hall import HallSensor


@dataclass
class MotorParams:
    R: float = 2.5          # winding resistance (Ω)
    L: float = 0.001        # winding inductance (H)
    Kt: float = 0.05        # torque constant (Nm/A)
    Ke: float = 0.05        # back-EMF constant (V·s/rad)
    J: float = 0.0001       # rotor inertia (kg·m²)
    b: float = 0.001        # viscous damping (Nm·s/rad)
    Tc: float = 0.005       # coulomb/static friction (Nm)
    moment_arm: float = 0.1  # payload moment arm (m)
    gear_ratio: float = 1.0
    supply_voltage: float = 24.0


class ServoMotor:
    def __init__(self, params: MotorParams, pid: PIDController, hall: HallSensor):
        self.params = params
        self.pid = pid
        self.hall = hall
        self._lock = threading.Lock()

        # Electrical state
        self.current: float = 0.0
        self.voltage_applied: float = 0.0

        # Mechanical state
        self.position_deg: float = 0.0
        self.omega_rad: float = 0.0

        # Control
        self.target_deg: float = 0.0

        # Payload
        self.payload_mass_kg: float = 0.0

        # Outputs (updated each step)
        self.torque: float = 0.0
        self.power: float = 0.0

    def set_target(self, deg: float) -> None:
        self.target_deg = float(deg) % 360.0

    def set_payload(self, mass_kg: float) -> None:
        self.payload_mass_kg = max(0.0, float(mass_kg))

    def reset(self) -> None:
        with self._lock:
            self.current = 0.0
            self.voltage_applied = 0.0
            self.position_deg = 0.0
            self.omega_rad = 0.0
            self.torque = 0.0
            self.power = 0.0
            self.pid.reset()
            self.hall.update(0.0)

    def step(self, dt: float) -> None:
        # Snapshot mutable control inputs to avoid races during multi-field reads
        target = self.target_deg
        payload_mass = self.payload_mass_kg

        # PID: compute voltage command from position error
        error = target - self.position_deg
        self.voltage_applied = self.pid.update(error, dt)

        # Electrical: exact solution of RL + back-EMF circuit.
        # I(t+dt) = I_ss + (I(t) - I_ss) * exp(-R*dt/L)
        # This is unconditionally stable regardless of dt vs L/R time constant.
        back_emf = self.params.Ke * self.omega_rad
        I_ss = (self.voltage_applied - back_emf) / self.params.R
        decay = math.exp(-self.params.R * dt / self.params.L)
        self.current = I_ss + (self.current - I_ss) * decay

        # Torque components
        T_motor = self.params.Kt * self.current
        T_load = payload_mass * 9.81 * self.params.moment_arm

        omega = self.omega_rad
        if abs(omega) > 1e-4:
            T_friction = self.params.Tc * math.copysign(1.0, omega)
        else:
            # Static friction: resist net torque up to Tc
            net_before_friction = T_motor - T_load
            T_friction = max(-self.params.Tc, min(self.params.Tc, net_before_friction))

        T_damping = self.params.b * omega
        T_net = T_motor - T_load - T_friction - T_damping
        self.torque = T_net

        # Mechanical integration
        alpha = T_net / self.params.J
        self.omega_rad += alpha * dt
        self.position_deg = (self.position_deg + math.degrees(self.omega_rad * dt)) % 360.0

        # Power
        self.power = self.voltage_applied * self.current

        # Hall sensor
        self.hall.update(self.position_deg)
