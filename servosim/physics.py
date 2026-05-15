import math
import threading
from dataclasses import dataclass
from enum import Enum

from .pid import PIDController
from .hall import HallSensor


class PayloadDirection(str, Enum):
    HORIZONTAL = "horizontal"   # arm sweeps horizontal plane; zero gravity torque
    VERTICAL   = "vertical"     # arm sweeps vertical plane; gravity = m·g·r·cos(θ)
    ROTATIONAL = "rotational"   # flywheel: adds m·r² to inertia, zero gravity torque
    ANGLED     = "angled"       # tilted plane at tilt_deg from horizontal


@dataclass
class InjectionParams:
    enabled: bool = False
    engagement_distance: float = 30.0  # degrees — resistance starts this far from target
    release_distance: float = 5.0      # degrees — suction disappears beyond this
    k_approach: float = 0.08           # Nm scale for logarithmic approach curve
    k_release: float = 0.04            # Nm constant suction resisting retraction
    holding_force: float = 0.02        # Nm backpressure when stationary near target
    shear_thinning: float = 0.5        # 1/(rad/s): higher speed reduces approach resistance


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
        self.payload_direction: PayloadDirection = PayloadDirection.VERTICAL
        self.payload_tilt_deg: float = 45.0  # used only in ANGLED mode

        # Injection load (stacks with gravity)
        self.injection = InjectionParams()
        self.injection_torque: float = 0.0
        self.cavity_pressure: float = 0.0

        # Outputs (updated each step)
        self.torque: float = 0.0
        self.power: float = 0.0
        self.gravity_torque: float = 0.0

    def set_target(self, deg: float) -> None:
        self.target_deg = float(deg) % 360.0

    def set_payload(self, mass_kg: float) -> None:
        self.payload_mass_kg = max(0.0, float(mass_kg))

    def set_injection(self, **kwargs) -> None:
        for k, v in kwargs.items():
            if hasattr(self.injection, k):
                cur = getattr(self.injection, k)
                setattr(self.injection, k, type(cur)(v))

    def set_payload_direction(self, direction: PayloadDirection,
                               tilt_deg: float | None = None) -> None:
        self.payload_direction = PayloadDirection(direction)
        if tilt_deg is not None:
            self.payload_tilt_deg = float(tilt_deg)

    def reset(self) -> None:
        with self._lock:
            self.current = 0.0
            self.voltage_applied = 0.0
            self.position_deg = 0.0
            self.omega_rad = 0.0
            self.torque = 0.0
            self.power = 0.0
            self.injection_torque = 0.0
            self.cavity_pressure = 0.0
            self.pid.reset()
            self.hall.update(0.0)

    def step(self, dt: float) -> None:
        # Snapshot mutable control inputs to avoid races during multi-field reads
        target    = self.target_deg
        mass      = self.payload_mass_kg
        direction = self.payload_direction
        tilt_deg  = self.payload_tilt_deg
        pos_deg   = self.position_deg

        # PID: compute voltage command from position error
        error = target - pos_deg
        self.voltage_applied = self.pid.update(error, dt)

        # Electrical: exact solution of RL + back-EMF circuit.
        # I(t+dt) = I_ss + (I(t) - I_ss) * exp(-R*dt/L)
        # Unconditionally stable regardless of dt vs L/R time constant.
        back_emf = self.params.Ke * self.omega_rad
        I_ss = (self.voltage_applied - back_emf) / self.params.R
        decay = math.exp(-self.params.R * dt / self.params.L)
        self.current = I_ss + (self.current - I_ss) * decay

        # Gravity torque — signed, position-dependent.
        # Convention: θ=0 → arm horizontal; positive torque = CCW.
        # T_gravity = m·g·scale·r·cos(θ), scale depends on plane orientation.
        pos_rad = math.radians(pos_deg)
        arm = self.params.moment_arm
        if direction in (PayloadDirection.HORIZONTAL, PayloadDirection.ROTATIONAL):
            T_gravity = 0.0
        elif direction == PayloadDirection.VERTICAL:
            T_gravity = mass * 9.81 * arm * math.cos(pos_rad)
        else:  # ANGLED: tilt_deg=0 → horizontal, tilt_deg=90 → vertical
            T_gravity = mass * 9.81 * math.sin(math.radians(tilt_deg)) * arm * math.cos(pos_rad)
        self.gravity_torque = T_gravity

        # Effective inertia: ROTATIONAL mode adds payload moment of inertia
        J_eff = self.params.J
        if direction == PayloadDirection.ROTATIONAL:
            J_eff += mass * arm ** 2

        # Motor torque
        T_motor = self.params.Kt * self.current
        omega = self.omega_rad

        # Injection load (stacks with gravity)
        inj = self.injection
        if inj.enabled:
            _EPS = 0.01  # degrees — avoids log(0)
            d_signed = ((target - pos_deg + 180.0) % 360.0) - 180.0
            d_abs = abs(d_signed)
            sign_to_tgt = math.copysign(1.0, d_signed) if d_abs > 1e-6 else 0.0
            omega_abs = abs(omega)
            T_inj = 0.0
            if omega_abs > 1e-4:
                sign_mot = math.copysign(1.0, omega)
                if sign_mot == sign_to_tgt and d_abs < inj.engagement_distance:
                    ratio = max(d_abs, _EPS) / inj.engagement_distance
                    mag = inj.k_approach * (-math.log(ratio))
                    mag *= 1.0 / (1.0 + inj.shear_thinning * omega_abs)
                    T_inj = -sign_to_tgt * mag
                elif sign_mot != sign_to_tgt and d_abs < inj.release_distance:
                    T_inj = sign_to_tgt * inj.k_release
            elif d_abs < inj.release_distance:
                T_inj = -sign_to_tgt * inj.holding_force
            self.injection_torque = T_inj
            if d_abs < inj.engagement_distance:
                self.cavity_pressure = max(0.0, -math.log(max(d_abs, _EPS) / inj.engagement_distance))
            else:
                self.cavity_pressure = 0.0
        else:
            self.injection_torque = 0.0
            self.cavity_pressure = 0.0

        # Friction + damping
        if abs(omega) > 1e-4:
            T_friction = self.params.Tc * math.copysign(1.0, omega)
        else:
            net_before_friction = T_motor - T_gravity - self.injection_torque
            T_friction = max(-self.params.Tc, min(self.params.Tc, net_before_friction))

        T_damping = self.params.b * omega
        T_net = T_motor - T_gravity - self.injection_torque - T_friction - T_damping
        self.torque = T_net

        # Mechanical integration
        alpha = T_net / J_eff
        self.omega_rad += alpha * dt
        self.position_deg = (pos_deg + math.degrees(self.omega_rad * dt)) % 360.0

        # Power
        self.power = self.voltage_applied * self.current

        # Hall sensor
        self.hall.update(self.position_deg)
