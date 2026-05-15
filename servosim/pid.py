class PIDController:
    def __init__(self, kp: float = 10.0, ki: float = 0.5, kd: float = 0.1,
                 output_min: float = -24.0, output_max: float = 24.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._output_min = output_min
        self._output_max = output_max
        self._integral: float = 0.0
        self._prev_error: float = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0

    @staticmethod
    def _wrap_error(error: float) -> float:
        return ((error + 180.0) % 360.0) - 180.0

    def update(self, error_raw: float, dt: float) -> float:
        error = self._wrap_error(error_raw)
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        raw = self.kp * error + self.ki * self._integral + self.kd * derivative
        clamped = max(self._output_min, min(self._output_max, raw))
        # Anti-windup: undo integral accumulation when saturated
        if raw != clamped:
            self._integral -= error * dt
        self._prev_error = error
        return clamped

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._prev_error = 0.0
