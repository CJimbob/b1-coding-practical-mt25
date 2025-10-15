from dataclasses import dataclass


@dataclass
class PDController:
    """Discrete-time proportional-derivative controller."""

    kp: float = 0.15
    kd: float = 0.6

    def __post_init__(self):
        self._prev_error = 0.0

    def reset(self) -> None:
        """Clear controller state before a new simulation."""
        self._prev_error = 0.0

    def compute(self, reference: float, measurement: float) -> float:
        """Return the control effort for the current reference/measurement pair."""
        error = reference - measurement
        derivative = error - self._prev_error
        self._prev_error = error
        return self.kp * error + self.kd * derivative
