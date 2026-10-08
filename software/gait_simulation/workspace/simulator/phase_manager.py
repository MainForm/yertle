import math


class PhaseManager:
    """Manage the phase of a periodic gait cycle."""

    PHASE_PER_CYCLE = 2.0 * math.pi

    def __init__(self, frequency: float, initial_phase: float = 0.0):
        if frequency < 0.0:
            raise ValueError("frequency must be non-negative")

        self._frequency = frequency
        self._phase = initial_phase % self.PHASE_PER_CYCLE

    def step(self, delta_time: float) -> float:
        """Advance the phase by dt seconds."""
        if delta_time < 0.0:
            raise ValueError("dt must be non-negative")

        self._phase = (
            self._phase + self.PHASE_PER_CYCLE * self._frequency * delta_time # 2πf * Δt
        ) % self.PHASE_PER_CYCLE

        return self._phase

    def reset(self, phase: float = 0.0) -> None:
        """Reset the gait phase."""
        self._phase = phase % self.PHASE_PER_CYCLE

    @property
    def phase(self) -> float:
        """Current phase in radians: [0, 2π)."""
        return self._phase

    @property
    def normalized_phase(self) -> float:
        """Current phase normalized to [0, 1)."""
        return self._phase / self.PHASE_PER_CYCLE

    @property
    def frequency(self) -> float:
        return self._frequency

    @frequency.setter
    def frequency(self, value: float) -> None:
        if value < 0.0:
            raise ValueError("frequency must be non-negative")
        self._frequency = value