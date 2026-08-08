"""Minimal scalar Kalman filter.

Tracks a single quantity (e.g. one geographic axis) through a constant-
velocity predict step followed by a measurement update. Two independent
instances -- one per axis -- give a "1D x2" filter for a lat/lon pair,
since latitude and longitude noise are uncorrelated for this demo's
equirectangular projection.
"""


class KalmanFilter1D:
    def __init__(
        self,
        initial_value: float,
        initial_variance: float,
        process_variance: float,
        measurement_variance: float,
    ) -> None:
        self.x = initial_value
        self.p = initial_variance
        self.q = process_variance
        self.r = measurement_variance

    def predict(self, velocity: float, dt: float) -> float:
        """Advance the estimate by `velocity * dt` and grow the uncertainty."""
        self.x += velocity * dt
        self.p += self.q * dt
        return self.x

    def update(self, measurement: float) -> float:
        """Correct the estimate toward a noisy measurement."""
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1.0 - k)
        return self.x
