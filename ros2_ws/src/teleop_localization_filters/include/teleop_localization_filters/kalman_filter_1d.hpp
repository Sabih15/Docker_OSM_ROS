// Minimal scalar Kalman filter -- header-only C++ counterpart to
// teleop_localization_filters/kalman_filter_1d.py.
//
// Tracks a single quantity through a constant-velocity predict step
// followed by a measurement update. Two independent instances -- one per
// axis -- give a "1D x2" filter for a lat/lon pair. Not yet wired into
// teleop_localization_cpp; available for when that node needs it.

#pragma once

namespace teleop_localization_filters
{

class KalmanFilter1D
{
public:
  KalmanFilter1D(
    double initial_value, double initial_variance,
    double process_variance, double measurement_variance)
  : x_(initial_value), p_(initial_variance), q_(process_variance), r_(measurement_variance)
  {}

  double predict(double velocity, double dt)
  {
    x_ += velocity * dt;
    p_ += q_ * dt;
    return x_;
  }

  double update(double measurement)
  {
    const double k = p_ / (p_ + r_);
    x_ += k * (measurement - x_);
    p_ *= (1.0 - k);
    return x_;
  }

  double value() const {return x_;}
  double covariance() const {return p_;}

private:
  double x_, p_, q_, r_;
};

}  // namespace teleop_localization_filters
