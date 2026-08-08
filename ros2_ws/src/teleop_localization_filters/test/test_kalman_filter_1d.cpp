// Tests for KalmanFilter1D covering initialization, the predict/update
// steps in isolation, trust-weighting edge cases, convergence/steady-state
// behaviour, noise rejection, target tracking, and instance independence.
//
// Mirrors test_kalman_filter_1d.py so both language implementations are
// checked against the same scenarios.

#include <cmath>
#include <random>

#include "gtest/gtest.h"
#include "teleop_localization_filters/kalman_filter_1d.hpp"

using teleop_localization_filters::KalmanFilter1D;

namespace
{
constexpr double kTol = 1e-9;
}  // namespace

// ---- Construction / initial state ------------------------------------------

TEST(KalmanFilter1D, InitialStateMatchesConstructorArgs)
{
  KalmanFilter1D kf(10.0, 4.0, 0.1, 1.0);
  EXPECT_DOUBLE_EQ(kf.value(), 10.0);
  EXPECT_DOUBLE_EQ(kf.covariance(), 4.0);
}

// ---- Predict step (in isolation, no update) --------------------------------

TEST(KalmanFilter1D, PredictAdvancesStateByVelocityTimesDt)
{
  KalmanFilter1D kf(0.0, 1.0, 0.01, 1.0);
  kf.predict(2.0, 0.5);
  EXPECT_NEAR(kf.value(), 1.0, kTol);
}

TEST(KalmanFilter1D, PredictWithZeroVelocityDoesNotMoveState)
{
  KalmanFilter1D kf(5.0, 1.0, 0.01, 1.0);
  kf.predict(0.0, 1.0);
  EXPECT_NEAR(kf.value(), 5.0, kTol);
}

TEST(KalmanFilter1D, PredictWithNegativeVelocityMovesStateBackward)
{
  KalmanFilter1D kf(0.0, 1.0, 0.01, 1.0);
  kf.predict(-3.0, 2.0);
  EXPECT_NEAR(kf.value(), -6.0, kTol);
}

TEST(KalmanFilter1D, PredictGrowsVarianceByProcessNoiseTimesDt)
{
  KalmanFilter1D kf(0.0, 1.0, 0.2, 1.0);
  kf.predict(0.0, 1.0);
  EXPECT_NEAR(kf.covariance(), 1.2, kTol);
}

TEST(KalmanFilter1D, PredictWithZeroDtIsNoOp)
{
  KalmanFilter1D kf(3.0, 2.0, 0.5, 1.0);
  kf.predict(100.0, 0.0);
  EXPECT_NEAR(kf.value(), 3.0, kTol);
  EXPECT_NEAR(kf.covariance(), 2.0, kTol);
}

TEST(KalmanFilter1D, PredictReturnsTheUpdatedState)
{
  KalmanFilter1D kf(0.0, 1.0, 0.0, 1.0);
  const double result = kf.predict(1.0, 1.0);
  EXPECT_DOUBLE_EQ(result, kf.value());
}

TEST(KalmanFilter1D, RepeatedPredictsAccumulate)
{
  KalmanFilter1D kf(0.0, 1.0, 0.1, 1.0);
  for (int i = 0; i < 10; ++i) {
    kf.predict(1.0, 1.0);
  }
  EXPECT_NEAR(kf.value(), 10.0, kTol);
  EXPECT_NEAR(kf.covariance(), 1.0 + 10 * 0.1, kTol);
}

// ---- Update step (in isolation, no predict) --------------------------------

TEST(KalmanFilter1D, UpdateWithMeasurementEqualToStateLeavesStateUnchanged)
{
  KalmanFilter1D kf(4.0, 1.0, 0.0, 1.0);
  kf.update(4.0);
  EXPECT_NEAR(kf.value(), 4.0, kTol);
}

TEST(KalmanFilter1D, UpdateMovesStateTowardMeasurementNotPastIt)
{
  KalmanFilter1D kf(0.0, 1.0, 0.0, 1.0);
  kf.update(10.0);
  EXPECT_GT(kf.value(), 0.0);
  EXPECT_LT(kf.value(), 10.0);
}

TEST(KalmanFilter1D, UpdateGainMatchesTheKalmanGainFormula)
{
  const double p = 3.0;
  const double r = 1.0;
  KalmanFilter1D kf(0.0, p, 0.0, r);
  kf.update(4.0);
  const double expected_k = p / (p + r);
  const double expected_x = expected_k * 4.0;
  EXPECT_NEAR(kf.value(), expected_x, kTol);
}

TEST(KalmanFilter1D, UpdateShrinksVarianceWhenMeasurementNoiseIsFinite)
{
  KalmanFilter1D kf(0.0, 1.0, 0.0, 1.0);
  kf.update(5.0);
  EXPECT_LT(kf.covariance(), 1.0);
}

TEST(KalmanFilter1D, UpdateReturnsTheUpdatedState)
{
  KalmanFilter1D kf(0.0, 1.0, 0.0, 1.0);
  const double result = kf.update(2.0);
  EXPECT_DOUBLE_EQ(result, kf.value());
}

// ---- Trust-weighting edge cases ---------------------------------------------

TEST(KalmanFilter1D, NearZeroMeasurementNoiseSnapsAlmostExactlyToMeasurement)
{
  KalmanFilter1D kf(0.0, 1.0, 0.0, 1e-12);
  kf.update(100.0);
  EXPECT_NEAR(kf.value(), 100.0, 1e-6);
}

TEST(KalmanFilter1D, VeryLargeMeasurementNoiseBarelyMovesTheState)
{
  KalmanFilter1D kf(0.0, 1.0, 0.0, 1e12);
  kf.update(100.0);
  EXPECT_NEAR(kf.value(), 0.0, 1e-6);
}

TEST(KalmanFilter1D, LargerPredictionUncertaintyTrustsTheMeasurementMore)
{
  KalmanFilter1D kf_confident(0.0, 0.01, 0.0, 1.0);
  KalmanFilter1D kf_unsure(0.0, 100.0, 0.0, 1.0);
  kf_confident.update(10.0);
  kf_unsure.update(10.0);
  EXPECT_LT(std::abs(kf_unsure.value() - 10.0), std::abs(kf_confident.value() - 10.0));
}

// ---- Convergence & steady state -----------------------------------------------

TEST(KalmanFilter1D, RepeatedNoiseFreeUpdatesConvergeExactlyToTrueValue)
{
  const double true_value = 42.0;
  KalmanFilter1D kf(0.0, 10.0, 0.01, 1.0);
  for (int i = 0; i < 200; ++i) {
    kf.predict(0.0, 1.0);
    kf.update(true_value);
  }
  EXPECT_NEAR(kf.value(), true_value, 1e-3);
}

TEST(KalmanFilter1D, VarianceConvergesToTheAnalyticalSteadyState)
{
  // For constant q, r the recursion p_{k+1} = (p_k+q)*r/(p_k+q+r) has a
  // fixed point solving p^2 + q*p - q*r = 0, i.e.
  // p_ss = (sqrt(q^2 + 4qr) - q) / 2 (positive root).
  const double q = 0.2;
  const double r = 1.0;
  const double p_ss = (std::sqrt(q * q + 4 * q * r) - q) / 2.0;

  KalmanFilter1D kf(0.0, 5.0, q, r);
  for (int i = 0; i < 500; ++i) {
    kf.predict(0.0, 1.0);
    kf.update(0.0);
  }

  EXPECT_NEAR(kf.covariance(), p_ss, p_ss * 1e-3);
}

TEST(KalmanFilter1D, VarianceNeverGoesNegativeOrDiverges)
{
  KalmanFilter1D kf(0.0, 1.0, 0.05, 0.5);
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> vel_dist(-1.0, 1.0);
  std::normal_distribution<double> meas_dist(0.0, 1.0);

  for (int i = 0; i < 2000; ++i) {
    kf.predict(vel_dist(rng), 0.1);
    kf.update(meas_dist(rng));
    ASSERT_GT(kf.covariance(), 0.0);
    ASSERT_TRUE(std::isfinite(kf.covariance()));
    ASSERT_TRUE(std::isfinite(kf.value()));
  }
}

// ---- Noise rejection (statistical, fixed seed for reproducibility) ----------

TEST(KalmanFilter1D, FilteredRmseBeatsRawMeasurementRmseOnAStaticSignal)
{
  std::mt19937 rng(7);
  const double true_value = 5.0;
  const double noise_std = 1.0;
  std::normal_distribution<double> noise(0.0, noise_std);

  KalmanFilter1D kf(true_value, noise_std * noise_std, 0.01, noise_std * noise_std);

  double raw_sq_err = 0.0;
  double filt_sq_err = 0.0;
  const int n = 300;
  for (int i = 0; i < n; ++i) {
    kf.predict(0.0, 1.0);
    const double meas = true_value + noise(rng);
    const double est = kf.update(meas);
    raw_sq_err += (meas - true_value) * (meas - true_value);
    filt_sq_err += (est - true_value) * (est - true_value);
  }

  const double raw_rmse = std::sqrt(raw_sq_err / n);
  const double filt_rmse = std::sqrt(filt_sq_err / n);
  EXPECT_LT(filt_rmse, raw_rmse * 0.6);
}

// ---- Tracking a moving target (bias / lag) -------------------------------------

TEST(KalmanFilter1D, FilterTracksAConstantVelocityTargetWithoutSteadyBias)
{
  std::mt19937 rng(3);
  const double velocity = 0.5;
  const double noise_std = 0.3;
  const double dt = 0.1;
  std::normal_distribution<double> noise(0.0, noise_std);

  KalmanFilter1D kf(0.0, noise_std * noise_std, 0.05, noise_std * noise_std);

  double true_pos = 0.0;
  for (int i = 0; i < 500; ++i) {
    true_pos += velocity * dt;
    kf.predict(velocity, dt);
    const double meas = true_pos + noise(rng);
    kf.update(meas);
  }

  EXPECT_NEAR(kf.value(), true_pos, 1.0);
}

TEST(KalmanFilter1D, FilterTracksTheRealNodesFigureEightTrajectory)
{
  // Replays localization_node.cpp's actual scenario end to end: same
  // rate, velocity limits, and noise/process-noise parameters as the
  // Python node (teleop_localization_cpp doesn't use this filter yet,
  // but the parameters are shared across both implementations).
  std::mt19937 rng(42);

  const double rate_hz = 30.0;
  const double dt = 1.0 / rate_hz;
  const double max_lin = 0.2;
  const double max_ang = 0.2;
  const double lat0 = 52.139200;
  const double gps_noise_std_m = 2.5;
  const double process_noise_std_m = 0.5;
  const double earth_radius_m = 6378137.0;
  const double m_per_deg_lat = (M_PI / 180.0) * earth_radius_m;

  const double meas_var = std::pow(gps_noise_std_m / m_per_deg_lat, 2);
  const double proc_var = std::pow(process_noise_std_m / m_per_deg_lat, 2);
  KalmanFilter1D kf(lat0, meas_var, proc_var, meas_var);
  std::normal_distribution<double> noise(0.0, std::sqrt(meas_var));

  double t = 0.0;
  double theta = 0.0;
  double y = 0.0;
  double raw_sq_err = 0.0;
  double filt_sq_err = 0.0;
  const int steps = static_cast<int>(60.0 * rate_hz);

  for (int i = 0; i < steps; ++i) {
    t += dt;
    const double w = max_ang * std::sin(0.10 * t);
    theta += w * dt;
    theta = std::atan2(std::sin(theta), std::cos(theta));
    y += max_lin * std::sin(theta) * dt;

    const double lat_true = lat0 + y / m_per_deg_lat;
    const double lat_vel_deg_s = (max_lin * std::sin(theta)) / m_per_deg_lat;

    kf.predict(lat_vel_deg_s, dt);
    const double lat_meas = lat_true + noise(rng);
    const double lat_filt = kf.update(lat_meas);

    raw_sq_err += std::pow((lat_meas - lat_true) * m_per_deg_lat, 2);
    filt_sq_err += std::pow((lat_filt - lat_true) * m_per_deg_lat, 2);
  }

  const double raw_rmse = std::sqrt(raw_sq_err / steps);
  const double filt_rmse = std::sqrt(filt_sq_err / steps);

  EXPECT_LT(filt_rmse, raw_rmse);
  EXPECT_LT(filt_rmse, gps_noise_std_m);
}

// ---- Independence between instances ---------------------------------------------

TEST(KalmanFilter1D, TwoInstancesDoNotShareState)
{
  KalmanFilter1D kf_lat(1.0, 1.0, 0.1, 1.0);
  KalmanFilter1D kf_lon(2.0, 1.0, 0.1, 1.0);
  kf_lat.predict(5.0, 1.0);
  kf_lat.update(100.0);
  EXPECT_NEAR(kf_lon.value(), 2.0, kTol);
  EXPECT_NEAR(kf_lon.covariance(), 1.0, kTol);
}
