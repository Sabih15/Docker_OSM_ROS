"""Tests for KalmanFilter1D covering initialization, the predict/update
steps in isolation, trust-weighting edge cases, convergence/steady-state
behaviour, noise rejection, target tracking, and instance independence.

Mirrors test_kalman_filter_1d.cpp so both language implementations are
checked against the same scenarios.
"""

import math
import random

import pytest

from teleop_localization_filters.kalman_filter_1d import KalmanFilter1D


# ---- Construction / initial state ------------------------------------------

def test_initial_state_matches_constructor_args():
    kf = KalmanFilter1D(initial_value=10.0, initial_variance=4.0,
                         process_variance=0.1, measurement_variance=1.0)
    assert kf.x == 10.0
    assert kf.p == 4.0
    assert kf.q == 0.1
    assert kf.r == 1.0


# ---- Predict step (in isolation, no update) --------------------------------

def test_predict_advances_state_by_velocity_times_dt():
    kf = KalmanFilter1D(0.0, 1.0, 0.01, 1.0)
    kf.predict(velocity=2.0, dt=0.5)
    assert kf.x == pytest.approx(1.0)


def test_predict_with_zero_velocity_does_not_move_state():
    kf = KalmanFilter1D(5.0, 1.0, 0.01, 1.0)
    kf.predict(velocity=0.0, dt=1.0)
    assert kf.x == pytest.approx(5.0)


def test_predict_with_negative_velocity_moves_state_backward():
    kf = KalmanFilter1D(0.0, 1.0, 0.01, 1.0)
    kf.predict(velocity=-3.0, dt=2.0)
    assert kf.x == pytest.approx(-6.0)


def test_predict_grows_variance_by_process_noise_times_dt():
    kf = KalmanFilter1D(0.0, 1.0, 0.2, 1.0)
    kf.predict(velocity=0.0, dt=1.0)
    assert kf.p == pytest.approx(1.2)


def test_predict_with_zero_dt_is_a_no_op():
    kf = KalmanFilter1D(3.0, 2.0, 0.5, 1.0)
    kf.predict(velocity=100.0, dt=0.0)
    assert kf.x == pytest.approx(3.0)
    assert kf.p == pytest.approx(2.0)


def test_predict_returns_the_updated_state():
    kf = KalmanFilter1D(0.0, 1.0, 0.0, 1.0)
    result = kf.predict(velocity=1.0, dt=1.0)
    assert result == kf.x


def test_repeated_predicts_accumulate():
    kf = KalmanFilter1D(0.0, 1.0, 0.1, 1.0)
    for _ in range(10):
        kf.predict(velocity=1.0, dt=1.0)
    assert kf.x == pytest.approx(10.0)
    assert kf.p == pytest.approx(1.0 + 10 * 0.1)


# ---- Update step (in isolation, no predict) --------------------------------

def test_update_with_measurement_equal_to_state_leaves_state_unchanged():
    kf = KalmanFilter1D(4.0, 1.0, 0.0, 1.0)
    kf.update(4.0)
    assert kf.x == pytest.approx(4.0)


def test_update_moves_state_toward_measurement_not_past_it():
    kf = KalmanFilter1D(0.0, 1.0, 0.0, 1.0)
    kf.update(10.0)
    assert 0.0 < kf.x < 10.0


def test_update_gain_matches_the_kalman_gain_formula():
    p, r = 3.0, 1.0
    kf = KalmanFilter1D(0.0, p, 0.0, r)
    kf.update(4.0)
    expected_k = p / (p + r)
    expected_x = 0.0 + expected_k * (4.0 - 0.0)
    assert kf.x == pytest.approx(expected_x)


def test_update_shrinks_variance_when_measurement_noise_is_finite():
    kf = KalmanFilter1D(0.0, 1.0, 0.0, 1.0)
    kf.update(5.0)
    assert kf.p < 1.0


def test_update_returns_the_updated_state():
    kf = KalmanFilter1D(0.0, 1.0, 0.0, 1.0)
    result = kf.update(2.0)
    assert result == kf.x


# ---- Trust-weighting edge cases ---------------------------------------------

def test_near_zero_measurement_noise_snaps_almost_exactly_to_measurement():
    kf = KalmanFilter1D(0.0, 1.0, 0.0, 1e-12)
    kf.update(100.0)
    assert kf.x == pytest.approx(100.0, abs=1e-6)


def test_very_large_measurement_noise_barely_moves_the_state():
    kf = KalmanFilter1D(0.0, 1.0, 0.0, 1e12)
    kf.update(100.0)
    assert kf.x == pytest.approx(0.0, abs=1e-6)


def test_larger_prediction_uncertainty_trusts_the_measurement_more():
    kf_confident = KalmanFilter1D(0.0, 0.01, 0.0, 1.0)
    kf_unsure = KalmanFilter1D(0.0, 100.0, 0.0, 1.0)
    kf_confident.update(10.0)
    kf_unsure.update(10.0)
    assert abs(kf_unsure.x - 10.0) < abs(kf_confident.x - 10.0)


# ---- Convergence & steady state ----------------------------------------------

def test_repeated_noise_free_updates_converge_exactly_to_true_value():
    true_value = 42.0
    kf = KalmanFilter1D(0.0, 10.0, 0.01, 1.0)
    for _ in range(200):
        kf.predict(0.0, 1.0)
        kf.update(true_value)
    assert kf.x == pytest.approx(true_value, abs=1e-3)


def test_variance_converges_to_the_analytical_steady_state():
    """For constant q, r the recursion p_{k+1} = (p_k+q)*r/(p_k+q+r)
    has a fixed point solving p^2 + q*p - q*r = 0, i.e.
    p_ss = (sqrt(q^2 + 4qr) - q) / 2 (positive root)."""
    q, r = 0.2, 1.0
    p_ss = (math.sqrt(q ** 2 + 4 * q * r) - q) / 2.0

    kf = KalmanFilter1D(0.0, 5.0, q, r)
    for _ in range(500):
        kf.predict(0.0, 1.0)
        kf.update(0.0)

    assert kf.p == pytest.approx(p_ss, rel=1e-3)


def test_variance_never_goes_negative_or_diverges():
    kf = KalmanFilter1D(0.0, 1.0, 0.05, 0.5)
    rng = random.Random(1)
    for _ in range(2000):
        kf.predict(velocity=rng.uniform(-1, 1), dt=0.1)
        kf.update(rng.gauss(0.0, 1.0))
        assert kf.p > 0.0
        assert math.isfinite(kf.p)
        assert math.isfinite(kf.x)


# ---- Noise rejection (statistical, fixed seed for reproducibility) ----------

def test_filtered_rmse_beats_raw_measurement_rmse_on_a_static_signal():
    rng = random.Random(7)
    true_value = 5.0
    noise_std = 1.0
    kf = KalmanFilter1D(true_value, noise_std ** 2, 0.01, noise_std ** 2)

    raw_sq_err = filt_sq_err = 0.0
    n = 300
    for _ in range(n):
        kf.predict(0.0, 1.0)
        meas = true_value + rng.gauss(0.0, noise_std)
        est = kf.update(meas)
        raw_sq_err += (meas - true_value) ** 2
        filt_sq_err += (est - true_value) ** 2

    raw_rmse = math.sqrt(raw_sq_err / n)
    filt_rmse = math.sqrt(filt_sq_err / n)
    assert filt_rmse < raw_rmse * 0.6


# ---- Tracking a moving target (bias / lag) -----------------------------------

def test_filter_tracks_a_constant_velocity_target_without_steady_bias():
    rng = random.Random(3)
    velocity = 0.5
    noise_std = 0.3
    dt = 0.1
    kf = KalmanFilter1D(0.0, noise_std ** 2, 0.05, noise_std ** 2)

    true_pos = 0.0
    for _ in range(500):
        true_pos += velocity * dt
        kf.predict(velocity, dt)
        meas = true_pos + rng.gauss(0.0, noise_std)
        kf.update(meas)

    assert kf.x == pytest.approx(true_pos, abs=1.0)


def test_filter_tracks_the_real_nodes_figure_eight_trajectory():
    """Replays localization_node.py's actual scenario end to end: same
    rate, velocity limits, and noise/process-noise parameters."""
    rng = random.Random(42)

    rate_hz = 30.0
    dt = 1.0 / rate_hz
    max_lin = 0.2
    max_ang = 0.2
    lat0 = 52.139200
    gps_noise_std_m = 2.5
    process_noise_std_m = 0.5
    earth_radius_m = 6_378_137.0
    m_per_deg_lat = (math.pi / 180.0) * earth_radius_m

    meas_var = (gps_noise_std_m / m_per_deg_lat) ** 2
    proc_var = (process_noise_std_m / m_per_deg_lat) ** 2
    kf = KalmanFilter1D(lat0, meas_var, proc_var, meas_var)

    t = theta = y = 0.0
    raw_sq_err = filt_sq_err = 0.0
    steps = int(60.0 * rate_hz)

    for _ in range(steps):
        t += dt
        w = max_ang * math.sin(0.10 * t)
        theta += w * dt
        theta = math.atan2(math.sin(theta), math.cos(theta))
        y += max_lin * math.sin(theta) * dt

        lat_true = lat0 + y / m_per_deg_lat
        lat_vel_deg_s = (max_lin * math.sin(theta)) / m_per_deg_lat

        kf.predict(lat_vel_deg_s, dt)
        lat_meas = lat_true + rng.gauss(0.0, math.sqrt(meas_var))
        lat_filt = kf.update(lat_meas)

        raw_sq_err += ((lat_meas - lat_true) * m_per_deg_lat) ** 2
        filt_sq_err += ((lat_filt - lat_true) * m_per_deg_lat) ** 2

    raw_rmse = math.sqrt(raw_sq_err / steps)
    filt_rmse = math.sqrt(filt_sq_err / steps)

    assert filt_rmse < raw_rmse
    assert filt_rmse < gps_noise_std_m


# ---- Independence between instances ------------------------------------------

def test_two_instances_do_not_share_state():
    kf_lat = KalmanFilter1D(1.0, 1.0, 0.1, 1.0)
    kf_lon = KalmanFilter1D(2.0, 1.0, 0.1, 1.0)
    kf_lat.predict(5.0, 1.0)
    kf_lat.update(100.0)
    assert kf_lon.x == pytest.approx(2.0)
    assert kf_lon.p == pytest.approx(1.0)
