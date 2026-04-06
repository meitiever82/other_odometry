#pragma once
/**
 * @file Propagator.h
 * @brief IMU-driven state propagation for ESKF.
 * Ref: FAST_LIO use-ikfom.hpp + OpenVINS Propagator.h
 */

#include "State.h"

namespace leg_odom {

class Propagator {
 public:
  static void predict(State& s, const NoiseParams& noise,
                      const Eigen::Vector3d& accel,
                      const Eigen::Vector3d& gyro,
                      double dt,
                      bool contact_left, bool contact_right) {
    if (!s.initialized || dt <= 0 || dt > 0.1) return;

    // De-bias
    Eigen::Vector3d a = accel - s.b_a;
    Eigen::Vector3d w = gyro - s.b_g;
    s.last_gyro_corrected = w;
    s.last_dt = dt;

    // Stillness detection (gyro norm buffer)
    double gnorm = w.norm();
    s.gyro_norm_buffer[s.gyro_buf_idx % 100] = gnorm;
    s.gyro_buf_idx++;
    if (s.gyro_buf_count < 100) s.gyro_buf_count++;
    if (s.gyro_buf_count >= 50) {
      double sum = 0, sum2 = 0;
      int n = std::min(s.gyro_buf_count, 50);
      for (int i = 0; i < n; i++) {
        int idx = (s.gyro_buf_idx - 1 - i + 100) % 100;
        sum += s.gyro_norm_buffer[idx];
        sum2 += s.gyro_norm_buffer[idx] * s.gyro_norm_buffer[idx];
      }
      double mean = sum / n;
      double var = sum2 / n - mean * mean;
      double v_norm = s.v.head<2>().norm();
      s.is_still = (std::sqrt(std::max(var, 0.0)) < 0.005) && (v_norm < 0.05);
    }

    // Nominal state update
    Eigen::Vector3d g(0, 0, -9.81);
    Eigen::Vector3d a_world = s.R * a + g;
    s.p += s.v * dt + 0.5 * a_world * dt * dt;
    s.v += a_world * dt;
    s.R = s.R * exp_so3(w * dt);

    // State transition matrix F (error-state)
    Eigen::Matrix<double, DIM, DIM> F = Eigen::Matrix<double, DIM, DIM>::Identity();
    F.block<3,3>(P, V) = Eigen::Matrix3d::Identity() * dt;
    F.block<3,3>(P, TH) = -0.5 * s.R * skew(a) * dt * dt;
    F.block<3,3>(P, BA) = -0.5 * s.R * dt * dt;
    F.block<3,3>(V, TH) = -s.R * skew(a) * dt;
    F.block<3,3>(V, BA) = -s.R * dt;
    F.block<3,3>(TH, TH) = exp_so3(-w * dt);
    F.block<3,3>(TH, BG) = -Eigen::Matrix3d::Identity() * dt;

    // Process noise Q
    Eigen::Matrix<double, DIM, DIM> Q = Eigen::Matrix<double, DIM, DIM>::Zero();
    Q.block<3,3>(P, P) = Eigen::Matrix3d::Identity() * std::pow(noise.sigma_a * dt, 2);
    Q.block<3,3>(V, V) = Eigen::Matrix3d::Identity() * std::pow(noise.sigma_a * dt, 2);
    Q.block<3,3>(TH, TH) = Eigen::Matrix3d::Identity() * std::pow(noise.sigma_g * dt, 2);
    Q.block<3,3>(BA, BA) = Eigen::Matrix3d::Identity() * std::pow(noise.sigma_ba * std::sqrt(dt), 2);
    Q.block<3,3>(BG, BG) = Eigen::Matrix3d::Identity() * std::pow(noise.sigma_bg * std::sqrt(dt), 2);

    double sig_fl = contact_left ? noise.sigma_contact : noise.sigma_swing;
    double sig_fr = contact_right ? noise.sigma_contact : noise.sigma_swing;
    Q.block<3,3>(FL, FL) = Eigen::Matrix3d::Identity() * std::pow(sig_fl * std::sqrt(dt), 2);
    Q.block<3,3>(FR, FR) = Eigen::Matrix3d::Identity() * std::pow(sig_fr * std::sqrt(dt), 2);

    // Covariance propagation
    s.P = F * s.P * F.transpose() + Q;
    s.P = 0.5 * (s.P + s.P.transpose());  // symmetrize

    // Lock bias covariance if sigma = 0
    if (noise.sigma_ba == 0) {
      s.P.row(BA).setZero(); s.P.row(BA+1).setZero(); s.P.row(BA+2).setZero();
      s.P.col(BA).setZero(); s.P.col(BA+1).setZero(); s.P.col(BA+2).setZero();
    }
    if (noise.sigma_bg == 0) {
      s.P.row(BG).setZero(); s.P.row(BG+1).setZero(); s.P.row(BG+2).setZero();
      s.P.col(BG).setZero(); s.P.col(BG+1).setZero(); s.P.col(BG+2).setZero();
    }
  }
};

}  // namespace leg_odom
