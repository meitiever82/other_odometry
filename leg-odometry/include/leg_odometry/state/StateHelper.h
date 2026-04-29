#pragma once
/**
 * @file StateHelper.h
 * @brief EKF update helper: Kalman gain computation + state injection.
 * Ref: OpenVINS StateHelper.h (simplified, no marginalization)
 */

#include "State.h"

namespace leg_odom {

class StateHelper {
 public:
  /**
   * @brief Generic EKF update: any observation dimension.
   * @param s State
   * @param noise NoiseParams (for bias locking)
   * @param residual (m x 1) residual vector
   * @param H (m x DIM) Jacobian matrix
   * @param R (m x m) observation noise covariance
   */
  static void ekf_update(State& s, const NoiseParams& noise,
                          const Eigen::VectorXd& residual,
                          const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& R) {
    if (!residual.allFinite()) return;

    int m = residual.size();
    Eigen::MatrixXd S = H * s.P * H.transpose() + R;
    Eigen::MatrixXd K;

    // Use Cholesky for numerical stability
    Eigen::LLT<Eigen::MatrixXd> llt(S);
    if (llt.info() != Eigen::Success) return;
    K = s.P * H.transpose() * llt.solve(Eigen::MatrixXd::Identity(m, m));

    Eigen::VectorXd dx = K * residual;
    if (!dx.allFinite()) return;

    // Clamp position correction
    double dp_norm = dx.segment<3>(P).norm();
    if (dp_norm > 1.0) {
      dx *= 1.0 / dp_norm;
    }

    // Lock bias
    if (noise.sigma_ba == 0) dx.segment<3>(BA).setZero();
    if (noise.sigma_bg == 0) dx.segment<3>(BG).setZero();

    // Inject error-state into nominal state
    s.p += dx.segment<3>(P);
    s.v += dx.segment<3>(V);
    s.R = s.R * exp_so3(dx.segment<3>(TH));
    s.b_a += dx.segment<3>(BA);
    s.b_g += dx.segment<3>(BG);
    s.p_fl += dx.segment<3>(FL);
    s.p_fr += dx.segment<3>(FR);

    // Joseph-form covariance update
    Eigen::Matrix<double, DIM, DIM> I_KH =
        Eigen::Matrix<double, DIM, DIM>::Identity() - K * H;
    s.P = I_KH * s.P * I_KH.transpose() + K * R * K.transpose();
    s.P = 0.5 * (s.P + s.P.transpose());
  }
};

}  // namespace leg_odom
