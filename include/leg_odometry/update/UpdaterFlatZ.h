#pragma once
/**
 * @file UpdaterFlatZ.h
 * @brief Flat ground constraint: p_z ≈ 0, v_z ≈ 0.
 */

#include "leg_odometry/state/State.h"
#include "leg_odometry/state/StateHelper.h"

namespace leg_odom {

class UpdaterFlatZ {
 public:
  static void update(State& s, const NoiseParams& noise) {
    // v_z ≈ 0
    if (noise.sigma_flat_vz > 0) {
      Eigen::MatrixXd H(1, DIM);
      H.setZero();
      H(0, V + 2) = 1.0;
      Eigen::VectorXd res(1);
      res << -s.v(2);
      Eigen::MatrixXd R(1, 1);
      R << noise.sigma_flat_vz * noise.sigma_flat_vz;
      StateHelper::ekf_update(s, noise, res, H, R);
    }

    // p_z ≈ 0
    if (noise.sigma_flat_z > 0) {
      Eigen::MatrixXd H(1, DIM);
      H.setZero();
      H(0, P + 2) = 1.0;
      Eigen::VectorXd res(1);
      res << -s.p(2);
      Eigen::MatrixXd R(1, 1);
      R << noise.sigma_flat_z * noise.sigma_flat_z;
      StateHelper::ekf_update(s, noise, res, H, R);
    }
  }
};

}  // namespace leg_odom
