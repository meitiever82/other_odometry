#pragma once
/**
 * @file UpdaterFK.h
 * @brief FK position observation: p + R * FK(q) = p_foot
 */

#include "leg_odometry/state/State.h"
#include "leg_odometry/state/StateHelper.h"

namespace leg_odom {

class UpdaterFK {
 public:
  static void update(State& s, const NoiseParams& noise,
                     const Eigen::Vector3d& fk_left_body,
                     const Eigen::Vector3d& fk_right_body) {
    double sigma = noise.sigma_fk;
    Eigen::Matrix3d R_obs = Eigen::Matrix3d::Identity() * sigma * sigma;

    // Left foot
    {
      Eigen::Vector3d r_world = s.R * fk_left_body;
      Eigen::Vector3d z = s.p + r_world;
      Eigen::Vector3d res = -(z - s.p_fl);

      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, DIM);
      H.block<3,3>(0, P) = Eigen::Matrix3d::Identity();
      H.block<3,3>(0, TH) = -skew(r_world);
      H.block<3,3>(0, FL) = -Eigen::Matrix3d::Identity();

      StateHelper::ekf_update(s, noise, res, H, R_obs);
    }

    // Right foot
    {
      Eigen::Vector3d r_world = s.R * fk_right_body;
      Eigen::Vector3d z = s.p + r_world;
      Eigen::Vector3d res = -(z - s.p_fr);

      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, DIM);
      H.block<3,3>(0, P) = Eigen::Matrix3d::Identity();
      H.block<3,3>(0, TH) = -skew(r_world);
      H.block<3,3>(0, FR) = -Eigen::Matrix3d::Identity();

      StateHelper::ekf_update(s, noise, res, H, R_obs);
    }
  }
};

}  // namespace leg_odom
