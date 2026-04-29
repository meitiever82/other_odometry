#pragma once
/**
 * @file so3_utils.h
 * @brief Minimal SO(3) helpers: skew-symmetric matrix and exp map.
 */

#include <cmath>
#include <Eigen/Dense>

namespace leg_odom {

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m <<   0.0, -v(2),  v(1),
        v(2),   0.0, -v(0),
       -v(1),  v(0),   0.0;
  return m;
}

inline Eigen::Matrix3d exp_so3(const Eigen::Vector3d& phi) {
  const double angle = phi.norm();
  if (angle < 1e-10) {
    return Eigen::Matrix3d::Identity() + skew(phi);
  }
  const Eigen::Vector3d axis = phi / angle;
  const Eigen::Matrix3d K = skew(axis);
  return Eigen::Matrix3d::Identity()
         + std::sin(angle) * K
         + (1.0 - std::cos(angle)) * K * K;
}

}  // namespace leg_odom
