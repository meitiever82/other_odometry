#pragma once
/**
 * @file State.h
 * @brief 21-DOF Error-State for legged robot odometry.
 *
 * Nominal state:
 *   p (3)    - position in world frame
 *   v (3)    - velocity in world frame
 *   R (3x3)  - rotation matrix (SO3)
 *   b_a (3)  - accelerometer bias
 *   b_g (3)  - gyroscope bias
 *   p_fl (3) - left foot position in world frame
 *   p_fr (3) - right foot position in world frame
 *
 * Error state (21-DOF):
 *   dp, dv, dtheta, db_a, db_g, dp_fl, dp_fr
 *
 * Reference: Bloesch et al. "State Estimation for Legged Robots", RSS 2013.
 */

#include <Eigen/Dense>
#include <cmath>

namespace leg_odom {

// Error-state indices
enum StateIdx {
  P = 0,     // position [0:3)
  V = 3,     // velocity [3:6)
  TH = 6,    // rotation error [6:9)
  BA = 9,    // accel bias [9:12)
  BG = 12,   // gyro bias [12:15)
  FL = 15,   // left foot [15:18)
  FR = 18,   // right foot [18:21)
  DIM = 21   // total error-state dimension
};

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;
  return m;
}

inline Eigen::Matrix3d exp_so3(const Eigen::Vector3d& phi) {
  double angle = phi.norm();
  if (angle < 1e-10) {
    return Eigen::Matrix3d::Identity() + skew(phi);
  }
  Eigen::Vector3d axis = phi / angle;
  Eigen::Matrix3d K = skew(axis);
  return Eigen::Matrix3d::Identity() + std::sin(angle) * K + (1.0 - std::cos(angle)) * K * K;
}

struct NoiseParams {
  double sigma_a = 0.1;       // accel noise (m/s^2)
  double sigma_g = 0.01;      // gyro noise (rad/s)
  double sigma_ba = 0.0;      // accel bias walk (m/s^2/sqrt(s))
  double sigma_bg = 0.001;    // gyro bias walk (rad/s/sqrt(s))
  double sigma_contact = 0.002; // foot contact noise (m/sqrt(s))
  double sigma_swing = 1.0;   // foot swing noise (m/sqrt(s))
  double sigma_fk = 0.005;    // FK position noise (m)
  double sigma_zupt = 0.03;   // ZUPT velocity noise (m/s)
  double sigma_flat_z = 0.001; // flat ground Z position (m)
  double sigma_flat_vz = 0.001; // flat ground Z velocity (m/s)
};

struct State {
  // Nominal state
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_fl = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_fr = Eigen::Vector3d::Zero();

  // Error-state covariance
  Eigen::Matrix<double, DIM, DIM> P = Eigen::Matrix<double, DIM, DIM>::Identity() * 0.01;

  // Cached values
  Eigen::Vector3d last_gyro_corrected = Eigen::Vector3d::Zero();
  double last_dt = 0.005;

  // Previous FK (for ZUPT)
  Eigen::Vector3d prev_fk_left = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_fk_right = Eigen::Vector3d::Zero();
  bool has_prev_fk = false;

  // Step-level tracking
  Eigen::Vector3d stance_start_fk_left = Eigen::Vector3d::Zero();
  Eigen::Vector3d stance_start_fk_right = Eigen::Vector3d::Zero();
  int stance_frames_left = 0;
  int stance_frames_right = 0;
  bool prev_contact_left = false;
  bool prev_contact_right = false;

  // Stillness detection
  double gyro_norm_buffer[100] = {};
  int gyro_buf_idx = 0;
  int gyro_buf_count = 0;
  bool is_still = false;

  bool initialized = false;

  void initialize(const Eigen::Vector3d& accel,
                  const Eigen::Vector3d& fk_left,
                  const Eigen::Vector3d& fk_right) {
    // Estimate initial rotation from gravity
    Eigen::Vector3d g_body = -accel.normalized() * 9.81;
    Eigen::Vector3d g_world(0, 0, -9.81);
    R = rotation_from_gravity(g_body, g_world);

    p.setZero();
    v.setZero();
    b_a = accel - R.transpose() * Eigen::Vector3d(0, 0, 9.81);
    b_g.setZero();

    p_fl = p + R * fk_left;
    p_fr = p + R * fk_right;

    P = Eigen::Matrix<double, DIM, DIM>::Identity() * 0.01;
    P.block<3,3>(V, V) = Eigen::Matrix3d::Identity() * 0.01;
    P.block<3,3>(BA, BA) = Eigen::Matrix3d::Identity() * 0.0; // locked
    P.block<3,3>(BG, BG) = Eigen::Matrix3d::Identity() * 0.01 * 0.01;

    prev_fk_left = fk_left;
    prev_fk_right = fk_right;
    has_prev_fk = true;
    initialized = true;
  }

  // Get pose as (position, quaternion [x,y,z,w])
  std::pair<Eigen::Vector3d, Eigen::Vector4d> get_pose() const {
    Eigen::Quaterniond q(R);
    return {p, Eigen::Vector4d(q.x(), q.y(), q.z(), q.w())};
  }

 private:
  static Eigen::Matrix3d rotation_from_gravity(const Eigen::Vector3d& g_body,
                                                const Eigen::Vector3d& g_world) {
    Eigen::Vector3d gb = g_body.normalized();
    Eigen::Vector3d gw = g_world.normalized();
    Eigen::Vector3d v = gb.cross(gw);
    double c = gb.dot(gw);
    if (v.norm() < 1e-10) {
      if (c > 0) return Eigen::Matrix3d::Identity();
      else return -Eigen::Matrix3d::Identity();
    }
    Eigen::Matrix3d V = skew(v);
    return Eigen::Matrix3d::Identity() + V + V * V / (1.0 + c);
  }
};

}  // namespace leg_odom
