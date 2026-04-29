#pragma once
/**
 * @file leg_factors.h
 * @brief Custom GTSAM factors for legged robot odometry.
 *
 * Factors:
 *   1. FKPositionFactor: p + R * fk_body = p_foot
 *   2. FootVelocityFactor (ZUPT): v + omega x (R*fk) + R*dfk/dt = 0
 *   3. FlatGroundFactor: pose.z() = 0, vel.z() = 0
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

namespace leg_odom {

using gtsam::Key;
using gtsam::Matrix;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

// ============================================================
// 1. FK Position Factor: p + R * fk_body = p_foot
// ============================================================
class FKPositionFactor : public gtsam::NoiseModelFactorN<Pose3, Point3> {
  using Base = gtsam::NoiseModelFactorN<Pose3, Point3>;
  Vector3 fk_body_;  // FK result in body frame

 public:
  FKPositionFactor(Key pose_key, Key foot_key, const Vector3& fk_body,
                   const gtsam::SharedNoiseModel& model)
      : Base(model, pose_key, foot_key), fk_body_(fk_body) {}

  Vector evaluateError(const Pose3& pose, const Point3& foot,
                       gtsam::OptionalMatrixType H1,
                       gtsam::OptionalMatrixType H2) const override {
    // error = p + R * fk - foot
    Eigen::Matrix3d R = pose.rotation().matrix();
    Point3 t = pose.translation();
    Vector3 r_world = R * fk_body_;
    Vector3 error = t + r_world - foot;

    if (H1) {
      // dError/dPose: GTSAM body-frame perturbation
      // d(R*fk)/d(omega_body) = -R * skew(fk)
      // d(t + R*v)/d(v_body) = R
      *H1 = gtsam::Matrix::Zero(3, 6);
      H1->block<3, 3>(0, 0) = -R * gtsam::skewSymmetric(fk_body_);
      H1->block<3, 3>(0, 3) = R;
    }
    if (H2) {
      *H2 = -gtsam::Matrix::Identity(3, 3);
    }
    return error;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<FKPositionFactor>(*this);
  }
};

// ============================================================
// 2. Foot Velocity Factor (ZUPT):
//    v + omega_world x (R*fk) + R*dfk/dt ≈ 0
//    Simplified: constrains v to expected velocity from FK kinematics
// ============================================================
class FootVelocityFactor : public gtsam::NoiseModelFactorN<Pose3, Vector3> {
  using Base = gtsam::NoiseModelFactorN<Pose3, Vector3>;
  Vector3 fk_body_;       // current FK in body frame
  Vector3 fk_body_prev_;  // previous FK in body frame
  Vector3 gyro_body_;     // body angular velocity (corrected)
  double dt_;             // time step

 public:
  FootVelocityFactor(Key pose_key, Key vel_key,
                     const Vector3& fk_body, const Vector3& fk_body_prev,
                     const Vector3& gyro_body, double dt,
                     const gtsam::SharedNoiseModel& model)
      : Base(model, pose_key, vel_key),
        fk_body_(fk_body), fk_body_prev_(fk_body_prev),
        gyro_body_(gyro_body), dt_(dt) {}

  Vector evaluateError(const Pose3& pose, const Vector3& vel,
                       gtsam::OptionalMatrixType H1,
                       gtsam::OptionalMatrixType H2) const override {
    Eigen::Matrix3d R = pose.rotation().matrix();

    // R * dFK/dt
    Vector3 R_dfk = R * ((fk_body_ - fk_body_prev_) / dt_);

    // omega_world x (R * FK)
    Vector3 r_world = R * fk_body_;
    Vector3 omega_world = R * gyro_body_;
    Vector3 omega_cross = omega_world.cross(r_world);

    // v_expected = -(omega_cross + R_dfk)
    Vector3 v_expected = -(omega_cross + R_dfk);

    // error = vel - v_expected
    Vector3 error = vel - v_expected;

    if (H1) {
      // Numerical derivative for simplicity (complex analytical form)
      *H1 = gtsam::numericalDerivative21<Vector3, Pose3, Vector3>(
          [this](const Pose3& p, const Vector3& v) -> Vector3 {
            Eigen::Matrix3d Rp = p.rotation().matrix();
            Vector3 R_dfk_p = Rp * ((fk_body_ - fk_body_prev_) / dt_);
            Vector3 r_w = Rp * fk_body_;
            Vector3 ow = Rp * gyro_body_;
            Vector3 oc = ow.cross(r_w);
            return v - (-(oc + R_dfk_p));
          },
          pose, vel);
    }
    if (H2) {
      *H2 = gtsam::Matrix::Identity(3, 3);
    }
    return error;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<FootVelocityFactor>(*this);
  }
};

// ============================================================
// 3. Flat Ground Factor: pose.z() ≈ 0 (1D)
// ============================================================
class FlatZPositionFactor : public gtsam::NoiseModelFactorN<Pose3> {
  using Base = gtsam::NoiseModelFactorN<Pose3>;

 public:
  FlatZPositionFactor(Key pose_key, const gtsam::SharedNoiseModel& model)
      : Base(model, pose_key) {}

  Vector evaluateError(const Pose3& pose,
                       gtsam::OptionalMatrixType H1) const override {
    if (H1) {
      *H1 = gtsam::Matrix::Zero(1, 6);
      // dz/d(body_frame_perturbation): d(t_z)/d(v_body) = R[2,:]
      Eigen::Matrix3d R = pose.rotation().matrix();
      H1->block<1, 3>(0, 3) = R.row(2);
    }
    return (Vector(1) << pose.z()).finished();
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<FlatZPositionFactor>(*this);
  }
};

// ============================================================
// 4. Flat Ground Velocity Factor: vel.z() ≈ 0 (1D)
// ============================================================
class FlatZVelocityFactor : public gtsam::NoiseModelFactorN<Vector3> {
  using Base = gtsam::NoiseModelFactorN<Vector3>;

 public:
  FlatZVelocityFactor(Key vel_key, const gtsam::SharedNoiseModel& model)
      : Base(model, vel_key) {}

  Vector evaluateError(const Vector3& vel,
                       gtsam::OptionalMatrixType H1) const override {
    if (H1) {
      *H1 = gtsam::Matrix::Zero(1, 3);
      (*H1)(0, 2) = 1.0;
    }
    return (Vector(1) << vel(2)).finished();
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<FlatZVelocityFactor>(*this);
  }
};

}  // namespace leg_odom
