#pragma once
/**
 * @file GTSAMSmoother.h
 * @brief GTSAM sliding-window smoother for gyro bias estimation.
 * Runs asynchronously, feeds back bias to ESKF.
 */

#include <deque>
#include <memory>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

#include "leg_factors.h"

namespace leg_odom {

inline gtsam::Key Xk(int i) { return gtsam::Symbol('x', i); }
inline gtsam::Key Vk(int i) { return gtsam::Symbol('v', i); }
inline gtsam::Key Bk(int i) { return gtsam::Symbol('b', i); }
inline gtsam::Key FLk(int i) { return gtsam::Symbol('f', i); }
inline gtsam::Key FRk(int i) { return gtsam::Symbol('g', i); }

struct KeyframeData {
  gtsam::Pose3 pose;
  Eigen::Vector3d velocity;
  gtsam::imuBias::ConstantBias bias;
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> pim;
  Eigen::Vector3d fk_left, fk_right;
  bool contact_left, contact_right;
};

class GTSAMSmoother {
 public:
  GTSAMSmoother(double sigma_fk, double sigma_contact, double sigma_swing,
                double sigma_ba_walk, double sigma_bg_walk,
                double sigma_flat_z, double gravity_mag = 9.81)
      : sigma_fk_(sigma_fk), sigma_contact_(sigma_contact),
        sigma_swing_(sigma_swing), sigma_ba_(sigma_ba_walk),
        sigma_bg_(sigma_bg_walk), sigma_flat_z_(sigma_flat_z) {
    auto imu_params = gtsam::PreintegrationParams::MakeSharedD(gravity_mag);
    imu_params->setAccelerometerCovariance(Eigen::Matrix3d::Identity() * 0.01);
    imu_params->setGyroscopeCovariance(Eigen::Matrix3d::Identity() * 0.0001);
    imu_params->setIntegrationCovariance(Eigen::Matrix3d::Identity() * 1e-5);
    imu_params_ = imu_params;
  }

  std::shared_ptr<gtsam::PreintegratedImuMeasurements>
  create_preintegrator(const gtsam::imuBias::ConstantBias& bias) {
    return std::make_shared<gtsam::PreintegratedImuMeasurements>(imu_params_, bias);
  }

  void add_keyframe(const KeyframeData& kf) {
    keyframes_.push_back(kf);
    if (keyframes_.size() > window_size_) {
      keyframes_.pop_front();
    }
    since_opt_++;
  }

  bool should_optimize() const {
    return since_opt_ >= opt_interval_ && keyframes_.size() >= 5;
  }

  // Returns true if optimization succeeded, writes bias to out_ba, out_bg
  bool optimize(Eigen::Vector3d& out_ba, Eigen::Vector3d& out_bg) {
    since_opt_ = 0;
    int n = keyframes_.size();
    if (n < 3) return false;

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    // Prior on first keyframe
    auto& kf0 = keyframes_[0];
    graph.addPrior(Xk(0), kf0.pose,
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished()));
    graph.addPrior(Vk(0), kf0.velocity,
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
    graph.addPrior(Bk(0), kf0.bias,
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.5, 0.5, 0.5, 0.1, 0.1, 0.1).finished()));

    Eigen::Matrix3d R0 = kf0.pose.rotation().matrix();
    Eigen::Vector3d t0 = kf0.pose.translation();
    auto foot_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    graph.addPrior(FLk(0), gtsam::Point3(t0 + R0 * kf0.fk_left), foot_noise);
    graph.addPrior(FRk(0), gtsam::Point3(t0 + R0 * kf0.fk_right), foot_noise);

    values.insert(Xk(0), kf0.pose);
    values.insert(Vk(0), kf0.velocity);
    values.insert(Bk(0), kf0.bias);
    values.insert(FLk(0), gtsam::Point3(t0 + R0 * kf0.fk_left));
    values.insert(FRk(0), gtsam::Point3(t0 + R0 * kf0.fk_right));

    auto fk_noise = gtsam::noiseModel::Isotropic::Sigma(3, sigma_fk_);

    for (int j = 1; j < n; j++) {
      int i = j - 1;
      auto& kf = keyframes_[j];

      // IMU factor
      if (kf.pim && kf.pim->deltaTij() > 0) {
        graph.emplace_shared<gtsam::ImuFactor>(Xk(i), Vk(i), Xk(j), Vk(j), Bk(i), *kf.pim);
      }

      // Bias between
      double dt = kf.pim ? std::max(kf.pim->deltaTij(), 1e-4) : 0.05;
      auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << std::max(sigma_ba_ * std::sqrt(dt), 1e-6),
                                std::max(sigma_ba_ * std::sqrt(dt), 1e-6),
                                std::max(sigma_ba_ * std::sqrt(dt), 1e-6),
                                std::max(sigma_bg_ * std::sqrt(dt), 1e-6),
                                std::max(sigma_bg_ * std::sqrt(dt), 1e-6),
                                std::max(sigma_bg_ * std::sqrt(dt), 1e-6)).finished());
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
          Bk(i), Bk(j), gtsam::imuBias::ConstantBias(), bias_noise);

      // FK factors
      graph.emplace_shared<FKPositionFactor>(Xk(j), FLk(j), kf.fk_left, fk_noise);
      graph.emplace_shared<FKPositionFactor>(Xk(j), FRk(j), kf.fk_right, fk_noise);

      // Contact factors
      double sig_l = kf.contact_left ? sigma_contact_ * std::sqrt(dt) : sigma_swing_;
      double sig_r = kf.contact_right ? sigma_contact_ * std::sqrt(dt) : sigma_swing_;
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(
          FLk(i), FLk(j), gtsam::Point3(0, 0, 0),
          gtsam::noiseModel::Isotropic::Sigma(3, std::max(sig_l, 1e-6)));
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(
          FRk(i), FRk(j), gtsam::Point3(0, 0, 0),
          gtsam::noiseModel::Isotropic::Sigma(3, std::max(sig_r, 1e-6)));

      // Flat Z
      if (sigma_flat_z_ > 0) {
        graph.emplace_shared<FlatZPositionFactor>(Xk(j),
            gtsam::noiseModel::Isotropic::Sigma(1, sigma_flat_z_));
      }

      // Initial values
      Eigen::Matrix3d Rj = kf.pose.rotation().matrix();
      Eigen::Vector3d tj = kf.pose.translation();
      values.insert(Xk(j), kf.pose);
      values.insert(Vk(j), kf.velocity);
      values.insert(Bk(j), kf.bias);
      values.insert(FLk(j), gtsam::Point3(tj + Rj * kf.fk_left));
      values.insert(FRk(j), gtsam::Point3(tj + Rj * kf.fk_right));
    }

    // Optimize
    try {
      gtsam::LevenbergMarquardtParams lm_params;
      lm_params.setMaxIterations(20);
      gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
      auto result = optimizer.optimize();

      auto bias = result.at<gtsam::imuBias::ConstantBias>(Bk(n - 1));
      out_ba = bias.accelerometer();
      out_bg = bias.gyroscope();
      return true;
    } catch (const std::exception& e) {
      return false;
    }
  }

  int window_size_ = 60;
  int opt_interval_ = 20;

 private:
  std::deque<KeyframeData> keyframes_;
  int since_opt_ = 0;

  double sigma_fk_, sigma_contact_, sigma_swing_;
  double sigma_ba_, sigma_bg_, sigma_flat_z_;
  std::shared_ptr<gtsam::PreintegrationParams> imu_params_;
};

}  // namespace leg_odom
