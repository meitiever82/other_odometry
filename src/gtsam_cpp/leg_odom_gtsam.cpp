/**
 * @file leg_odom_gtsam.cpp
 * @brief C++ GTSAM ISAM2 leg odometry.
 *
 * Reads pre-extracted CSV data (from Python rosbag reader),
 * runs ISAM2 factor graph optimization, outputs trajectory.
 *
 * Usage:
 *   ./leg_odom_gtsam <input.csv> <output.csv> [keyframe_interval]
 *
 * Input CSV columns (per row, 200Hz):
 *   timestamp,ax,ay,az,gx,gy,gz,fk_lx,fk_ly,fk_lz,fk_rx,fk_ry,fk_rz,
 *   contact_l,contact_r,gt_x,gt_y,gt_z,gt_qx,gt_qy,gt_qz,gt_qw
 */

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

#include "leg_factors.h"

using gtsam::symbol_shorthand::X;  // Pose3
using gtsam::symbol_shorthand::V;  // Vector3 velocity
using gtsam::symbol_shorthand::B;  // imuBias

// Foot symbols
gtsam::Key FL(int i) { return gtsam::Symbol('f', i); }
gtsam::Key FR(int i) { return gtsam::Symbol('g', i); }

struct IMUSample {
  double t;
  Eigen::Vector3d accel, gyro;
  Eigen::Vector3d fk_left, fk_right;
  bool contact_left, contact_right;
  Eigen::Vector3d gt_pos;
  Eigen::Vector4d gt_quat;  // x,y,z,w
};

std::vector<IMUSample> readCSV(const std::string& path) {
  std::vector<IMUSample> data;
  std::ifstream f(path);
  std::string line;
  std::getline(f, line);  // skip header

  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string token;
    IMUSample s;
    auto next = [&]() -> double {
      std::getline(ss, token, ',');
      return std::stod(token);
    };

    s.t = next();
    s.accel = {next(), next(), next()};
    s.gyro = {next(), next(), next()};
    s.fk_left = {next(), next(), next()};
    s.fk_right = {next(), next(), next()};
    s.contact_left = next() > 0.5;
    s.contact_right = next() > 0.5;
    s.gt_pos = {next(), next(), next()};
    s.gt_quat = {next(), next(), next(), next()};
    data.push_back(s);
  }
  return data;
}

Eigen::Matrix3d rotFromGravity(const Eigen::Vector3d& accel) {
  Eigen::Vector3d g_body = -accel.normalized() * 9.81;
  Eigen::Vector3d g_world(0, 0, -9.81);
  Eigen::Vector3d gb = g_body.normalized();
  Eigen::Vector3d gw = g_world.normalized();
  Eigen::Vector3d v = gb.cross(gw);
  double c = gb.dot(gw);
  if (v.norm() < 1e-10) {
    if (c > 0) return Eigen::Matrix3d::Identity();
    else return -Eigen::Matrix3d::Identity();
  }
  Eigen::Matrix3d V;
  V << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return Eigen::Matrix3d::Identity() + V + V * V / (1.0 + c);
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <input.csv> <output.csv> [kf_interval]" << std::endl;
    return 1;
  }

  std::string input_path = argv[1];
  std::string output_path = argv[2];
  int kf_interval = (argc > 3) ? std::stoi(argv[3]) : 5;  // 高频: 每5帧=40Hz

  std::cout << "Reading " << input_path << "..." << std::endl;
  auto data = readCSV(input_path);
  std::cout << "Loaded " << data.size() << " samples" << std::endl;

  // --- GTSAM Setup ---
  auto imu_params = gtsam::PreintegrationParams::MakeSharedD(9.81);
  imu_params->setAccelerometerCovariance(Eigen::Matrix3d::Identity() * 0.01);  // 0.1^2
  imu_params->setGyroscopeCovariance(Eigen::Matrix3d::Identity() * 0.0001);    // 0.01^2
  imu_params->setIntegrationCovariance(Eigen::Matrix3d::Identity() * 1e-5);

  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.1;
  gtsam::ISAM2 isam(isam_params);

  // Noise models
  double sigma_fk = 0.005;
  double sigma_contact = 0.002;
  double sigma_swing = 1.0;
  double sigma_zupt = 0.003;
  double sigma_flat_z = 0.001;
  double sigma_flat_vz = 0.001;
  double sigma_ba_walk = 1e-6;   // accel bias 几乎锁定
  double sigma_bg_walk = 0.0001; // gyro bias 保守估计

  auto fk_noise = gtsam::noiseModel::Isotropic::Sigma(3, sigma_fk);
  auto zupt_noise = gtsam::noiseModel::Isotropic::Sigma(3, sigma_zupt);
  auto flat_z_noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma_flat_z);
  auto flat_vz_noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma_flat_vz);

  // --- Initialize from first 50 frames ---
  Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero();
  int init_count = 50;
  for (int i = 0; i < init_count && i < (int)data.size(); i++) {
    avg_accel += data[i].accel;
  }
  avg_accel /= init_count;

  Eigen::Matrix3d R0 = rotFromGravity(avg_accel);
  gtsam::Pose3 init_pose(gtsam::Rot3(R0), gtsam::Point3(0, 0, 0));
  Eigen::Vector3d init_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_ba = avg_accel - R0.transpose() * Eigen::Vector3d(0, 0, 9.81);
  gtsam::imuBias::ConstantBias init_bias(init_ba, Eigen::Vector3d::Zero());

  Eigen::Vector3d fl_world = R0 * data[init_count].fk_left;
  Eigen::Vector3d fr_world = R0 * data[init_count].fk_right;

  // Add priors
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
  auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
  auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  auto foot_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);

  graph.addPrior(X(0), init_pose, pose_noise);
  graph.addPrior(V(0), init_vel, vel_noise);
  graph.addPrior(B(0), init_bias, bias_noise);
  graph.addPrior(FL(0), gtsam::Point3(fl_world), foot_noise);
  graph.addPrior(FR(0), gtsam::Point3(fr_world), foot_noise);

  values.insert(X(0), init_pose);
  values.insert(V(0), init_vel);
  values.insert(B(0), init_bias);
  values.insert(FL(0), gtsam::Point3(fl_world));
  values.insert(FR(0), gtsam::Point3(fr_world));

  isam.update(graph, values);
  graph.resize(0);
  values.clear();

  // --- Main loop ---
  auto current_bias = init_bias;
  auto pim = std::make_shared<gtsam::PreintegratedImuMeasurements>(imu_params, current_bias);

  auto last_pose = init_pose;
  auto last_vel = init_vel;
  int kf_index = 0;
  int imu_count = 0;
  double last_t = data[init_count].t;

  Eigen::Vector3d prev_fk_left = data[init_count].fk_left;
  Eigen::Vector3d prev_fk_right = data[init_count].fk_right;
  Eigen::Vector3d last_gyro_corrected = Eigen::Vector3d::Zero();

  // Output: timestamp, est_x, est_y, est_z, est_qx, est_qy, est_qz, est_qw
  std::ofstream out(output_path);
  out << "timestamp,est_x,est_y,est_z,est_qx,est_qy,est_qz,est_qw,"
      << "gt_x,gt_y,gt_z,gt_qx,gt_qy,gt_qz,gt_qw" << std::endl;

  // Write initial estimate
  {
    auto q0 = init_pose.rotation().toQuaternion();
    out << data[init_count].t << ",0,0,0,"
        << q0.x() << "," << q0.y() << "," << q0.z() << "," << q0.w() << ","
        << data[init_count].gt_pos.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
        << "," << data[init_count].gt_quat.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
        << std::endl;
  }

  for (size_t idx = init_count + 1; idx < data.size(); idx++) {
    const auto& s = data[idx];
    double dt = s.t - last_t;
    last_t = s.t;
    if (dt <= 0 || dt > 0.1) continue;

    last_gyro_corrected = s.gyro - current_bias.gyroscope();
    pim->integrateMeasurement(s.accel, s.gyro, dt);
    imu_count++;

    if (imu_count < kf_interval) continue;

    // === Create keyframe ===
    int i = kf_index;
    int j = i + 1;
    double delta_t = pim->deltaTij();

    gtsam::NonlinearFactorGraph kf_graph;
    gtsam::Values kf_values;

    // 1. IMU factor
    kf_graph.emplace_shared<gtsam::ImuFactor>(X(i), V(i), X(j), V(j), B(i), *pim);

    // 2. Bias evolution
    double sig_ba = std::max(sigma_ba_walk * std::sqrt(delta_t), 1e-6);
    double sig_bg = std::max(sigma_bg_walk * std::sqrt(delta_t), 1e-6);
    auto bias_between_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << sig_ba, sig_ba, sig_ba, sig_bg, sig_bg, sig_bg).finished());
    kf_graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        B(i), B(j), gtsam::imuBias::ConstantBias(), bias_between_noise);

    // 3. FK position factors (C++ native, fast!)
    kf_graph.emplace_shared<leg_odom::FKPositionFactor>(X(j), FL(j), s.fk_left, fk_noise);
    kf_graph.emplace_shared<leg_odom::FKPositionFactor>(X(j), FR(j), s.fk_right, fk_noise);

    // 4. Contact / swing foot constraints
    double sig_fl = s.contact_left ? sigma_contact * std::sqrt(delta_t) : sigma_swing;
    double sig_fr = s.contact_right ? sigma_contact * std::sqrt(delta_t) : sigma_swing;
    kf_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(
        FL(i), FL(j), gtsam::Point3(0, 0, 0),
        gtsam::noiseModel::Isotropic::Sigma(3, std::max(sig_fl, 1e-6)));
    kf_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(
        FR(i), FR(j), gtsam::Point3(0, 0, 0),
        gtsam::noiseModel::Isotropic::Sigma(3, std::max(sig_fr, 1e-6)));

    // 5. ZUPT velocity factors (contact feet)
    // 用关键帧间隔的 dt，不是单帧 dt
    double zupt_dt = delta_t;
    if (s.contact_left && zupt_dt > 0.001) {
      kf_graph.emplace_shared<leg_odom::FootVelocityFactor>(
          X(j), V(j), s.fk_left, prev_fk_left,
          last_gyro_corrected, zupt_dt, zupt_noise);
    }
    if (s.contact_right && zupt_dt > 0.001) {
      kf_graph.emplace_shared<leg_odom::FootVelocityFactor>(
          X(j), V(j), s.fk_right, prev_fk_right,
          last_gyro_corrected, zupt_dt, zupt_noise);
    }

    // 6. Flat ground
    kf_graph.emplace_shared<leg_odom::FlatZPositionFactor>(X(j), flat_z_noise);
    kf_graph.emplace_shared<leg_odom::FlatZVelocityFactor>(V(j), flat_vz_noise);

    // Initial values from IMU prediction
    gtsam::NavState pred = pim->predict(gtsam::NavState(last_pose, last_vel), current_bias);
    Eigen::Matrix3d R_pred = pred.pose().rotation().matrix();
    Eigen::Vector3d t_pred = pred.pose().translation();

    kf_values.insert(X(j), pred.pose());
    kf_values.insert(V(j), pred.velocity());
    kf_values.insert(B(j), current_bias);
    kf_values.insert(FL(j), gtsam::Point3(t_pred + R_pred * s.fk_left));
    kf_values.insert(FR(j), gtsam::Point3(t_pred + R_pred * s.fk_right));

    // ISAM2 update
    try {
      isam.update(kf_graph, kf_values);
      isam.update();  // extra iteration

      auto result = isam.calculateEstimate();
      last_pose = result.at<gtsam::Pose3>(X(j));
      last_vel = result.at<Eigen::Vector3d>(V(j));
      current_bias = result.at<gtsam::imuBias::ConstantBias>(B(j));

      // Write output
      auto q = last_pose.rotation().toQuaternion();
      auto t = last_pose.translation();
      out << s.t << ","
          << t.x() << "," << t.y() << "," << t.z() << ","
          << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
          << s.gt_pos.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
          << "," << s.gt_quat.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
          << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "ISAM2 error at kf " << j << ": " << e.what() << std::endl;
      // Fallback to prediction
      last_pose = pred.pose();
      last_vel = pred.velocity();
    }

    // Update state
    prev_fk_left = s.fk_left;
    prev_fk_right = s.fk_right;
    kf_index = j;
    pim = std::make_shared<gtsam::PreintegratedImuMeasurements>(imu_params, current_bias);
    imu_count = 0;

    if (j % 100 == 0) {
      auto t = last_pose.translation();
      std::cout << "KF " << j << ": pos=(" << t.x() << ", " << t.y() << ", " << t.z()
                << ") bias_g=(" << current_bias.gyroscope().transpose() << ")" << std::endl;
    }
  }

  out.close();
  std::cout << "Done! " << kf_index << " keyframes. Output: " << output_path << std::endl;
  return 0;
}
