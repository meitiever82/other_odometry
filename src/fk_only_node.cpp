/**
 * @file fk_only_node.cpp
 * @brief FK-only leg odometry ROS2 node.
 *
 * State:          p ∈ R³, R ∈ SO(3), gyro_bias ∈ R³.
 * No Kalman filter, no IMU acceleration integration into position/velocity.
 *
 * Pipeline (driven by /joint_states at ~200 Hz):
 *   1. gyro integrates R:               R ← R · exp_so3((gyro - bg) · dt)
 *   2. FK Jacobian gives foot velocity: v_foot_body = J(q) · q̇  (per foot)
 *   3. optional heel-toe rolling comp:  v_foot_body.x += sign · toe_offset · q̇_ankle_pitch
 *   4. contact detection:               cl, cr from LJPITCH / RJPITCH effort threshold
 *   5. accel-tilt Mahony (quasi-static + stance gated): pulls R's roll/pitch
 *      toward gravity; body z component of the error is zeroed so yaw stays free.
 *   6. body velocity in world:          v_world = -R · v_foot_body  (stance feet only,
 *                                                                     averaged if both)
 *   7. position integration:            p ← p + v_world · dt
 *   8. FlatZ clamp in stance:           p.z ← (1 - α) · p.z   (indoor flat-floor prior)
 *
 * Initialization: first `bias_window_sec` seconds are assumed static. At the end
 * of the window, gyro bias = avg(gyro), and R is gravity-aligned from avg(accel)
 * (yaw is free, initialized to 0). During the init window the robot is static, so
 * v_foot_body ≈ 0 regardless of R; R is left at identity.
 *
 * Subscribes: /imu, /joint_states
 * Publishes:  /leg_odometry (nav_msgs/Odometry), TF odom → base_link_leg_odom
 *
 * See scripts/fk_only_odometry.py for the Python reference of the same algorithm.
 */

#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "leg_odometry/so3_utils.h"     // exp_so3
#include "leg_odometry/state/Kinematics.h"

namespace {

constexpr double kGravity     = 9.81;
constexpr double kMaxDt       = 0.1;    // s, skip joint_state callbacks with larger dt

}  // namespace

class FkOnlyNode : public rclcpp::Node {
 public:
  FkOnlyNode() : Node("fk_only_odom") {
    // --- parameters ---
    declare_parameter("urdf_path", std::string(""));

    // contact detection (effort threshold + hysteresis)
    declare_parameter("effort_threshold_left",  5.0);
    declare_parameter("effort_threshold_right", 5.0);
    declare_parameter("effort_hysteresis",      1.0);
    declare_parameter("effort_joint_left",  std::string("LJPITCH"));
    declare_parameter("effort_joint_right", std::string("RJPITCH"));

    // static window for gyro-bias / gravity initialization
    declare_parameter("bias_window_sec", 3.0);

    // accel-tilt (Mahony) — pulls R's roll/pitch toward gravity
    declare_parameter("tilt_kp",             1.0);
    declare_parameter("tilt_accel_band",     0.5);   // m/s² around 9.81
    declare_parameter("tilt_require_stance", true);

    // FlatZ low-pass clamp during stance
    declare_parameter("flatz_enabled", true);
    declare_parameter("flatz_alpha",   0.05);

    // heel-toe rolling compensation: v_foot_body.x += sign · toe_offset · q̇_ankle_pitch
    // Tune per scenario — walking forward: ~0.20 m; spinning/unknown: 0.0.
    declare_parameter("foot_roll_toe_offset", 0.0);
    declare_parameter("foot_roll_sign",       1.0);

    // ROS output
    declare_parameter("publish_tf", true);
    declare_parameter("odom_frame", std::string("odom"));
    declare_parameter("base_frame", std::string("base_link_leg_odom"));
    declare_parameter("odom_topic", std::string("/leg_odometry"));

    // optional diagnostic CSV
    declare_parameter("diag_csv_path", std::string(""));

    // --- load URDF / kinematics ---
    const auto urdf = get_parameter("urdf_path").as_string();
    if (urdf.empty() || !kin_.init(urdf)) {
      RCLCPP_FATAL(get_logger(), "URDF load failed: '%s'", urdf.c_str());
      throw std::runtime_error("urdf init failed");
    }

    // --- read parameters ---
    const double th_l = get_parameter("effort_threshold_left").as_double();
    const double th_r = get_parameter("effort_threshold_right").as_double();
    const double hys  = get_parameter("effort_hysteresis").as_double();
    det_ = std::make_unique<leg_odom::ContactDetector>(th_l, th_r, hys);

    eff_joint_l_          = get_parameter("effort_joint_left").as_string();
    eff_joint_r_          = get_parameter("effort_joint_right").as_string();
    bias_window_sec_      = get_parameter("bias_window_sec").as_double();
    tilt_kp_              = get_parameter("tilt_kp").as_double();
    tilt_accel_band_      = get_parameter("tilt_accel_band").as_double();
    tilt_require_stance_  = get_parameter("tilt_require_stance").as_bool();
    flatz_enabled_        = get_parameter("flatz_enabled").as_bool();
    flatz_alpha_          = get_parameter("flatz_alpha").as_double();
    foot_roll_toe_offset_ = get_parameter("foot_roll_toe_offset").as_double();
    foot_roll_sign_       = get_parameter("foot_roll_sign").as_double();
    publish_tf_           = get_parameter("publish_tf").as_bool();
    odom_frame_           = get_parameter("odom_frame").as_string();
    base_frame_           = get_parameter("base_frame").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();

    const auto diag_path = get_parameter("diag_csv_path").as_string();
    if (!diag_path.empty()) {
      diag_fp_ = std::fopen(diag_path.c_str(), "w");
      if (diag_fp_) {
        std::fprintf(diag_fp_,
            "t_abs,x,y,z,roll,pitch,yaw,vx,vy,vz,"
            "vL_x,vL_y,vL_z,vR_x,vR_y,vR_z,cl,cr,tilt_applied\n");
      }
    }

    joint_map_         = leg_odom::default_joint_mapping();
    n_joints_expected_ = joint_map_.size();

    // --- ROS2 I/O ---
    // Deep reliable queue so that rosbag2 replay bursts don't drop messages; matches
    // Python offline parity (scripts/fk_only_odometry.py reads the bag file directly).
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(2000)).reliable();
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos,
        std::bind(&FkOnlyNode::joint_cb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", qos,
        std::bind(&FkOnlyNode::imu_cb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_bc_    = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    p_.setZero();
    R_.setIdentity();
    bg_.setZero();

    RCLCPP_INFO(get_logger(),
        "fk_only_odom started: effort_thr L/R=%.2f/%.2f hys=%.2f  bias_window=%.1fs  "
        "tilt_kp=%.2f band=%.2f  flatz=%s α=%.3f  foot_roll=%.3fm (sign %+.0f)  topic=%s",
        th_l, th_r, hys, bias_window_sec_,
        tilt_kp_, tilt_accel_band_,
        flatz_enabled_ ? "on" : "off", flatz_alpha_,
        foot_roll_toe_offset_, foot_roll_sign_,
        odom_topic.c_str());
  }

  ~FkOnlyNode() { if (diag_fp_) std::fclose(diag_fp_); }

 private:
  // -------------------------------------------------------------------------
  // IMU callback — accumulates bias during init window, then caches debiased
  // gyro and raw accel for the joint_state callback to consume.
  // -------------------------------------------------------------------------
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const Eigen::Vector3d g(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z);
    const Eigen::Vector3d a(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
    const double t_abs = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    last_accel_ = a;

    if (!init_done_) {
      if (init_window_start_ < 0.0) init_window_start_ = t_abs;
      if (t_abs - init_window_start_ < bias_window_sec_) {
        accum_gyro_  += g;
        accum_accel_ += a;
        n_static_++;
        last_gyro_ = g;   // use raw during init; R is held at identity
        return;
      }

      // end of init window: compute gyro bias + initial R from gravity
      if (n_static_ > 0) {
        bg_ = accum_gyro_ / n_static_;
        const Eigen::Vector3d avg_a = accum_accel_ / n_static_;
        const Eigen::Vector3d body_up = avg_a.normalized();
        const Eigen::Vector3d world_up(0, 0, 1);
        R_ = Eigen::Quaterniond::FromTwoVectors(body_up, world_up).toRotationMatrix();

        const auto rpy = R_to_rpy(R_);
        RCLCPP_INFO(get_logger(),
            "init done (%d samples): bg=[%+.5f,%+.5f,%+.5f] rad/s  "
            "avg_accel=[%+.3f,%+.3f,%+.3f]  initial rpy=[%+.2f,%+.2f,%+.2f] deg",
            n_static_, bg_.x(), bg_.y(), bg_.z(),
            avg_a.x(), avg_a.y(), avg_a.z(),
            rpy[0] * 180.0 / M_PI, rpy[1] * 180.0 / M_PI, rpy[2] * 180.0 / M_PI);
      }
      init_done_ = true;
    }
    last_gyro_ = g - bg_;
  }

  // -------------------------------------------------------------------------
  // Joint-state callback — main state update.  Runs at /joint_states rate.
  // -------------------------------------------------------------------------
  void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // cache latest q, q̇, effort (indexed by URDF joint name)
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto& name = msg->name[i];
      if (i < msg->effort.size()) efforts_[name] = msg->effort[i];
      const auto it = joint_map_.find(name);
      if (it != joint_map_.end()) {
        if (i < msg->position.size()) q_[it->second]  = msg->position[i];
        if (i < msg->velocity.size()) qd_[it->second] = msg->velocity[i];
      }
    }
    if (q_.size() < n_joints_expected_) return;

    const double t_abs = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if (last_t_ < 0.0) { last_t_ = t_abs; return; }
    const double dt = t_abs - last_t_;
    last_t_ = t_abs;
    if (dt <= 0.0 || dt > kMaxDt) return;

    // ---- 1. propagate R ----
    // Held at identity during the init window so position doesn't drift before
    // the gravity alignment takes effect (robot is static anyway).
    if (init_done_) {
      R_ = R_ * leg_odom::exp_so3(last_gyro_ * dt);
    }

    // ---- 2. FK Jacobian foot velocities (body frame) ----
    Eigen::Vector3d vL = kin_.foot_velocity_left(q_, qd_);
    Eigen::Vector3d vR = kin_.foot_velocity_right(q_, qd_);

    // ---- 3. heel-toe rolling compensation (optional) ----
    if (foot_roll_toe_offset_ > 0.0) {
      const auto itL = qd_.find("left_leg_ankle_pitch_joint");
      const auto itR = qd_.find("right_leg_ankle_pitch_joint");
      const double rate_L = (itL != qd_.end()) ? itL->second : 0.0;
      const double rate_R = (itR != qd_.end()) ? itR->second : 0.0;
      vL.x() += foot_roll_sign_ * foot_roll_toe_offset_ * rate_L;
      vR.x() += foot_roll_sign_ * foot_roll_toe_offset_ * rate_R;
    }

    // ---- 4. contact detection ----
    const double eL = efforts_.count(eff_joint_l_) ? efforts_[eff_joint_l_] : 0.0;
    const double eR = efforts_.count(eff_joint_r_) ? efforts_[eff_joint_r_] : 0.0;
    const auto [cl, cr] = det_->update(eL, eR);

    // ---- 5. accel-tilt Mahony (gated) ----
    tilt_applied_ = false;
    if (init_done_) {
      const double a_norm = last_accel_.norm();
      const bool stance_ok = !tilt_require_stance_ || cl || cr;
      if (stance_ok && a_norm > 1e-3 &&
          std::abs(a_norm - kGravity) < tilt_accel_band_) {
        const Eigen::Vector3d g_body_meas = last_accel_ / a_norm;
        const Eigen::Vector3d g_body_pred = R_.transpose() * Eigen::Vector3d(0, 0, 1);
        // err = meas × pred gives the body-frame rotation whose application
        // R ← R · exp(err·kp·dt) drives R^T · world_up from pred toward meas.
        Eigen::Vector3d err = g_body_meas.cross(g_body_pred);
        err.z() = 0.0;                          // keep yaw free
        R_ = R_ * leg_odom::exp_so3(tilt_kp_ * err * dt);
        tilt_applied_ = true;
      }
    }

    // ---- 6. body velocity in world ----
    Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
    if (cl && cr)      v_world = -R_ * (0.5 * (vL + vR));
    else if (cl)       v_world = -R_ * vL;
    else if (cr)       v_world = -R_ * vR;
    // swing: v_world = 0 (coast)

    // ---- 7. integrate position ----
    p_ += v_world * dt;

    // ---- 8. FlatZ clamp ----
    if (flatz_enabled_ && (cl || cr)) {
      p_.z() = (1.0 - flatz_alpha_) * p_.z();
    }

    if (diag_fp_) {
      const auto rpy = R_to_rpy(R_);
      std::fprintf(diag_fp_,
          "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
          "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n",
          t_abs, p_.x(), p_.y(), p_.z(),
          rpy[0], rpy[1], rpy[2],
          v_world.x(), v_world.y(), v_world.z(),
          vL.x(), vL.y(), vL.z(), vR.x(), vR.y(), vR.z(),
          static_cast<int>(cl), static_cast<int>(cr),
          static_cast<int>(tilt_applied_));
    }

    publish(msg->header.stamp, v_world);
  }

  void publish(const builtin_interfaces::msg::Time& stamp,
               const Eigen::Vector3d& v_world) {
    Eigen::Quaterniond q(R_);
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp         = stamp;
    odom.header.frame_id      = odom_frame_;
    odom.child_frame_id       = base_frame_;
    odom.pose.pose.position.x = p_.x();
    odom.pose.pose.position.y = p_.y();
    odom.pose.pose.position.z = p_.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x   = v_world.x();
    odom.twist.twist.linear.y   = v_world.y();
    odom.twist.twist.linear.z   = v_world.z();
    odom.twist.twist.angular.x  = last_gyro_.x();
    odom.twist.twist.angular.y  = last_gyro_.y();
    odom.twist.twist.angular.z  = last_gyro_.z();
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header               = odom.header;
      tf.child_frame_id       = base_frame_;
      tf.transform.translation.x = p_.x();
      tf.transform.translation.y = p_.y();
      tf.transform.translation.z = p_.z();
      tf.transform.rotation   = odom.pose.pose.orientation;
      tf_bc_->sendTransform(tf);
    }
  }

  // Return (roll, pitch, yaw). Uses Eigen's eulerAngles(2,1,0) which maps to
  // intrinsic ZYX and can yield a "180° flipped" decomposition for near-identity
  // R — benign for logging, the quaternion published on the odom topic is exact.
  static Eigen::Vector3d R_to_rpy(const Eigen::Matrix3d& R) {
    const Eigen::Vector3d ea = R.eulerAngles(2, 1, 0);  // yaw, pitch, roll
    return Eigen::Vector3d(ea[2], ea[1], ea[0]);
  }

  // --- kinematics / contact / config ---
  leg_odom::LegKinematics kin_;
  std::unique_ptr<leg_odom::ContactDetector> det_;
  std::map<std::string, std::string> joint_map_;
  size_t n_joints_expected_{0};
  std::string eff_joint_l_, eff_joint_r_;
  std::string odom_frame_, base_frame_;
  bool   publish_tf_{true};
  double bias_window_sec_{3.0};
  double tilt_kp_{1.0};
  double tilt_accel_band_{0.5};
  bool   tilt_require_stance_{true};
  bool   flatz_enabled_{true};
  double flatz_alpha_{0.05};
  double foot_roll_toe_offset_{0.0};
  double foot_roll_sign_{1.0};

  // --- state (the only variables the filter "owns") ---
  Eigen::Vector3d p_{Eigen::Vector3d::Zero()};         // world position
  Eigen::Matrix3d R_{Eigen::Matrix3d::Identity()};     // body→world rotation
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};        // gyro bias

  // --- IMU → joint-cb cache (latest-value hand-off between callbacks) ---
  Eigen::Vector3d last_gyro_{Eigen::Vector3d::Zero()}; // debiased gyro (raw during init)
  Eigen::Vector3d last_accel_{Eigen::Vector3d::Zero()};// raw accel

  // --- timing / diagnostics ---
  double last_t_{-1.0};                                // last /joint_states timestamp (s)
  bool   tilt_applied_{false};                         // last-frame Mahony gate flag

  // --- initialization window (accumulators used only until init_done_) ---
  double init_window_start_{-1.0};
  Eigen::Vector3d accum_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accum_accel_{Eigen::Vector3d::Zero()};
  int  n_static_{0};
  bool init_done_{false};

  // --- latest /joint_states contents (keyed by URDF joint name) ---
  std::map<std::string, double> q_, qd_, efforts_;

  // --- ROS2 I/O ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_bc_;

  // --- diagnostic CSV (opened iff diag_csv_path parameter non-empty) ---
  std::FILE* diag_fp_{nullptr};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FkOnlyNode>());
  rclcpp::shutdown();
  return 0;
}
