/**
 * @file simple_leg_odom_node.cpp
 * @brief 极简腿式里程计：姿态只看 IMU，平移只看腿 FK。
 *
 * 姿态:
 *   - roll/pitch 来自 accel 重力方向（低通）
 *   - yaw 来自 gyro z 的积分
 *   - 不估 IMU bias、不做 EKF、无 FK→R 反馈
 *
 * 平移（world frame）:
 *   - stance 脚作为世界锚点: world_foot = p + R * FK(q)
 *   - dp = R_prev * FK_prev - R_cur * FK_cur
 *   - 两脚都 contact 时取平均，都不 contact 时不更新
 *   - contact 由 LJPITCH / RJPITCH 的 effort 阈值判
 *
 * 订阅:   /joint_states, /imu
 * 发布:   /leg_odometry_simple (nav_msgs/Odometry),
 *        TF odom -> base_link_simple
 */

#include <memory>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "leg_odometry/state/Kinematics.h"

using namespace leg_odom;

class SimpleLegOdomNode : public rclcpp::Node {
 public:
  SimpleLegOdomNode() : Node("simple_leg_odom") {
    declare_parameter("urdf_path", "");
    declare_parameter("effort_threshold_left", 5.0);
    declare_parameter("effort_threshold_right", 5.0);
    declare_parameter("effort_hysteresis", 1.0);
    declare_parameter("effort_joint_left", std::string("LJPITCH"));
    declare_parameter("effort_joint_right", std::string("RJPITCH"));
    declare_parameter("tilt_alpha", 0.01);     // roll/pitch 向 accel 收敛的系数 (0~1)
    declare_parameter("init_frames", 50);
    declare_parameter("publish_tf", true);
    declare_parameter("odom_frame", std::string("odom"));
    declare_parameter("base_frame", std::string("base_link_simple"));

    auto urdf = get_parameter("urdf_path").as_string();
    if (urdf.empty() || !kin_.init(urdf)) {
      RCLCPP_FATAL(get_logger(), "URDF load failed: '%s'", urdf.c_str());
      throw std::runtime_error("urdf init failed");
    }

    double th_l = get_parameter("effort_threshold_left").as_double();
    double th_r = get_parameter("effort_threshold_right").as_double();
    double hys  = get_parameter("effort_hysteresis").as_double();
    det_l_ = std::make_unique<ContactDetector>(th_l, hys);
    det_r_ = std::make_unique<ContactDetector>(th_r, hys);

    eff_joint_l_ = get_parameter("effort_joint_left").as_string();
    eff_joint_r_ = get_parameter("effort_joint_right").as_string();
    tilt_alpha_  = get_parameter("tilt_alpha").as_double();
    init_frames_ = get_parameter("init_frames").as_int();
    publish_tf_  = get_parameter("publish_tf").as_bool();
    odom_frame_  = get_parameter("odom_frame").as_string();
    base_frame_  = get_parameter("base_frame").as_string();

    joint_map_ = default_joint_mapping();

    tf_bc_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto qos = rclcpp::SensorDataQoS();
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos,
        std::bind(&SimpleLegOdomNode::joint_cb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", qos,
        std::bind(&SimpleLegOdomNode::imu_cb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/leg_odometry_simple", 10);

    R_ = Eigen::Matrix3d::Identity();
    p_.setZero();
    v_.setZero();

    RCLCPP_INFO(get_logger(), "simple_leg_odom started (th_l=%.2f, th_r=%.2f, alpha=%.3f)",
                th_l, th_r, tilt_alpha_);
  }

 private:
  void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
      const auto& n = msg->name[i];
      if (i < msg->effort.size()) efforts_[n] = msg->effort[i];
      auto it = joint_map_.find(n);
      if (it != joint_map_.end() && i < msg->position.size()) {
        joints_[it->second] = msg->position[i];
      }
    }
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Eigen::Vector3d a(msg->linear_acceleration.x,
                      msg->linear_acceleration.y,
                      msg->linear_acceleration.z);
    Eigen::Vector3d w(msg->angular_velocity.x,
                      msg->angular_velocity.y,
                      msg->angular_velocity.z);
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    // --- Initialization: 用前 N 帧 accel 平均方向对齐 roll/pitch ---
    if (!initialized_) {
      a_init_ += a;
      init_count_++;
      if (init_count_ < init_frames_ || joints_.empty()) return;
      Eigen::Vector3d g_body = (a_init_ / init_count_).normalized();
      // 目标: R^T * [0,0,1] = g_body  (body 系下重力方向)
      Eigen::Vector3d z_world(0, 0, 1);
      Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(g_body, z_world);
      R_ = q.toRotationMatrix();
      last_t_ = t;
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "simple_leg_odom initialized");
      publish(msg->header.stamp);
      return;
    }

    double dt = t - last_t_;
    last_t_ = t;
    if (dt <= 0 || dt > 0.1) return;

    // --- 1) 姿态积分: R += R * skew(w) * dt (exp map) ---
    Eigen::Vector3d w_dt = w * dt;
    double theta = w_dt.norm();
    if (theta > 1e-9) {
      Eigen::Matrix3d K;
      Eigen::Vector3d axis = w_dt / theta;
      K <<     0, -axis.z(),  axis.y(),
           axis.z(),      0, -axis.x(),
          -axis.y(), axis.x(),       0;
      Eigen::Matrix3d dR = Eigen::Matrix3d::Identity()
                           + std::sin(theta) * K
                           + (1 - std::cos(theta)) * K * K;
      R_ = R_ * dR;
    }

    // --- 2) Complementary filter: 把 roll/pitch 向 accel 重力方向拉 ---
    if (a.norm() > 1e-3) {
      Eigen::Vector3d g_body_meas = a.normalized();
      Eigen::Vector3d g_body_pred = R_.transpose() * Eigen::Vector3d(0, 0, 1);
      Eigen::Vector3d err = g_body_pred.cross(g_body_meas);  // 小角度旋转向量 (body)
      // 只修正 roll/pitch: 投到 body x,y 两轴（剔除 z 分量以不动 yaw）
      err.z() = 0;
      Eigen::Vector3d corr = tilt_alpha_ * err;
      double cn = corr.norm();
      if (cn > 1e-9) {
        Eigen::AngleAxisd aa(cn, corr / cn);
        R_ = R_ * aa.toRotationMatrix();
      }
    }
    // 正交化（防数值漂）
    Eigen::Quaterniond q(R_);
    q.normalize();
    R_ = q.toRotationMatrix();

    // --- 3) 平移: FK 差分 ---
    if (!joints_.empty()) {
      auto fl = kin_.fk_left(joints_);
      auto fr = kin_.fk_right(joints_);
      double el = 0, er = 0;
      auto itl = efforts_.find(eff_joint_l_);
      if (itl != efforts_.end()) el = itl->second;
      auto itr = efforts_.find(eff_joint_r_);
      if (itr != efforts_.end()) er = itr->second;
      auto [cl, _cl] = det_l_->update(std::abs(el), 0.0);
      auto [cr, _cr] = det_r_->update(std::abs(er), 0.0);

      if (have_prev_fk_) {
        Eigen::Vector3d dp = Eigen::Vector3d::Zero();
        int n = 0;
        if (cl && prev_cl_) { dp += R_prev_ * fl_prev_ - R_ * fl; n++; }
        if (cr && prev_cr_) { dp += R_prev_ * fr_prev_ - R_ * fr; n++; }
        if (n > 0) {
          dp /= n;
          p_ += dp;
          v_ = dp / dt;
        } else {
          v_.setZero();  // 都不 contact: 不外推
        }
      }

      R_prev_ = R_;
      fl_prev_ = fl;
      fr_prev_ = fr;
      prev_cl_ = cl;
      prev_cr_ = cr;
      have_prev_fk_ = true;
    }

    publish(msg->header.stamp);
  }

  void publish(const builtin_interfaces::msg::Time& stamp) {
    Eigen::Quaterniond q(R_);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = p_.x();
    odom.pose.pose.position.y = p_.y();
    odom.pose.pose.position.z = p_.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = v_.x();
    odom.twist.twist.linear.y = v_.y();
    odom.twist.twist.linear.z = v_.z();
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = p_.x();
      tf.transform.translation.y = p_.y();
      tf.transform.translation.z = p_.z();
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_bc_->sendTransform(tf);
    }
  }

  // Config
  LegKinematics kin_;
  std::unique_ptr<ContactDetector> det_l_, det_r_;
  std::map<std::string, std::string> joint_map_;
  std::string eff_joint_l_, eff_joint_r_, odom_frame_, base_frame_;
  double tilt_alpha_{0.01};
  int init_frames_{50};
  bool publish_tf_{true};

  // State
  Eigen::Matrix3d R_, R_prev_;
  Eigen::Vector3d p_, v_;
  Eigen::Vector3d fl_prev_, fr_prev_;
  bool prev_cl_{false}, prev_cr_{false};
  bool have_prev_fk_{false};
  bool initialized_{false};
  double last_t_{-1};

  // Init
  Eigen::Vector3d a_init_{Eigen::Vector3d::Zero()};
  int init_count_{0};

  // Buffers
  std::map<std::string, double> joints_;
  std::map<std::string, double> efforts_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleLegOdomNode>());
  rclcpp::shutdown();
  return 0;
}
