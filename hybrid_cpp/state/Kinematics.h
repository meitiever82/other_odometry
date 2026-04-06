#pragma once
/**
 * @file Kinematics.h
 * @brief C++ KDL forward kinematics for CASBot02 legs.
 * Mirrors Python leg_odometry/kinematics.py
 */

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace leg_odom {

class LegKinematics {
 public:
  LegKinematics() = default;

  bool init(const std::string& urdf_path,
            const std::string& base_link = "base_link",
            const std::string& left_foot_link = "left_leg_ankle_roll_link",
            const std::string& right_foot_link = "right_leg_ankle_roll_link") {
    // Read URDF
    std::ifstream f(urdf_path);
    if (!f.is_open()) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    std::string urdf_str = ss.str();

    // Parse to KDL tree
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_str, tree)) return false;

    // Extract chains
    if (!tree.getChain(base_link, left_foot_link, chain_left_)) return false;
    if (!tree.getChain(base_link, right_foot_link, chain_right_)) return false;

    fk_left_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_left_);
    fk_right_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_right_);

    // Extract joint names
    left_joint_names_ = get_joint_names(chain_left_);
    right_joint_names_ = get_joint_names(chain_right_);

    initialized_ = true;
    return true;
  }

  Eigen::Vector3d fk_left(const std::map<std::string, double>& joints) const {
    return compute_fk(chain_left_, *fk_left_, left_joint_names_, joints);
  }

  Eigen::Vector3d fk_right(const std::map<std::string, double>& joints) const {
    return compute_fk(chain_right_, *fk_right_, right_joint_names_, joints);
  }

  const std::vector<std::string>& left_joint_names() const { return left_joint_names_; }
  const std::vector<std::string>& right_joint_names() const { return right_joint_names_; }
  bool is_initialized() const { return initialized_; }

 private:
  static std::vector<std::string> get_joint_names(const KDL::Chain& chain) {
    std::vector<std::string> names;
    for (unsigned i = 0; i < chain.getNrOfSegments(); i++) {
      auto& joint = chain.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::Fixed) {
        names.push_back(joint.getName());
      }
    }
    return names;
  }

  static Eigen::Vector3d compute_fk(
      const KDL::Chain& chain,
      KDL::ChainFkSolverPos_recursive& fk_solver,
      const std::vector<std::string>& joint_names,
      const std::map<std::string, double>& joints) {
    unsigned n = chain.getNrOfJoints();
    KDL::JntArray q(n);
    for (unsigned i = 0; i < joint_names.size() && i < n; i++) {
      auto it = joints.find(joint_names[i]);
      q(i) = (it != joints.end()) ? it->second : 0.0;
    }

    KDL::Frame frame;
    fk_solver.JntToCart(q, frame);
    return Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
  }

  KDL::Chain chain_left_, chain_right_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_left_, fk_right_;
  std::vector<std::string> left_joint_names_, right_joint_names_;
  bool initialized_ = false;
};

// Joint mapping: bag name -> URDF name
inline std::map<std::string, std::string> default_joint_mapping() {
  return {
    {"LJ0", "left_leg_pelvic_pitch_joint"},
    {"LJ1", "left_leg_pelvic_roll_joint"},
    {"LJ2", "left_leg_pelvic_yaw_joint"},
    {"LJ3", "left_leg_knee_pitch_joint"},
    {"LJPITCH", "left_leg_ankle_pitch_joint"},
    {"LJROLL", "left_leg_ankle_roll_joint"},
    {"RJ6", "right_leg_pelvic_pitch_joint"},
    {"RJ7", "right_leg_pelvic_roll_joint"},
    {"RJ8", "right_leg_pelvic_yaw_joint"},
    {"RJ9", "right_leg_knee_pitch_joint"},
    {"RJPITCH", "right_leg_ankle_pitch_joint"},
    {"RJROLL", "right_leg_ankle_roll_joint"},
  };
}

// Contact detection (effort threshold with hysteresis)
class ContactDetector {
 public:
  ContactDetector(double threshold = 5.0, double hysteresis = 1.0)
      : threshold_(threshold), hysteresis_(hysteresis) {}

  std::pair<bool, bool> update(double effort_left, double effort_right) {
    contact_left_ = detect(std::abs(effort_left), contact_left_);
    contact_right_ = detect(std::abs(effort_right), contact_right_);
    return {contact_left_, contact_right_};
  }

 private:
  bool detect(double effort_abs, bool prev) {
    double upper = threshold_ + hysteresis_;
    double lower = threshold_ - hysteresis_;
    if (effort_abs > upper) return true;
    if (effort_abs < lower) return false;
    return prev;
  }

  double threshold_, hysteresis_;
  bool contact_left_ = false, contact_right_ = false;
};

}  // namespace leg_odom
