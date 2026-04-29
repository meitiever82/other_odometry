/**
 * @file test_kinematics.cpp
 * @brief Test C++ KDL FK against pre-computed FK from CSV.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

#include "leg_odometry/state/Kinematics.h"

int main(int argc, char** argv) {
  leg_odom::LegKinematics kin;
  // 默认走 cwd 相对路径（从 casbot_ws 根目录跑），可用 argv[1] 覆盖
  std::string urdf = (argc > 1) ? argv[1] : "urdf/casbot02_7dof_shell.urdf";

  if (!kin.init(urdf)) {
    std::cerr << "Failed to load URDF" << std::endl;
    return 1;
  }

  std::cout << "Left joints: ";
  for (auto& n : kin.left_joint_names()) std::cout << n << " ";
  std::cout << std::endl;

  std::cout << "Right joints: ";
  for (auto& n : kin.right_joint_names()) std::cout << n << " ";
  std::cout << std::endl;

  // FK at zero position
  std::map<std::string, double> zero_joints;
  for (auto& n : kin.left_joint_names()) zero_joints[n] = 0.0;
  for (auto& n : kin.right_joint_names()) zero_joints[n] = 0.0;

  auto fl = kin.fk_left(zero_joints);
  auto fr = kin.fk_right(zero_joints);

  std::cout << "FK left at zero: " << fl.transpose() << std::endl;
  std::cout << "FK right at zero: " << fr.transpose() << std::endl;
  std::cout << "(Python: [-0.0228  0.1425 -0.8215])" << std::endl;

  // Verify against CSV first line（同样走 cwd 相对路径）
  std::ifstream f("data/sim/csv/straight_medium.csv");
  std::string header, line;
  std::getline(f, header);
  std::getline(f, line);

  std::istringstream ss(line);
  std::string tok;
  auto next = [&]() -> double { std::getline(ss, tok, ','); return std::stod(tok); };

  next(); // timestamp
  next(); next(); next(); // accel
  next(); next(); next(); // gyro
  double csv_flx = next(), csv_fly = next(), csv_flz = next();
  double csv_frx = next(), csv_fry = next(), csv_frz = next();

  std::cout << "\nCSV FK left: " << csv_flx << " " << csv_fly << " " << csv_flz << std::endl;
  std::cout << "C++ FK left: " << fl.transpose() << std::endl;
  std::cout << "Match: " << ((std::abs(fl.x()-csv_flx) + std::abs(fl.y()-csv_fly) + std::abs(fl.z()-csv_flz)) < 0.001 ? "YES" : "NO") << std::endl;

  return 0;
}
