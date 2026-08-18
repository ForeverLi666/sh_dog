#pragma once

#include <array>
#include <string>
#include <vector>

namespace sh_dog {

struct RobotState {
  double time_s{0.0};
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  std::array<double, 4> base_quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> base_angular_velocity{0.0, 0.0, 0.0};
  std::array<double, 3> base_position{0.0, 0.0, 0.0};
};

struct JointCommand {
  std::vector<std::string> joint_names;
  std::vector<double> desired_positions;
  std::vector<double> desired_velocities;
  std::vector<double> kp;
  std::vector<double> kd;
  std::vector<double> feedforward_torques;
};

} // namespace sh_dog
