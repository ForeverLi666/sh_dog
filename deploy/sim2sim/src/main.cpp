#include "sh_dog/runtime/policy_bundle.hpp"
#include "sh_dog/runtime/policy_runtime.hpp"
#include "sh_dog/sim2sim/mujoco_backend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

double number(const char *value, const char *name) {
  try {
    return std::stod(value);
  } catch (...) {
    throw std::runtime_error(std::string("invalid ") + name + ": " + value);
  }
}

sh_dog::JointCommand standCommand(const sh_dog::PolicyBundle &bundle) {
  sh_dog::JointCommand result;
  result.joint_names = bundle.joint_order;
  result.desired_positions = bundle.default_joint_positions;
  result.desired_velocities.assign(bundle.action_dimension, 0.0);
  result.kp = bundle.stand_kp;
  result.kd = bundle.stand_kd;
  result.feedforward_torques.assign(bundle.action_dimension, 0.0);
  return result;
}

} // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "--check-model") {
      sh_dog::MujocoBackend::checkModel(argv[2]);
      return 0;
    }
    if (argc < 3) {
      std::cerr << "usage: sh_dog_sim2sim <policy_bundle> <scene.xml> "
                   "[duration_s] [vx vy wz] [--viewer]\n";
      return 2;
    }
    const double duration_s = argc > 3 ? number(argv[3], "duration") : 20.0;
    const std::array<double, 3> command{argc > 4 ? number(argv[4], "vx") : 0.5,
                                        argc > 5 ? number(argv[5], "vy") : 0.0,
                                        argc > 6 ? number(argv[6], "wz") : 0.0};
    const bool viewer = argc > 7 && std::string(argv[7]) == "--viewer";
    sh_dog::PolicyBundle bundle = sh_dog::PolicyBundle::load(argv[1]);
    sh_dog::PolicyRuntime runtime(bundle);
    sh_dog::MujocoBackend backend(argv[2], bundle.joint_order,
                                  bundle.default_joint_positions, viewer);
    const auto ratio = bundle.control_period_s / backend.timestep();
    const int decimation = static_cast<int>(std::llround(ratio));
    if (decimation <= 0 || std::abs(ratio - decimation) > 1.0e-9) {
      throw std::runtime_error(
          "policy period must be an integer multiple of MuJoCo timestep");
    }
    sh_dog::JointCommand joint_command = standCommand(bundle);
    int simulation_step = 0;
    constexpr double warmup_s = 1.0;
    while (!backend.shouldExit() &&
           backend.read().time_s < duration_s + warmup_s) {
      const sh_dog::RobotState state = backend.read();
      if (state.base_position[2] < 0.15 ||
          !std::isfinite(state.base_position[2])) {
        throw std::runtime_error("base height safety stop");
      }
      if (state.time_s >= warmup_s && simulation_step % decimation == 0) {
        joint_command = runtime.step(state, command);
      }
      backend.write(joint_command);
      ++simulation_step;
      if (simulation_step % 400 == 0) {
        std::cout << "t=" << state.time_s - warmup_s
                  << " base=" << state.base_position[0] << ","
                  << state.base_position[1] << "," << state.base_position[2]
                  << "\n";
      }
    }
    const sh_dog::RobotState final_state = backend.read();
    std::cout << "PASS duration=" << duration_s
              << " final_base=" << final_state.base_position[0] << ","
              << final_state.base_position[1] << ","
              << final_state.base_position[2] << "\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "sim2sim error: " << error.what() << "\n";
    return 1;
  }
}
