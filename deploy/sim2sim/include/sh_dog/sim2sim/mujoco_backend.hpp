#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "sh_dog/runtime/types.hpp"

namespace sh_dog {

class MujocoBackend {
public:
  MujocoBackend(const std::filesystem::path &scene_path,
                const std::vector<std::string> &joint_names,
                const std::vector<double> &initial_joint_positions,
                bool viewer);
  ~MujocoBackend();
  MujocoBackend(const MujocoBackend &) = delete;
  MujocoBackend &operator=(const MujocoBackend &) = delete;

  RobotState read() const;
  void write(const JointCommand &command);
  bool shouldExit() const;
  double timestep() const;
  static void checkModel(const std::filesystem::path &scene_path);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace sh_dog
