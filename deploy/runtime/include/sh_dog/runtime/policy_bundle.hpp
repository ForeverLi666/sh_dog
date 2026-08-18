#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace sh_dog {

struct ObservationTerm {
  std::string name;
  std::size_t size{0};
  double clip_min{0.0};
  double clip_max{0.0};
  double scale{1.0};
};

struct PolicyBundle {
  std::filesystem::path directory;
  std::filesystem::path model_path;
  std::vector<std::string> joint_order;
  std::vector<ObservationTerm> observation_terms;
  std::vector<double> action_scale;
  std::vector<double> default_joint_positions;
  std::vector<double> kp;
  std::vector<double> kd;
  std::vector<double> stand_kp;
  std::vector<double> stand_kd;
  std::size_t observation_dimension{0};
  std::size_t action_dimension{0};
  std::optional<double> action_clip;
  double control_period_s{0.0};
  bool recurrent{false};
  std::size_t recurrent_hidden_size{0};
  std::size_t recurrent_num_layers{0};

  static PolicyBundle load(const std::filesystem::path &directory);
};

} // namespace sh_dog
