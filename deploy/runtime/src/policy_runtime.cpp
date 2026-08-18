#include "sh_dog/runtime/policy_runtime.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace sh_dog {
namespace {

void requireShape(const std::vector<int64_t> &actual,
                  const std::vector<int64_t> &expected,
                  const std::string &name) {
  if (actual != expected)
    throw std::runtime_error("ONNX tensor shape mismatch: " + name);
}

std::array<float, 3> projectedGravity(const std::array<double, 4> &quaternion) {
  double w = quaternion[0], x = quaternion[1], y = quaternion[2],
         z = quaternion[3];
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (!std::isfinite(norm) || norm < 1.0e-8)
    throw std::runtime_error("invalid base quaternion");
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
  return {static_cast<float>(-2.0 * (x * z - w * y)),
          static_cast<float>(-2.0 * (y * z + w * x)),
          static_cast<float>(-(1.0 - 2.0 * (x * x + y * y)))};
}

} // namespace

class PolicyRuntime::Impl {
public:
  explicit Impl(const PolicyBundle &bundle)
      : env_(ORT_LOGGING_LEVEL_WARNING, "sh_dog_policy"),
        session_(env_, bundle.model_path.c_str(), sessionOptions()),
        memory_(
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
    Ort::AllocatorWithDefaultOptions allocator;
    if (session_.GetInputCount() != (bundle.recurrent ? 2U : 1U) ||
        session_.GetOutputCount() != (bundle.recurrent ? 2U : 1U)) {
      throw std::runtime_error(
          "ONNX input/output count disagrees with manifest");
    }
    input_names_storage_.reserve(session_.GetInputCount());
    output_names_storage_.reserve(session_.GetOutputCount());
    input_names_.reserve(session_.GetInputCount());
    output_names_.reserve(session_.GetOutputCount());
    for (std::size_t i = 0; i < session_.GetInputCount(); ++i) {
      auto name = session_.GetInputNameAllocated(i, allocator);
      input_names_storage_.emplace_back(name.get());
      input_names_.push_back(input_names_storage_.back().c_str());
    }
    for (std::size_t i = 0; i < session_.GetOutputCount(); ++i) {
      auto name = session_.GetOutputNameAllocated(i, allocator);
      output_names_storage_.emplace_back(name.get());
      output_names_.push_back(output_names_storage_.back().c_str());
    }
    if (input_names_storage_[0] != "obs" ||
        output_names_storage_[0] != "actions") {
      throw std::runtime_error(
          "ONNX must expose obs -> actions as its first tensors");
    }
    requireShape(
        session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape(),
        {1, static_cast<int64_t>(bundle.observation_dimension)}, "obs");
    requireShape(
        session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape(),
        {1, static_cast<int64_t>(bundle.action_dimension)}, "actions");
    if (bundle.recurrent) {
      if (input_names_storage_[1] != "h_in" ||
          output_names_storage_[1] != "h_out") {
        throw std::runtime_error("recurrent ONNX must expose h_in -> h_out");
      }
      hidden_shape_ = {static_cast<int64_t>(bundle.recurrent_num_layers), 1,
                       static_cast<int64_t>(bundle.recurrent_hidden_size)};
      requireShape(
          session_.GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape(),
          hidden_shape_, "h_in");
      requireShape(
          session_.GetOutputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape(),
          hidden_shape_, "h_out");
      hidden_.assign(bundle.recurrent_num_layers * bundle.recurrent_hidden_size,
                     0.0F);
    }
  }

  std::vector<float> infer(const std::vector<float> &observation,
                           std::size_t action_dimension) {
    std::array<int64_t, 2> observation_shape{
        1, static_cast<int64_t>(observation.size())};
    std::vector<Ort::Value> inputs;
    inputs.push_back(Ort::Value::CreateTensor<float>(
        memory_, const_cast<float *>(observation.data()), observation.size(),
        observation_shape.data(), observation_shape.size()));
    if (!hidden_.empty()) {
      inputs.push_back(Ort::Value::CreateTensor<float>(
          memory_, hidden_.data(), hidden_.size(), hidden_shape_.data(),
          hidden_shape_.size()));
    }
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_.data(),
                                inputs.data(), inputs.size(),
                                output_names_.data(), output_names_.size());
    const float *action = outputs[0].GetTensorData<float>();
    std::vector<float> result(action, action + action_dimension);
    if (!hidden_.empty()) {
      const float *next_hidden = outputs[1].GetTensorData<float>();
      std::copy(next_hidden, next_hidden + hidden_.size(), hidden_.begin());
    }
    if (!std::all_of(result.begin(), result.end(),
                     [](float value) { return std::isfinite(value); })) {
      throw std::runtime_error("policy produced a non-finite action");
    }
    return result;
  }

  void reset() { std::fill(hidden_.begin(), hidden_.end(), 0.0F); }

private:
  static Ort::SessionOptions sessionOptions() {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    return options;
  }

  Ort::Env env_;
  Ort::Session session_;
  Ort::MemoryInfo memory_;
  std::vector<std::string> input_names_storage_;
  std::vector<std::string> output_names_storage_;
  std::vector<const char *> input_names_;
  std::vector<const char *> output_names_;
  std::vector<int64_t> hidden_shape_;
  std::vector<float> hidden_;
};

PolicyRuntime::PolicyRuntime(PolicyBundle bundle)
    : bundle_(std::move(bundle)), impl_(std::make_unique<Impl>(bundle_)),
      last_action_(bundle_.action_dimension, 0.0F) {}
PolicyRuntime::~PolicyRuntime() = default;
PolicyRuntime::PolicyRuntime(PolicyRuntime &&) noexcept = default;
PolicyRuntime &PolicyRuntime::operator=(PolicyRuntime &&) noexcept = default;

JointCommand PolicyRuntime::step(const RobotState &state,
                                 const std::array<double, 3> &command) {
  const std::size_t dof = bundle_.joint_order.size();
  if (state.joint_names.size() != state.joint_positions.size() ||
      state.joint_names.size() != state.joint_velocities.size()) {
    throw std::runtime_error("RobotState joint arrays have inconsistent sizes");
  }
  std::unordered_map<std::string, std::size_t> state_index;
  for (std::size_t i = 0; i < state.joint_names.size(); ++i)
    state_index.emplace(state.joint_names[i], i);
  std::vector<float> observation;
  observation.reserve(bundle_.observation_dimension);
  const auto &terms = bundle_.observation_terms;
  const auto append = [&observation](double value,
                                     const ObservationTerm &term) {
    observation.push_back(static_cast<float>(
        std::clamp(value, term.clip_min, term.clip_max) * term.scale));
  };
  for (double value : state.base_angular_velocity)
    append(value, terms[0]);
  for (float value : projectedGravity(state.base_quaternion_wxyz))
    append(value, terms[1]);
  for (double value : command)
    append(value, terms[2]);
  for (std::size_t i = 0; i < dof; ++i) {
    const auto found = state_index.find(bundle_.joint_order[i]);
    if (found == state_index.end())
      throw std::runtime_error("RobotState is missing joint: " +
                               bundle_.joint_order[i]);
    append(state.joint_positions[found->second] -
               bundle_.default_joint_positions[i],
           terms[3]);
  }
  for (const std::string &name : bundle_.joint_order) {
    append(state.joint_velocities[state_index.at(name)], terms[4]);
  }
  for (float value : last_action_)
    append(value, terms[5]);
  if (observation.size() != bundle_.observation_dimension ||
      !std::all_of(observation.begin(), observation.end(),
                   [](float value) { return std::isfinite(value); })) {
    throw std::runtime_error("observation is invalid");
  }
  last_action_ = impl_->infer(observation, bundle_.action_dimension);
  JointCommand result;
  result.joint_names = bundle_.joint_order;
  result.desired_positions.resize(dof);
  for (std::size_t i = 0; i < dof; ++i) {
    double action = static_cast<double>(last_action_[i]);
    if (bundle_.action_clip)
      action = std::clamp(action, -*bundle_.action_clip, *bundle_.action_clip);
    result.desired_positions[i] =
        bundle_.default_joint_positions[i] + bundle_.action_scale[i] * action;
  }
  result.desired_velocities.assign(dof, 0.0);
  result.feedforward_torques.assign(dof, 0.0);
  result.kp = bundle_.kp;
  result.kd = bundle_.kd;
  return result;
}

void PolicyRuntime::reset() {
  std::fill(last_action_.begin(), last_action_.end(), 0.0F);
  impl_->reset();
}

} // namespace sh_dog
