#pragma once

#include <array>
#include <memory>
#include <vector>

#include "sh_dog/runtime/policy_bundle.hpp"
#include "sh_dog/runtime/types.hpp"

namespace sh_dog {

class PolicyRuntime {
public:
  explicit PolicyRuntime(PolicyBundle bundle);
  ~PolicyRuntime();
  PolicyRuntime(PolicyRuntime &&) noexcept;
  PolicyRuntime &operator=(PolicyRuntime &&) noexcept;
  PolicyRuntime(const PolicyRuntime &) = delete;
  PolicyRuntime &operator=(const PolicyRuntime &) = delete;

  JointCommand step(const RobotState &state,
                    const std::array<double, 3> &velocity_command);
  void reset();
  const PolicyBundle &bundle() const { return bundle_; }

private:
  class Impl;
  PolicyBundle bundle_;
  std::unique_ptr<Impl> impl_;
  std::vector<float> last_action_;
};

} // namespace sh_dog
