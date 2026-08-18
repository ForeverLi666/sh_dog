#include "sh_dog/runtime/policy_bundle.hpp"

#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace sh_dog {
namespace {

std::string sha256(const std::filesystem::path &path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream)
    throw std::runtime_error("cannot open bundle file: " + path.string());
  EVP_MD_CTX *context = EVP_MD_CTX_new();
  if (!context || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("failed to initialize SHA-256");
  }
  std::array<char, 64 * 1024> buffer{};
  while (stream) {
    stream.read(buffer.data(), buffer.size());
    if (EVP_DigestUpdate(context, buffer.data(),
                         static_cast<std::size_t>(stream.gcount())) != 1) {
      EVP_MD_CTX_free(context);
      throw std::runtime_error("failed to update SHA-256");
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context, digest.data(), &digest_size) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("failed to finalize SHA-256");
  }
  EVP_MD_CTX_free(context);
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < digest_size; ++i)
    result << std::setw(2) << static_cast<int>(digest[i]);
  return result.str();
}

void verifyChecksums(const std::filesystem::path &directory) {
  std::ifstream checksums(directory / "checksum.sha256");
  if (!checksums)
    throw std::runtime_error("policy bundle is missing checksum.sha256");
  std::set<std::string> verified;
  std::string expected, filename;
  while (checksums >> expected >> filename) {
    if (filename != "policy.onnx" && filename != "manifest.yaml") {
      throw std::runtime_error("unexpected checksum target: " + filename);
    }
    if (!verified.insert(filename).second ||
        sha256(directory / filename) != expected) {
      throw std::runtime_error("checksum mismatch: " + filename);
    }
  }
  if (verified != std::set<std::string>{"manifest.yaml", "policy.onnx"}) {
    throw std::runtime_error(
        "checksum.sha256 must cover manifest.yaml and policy.onnx");
  }
}

template <typename T>
std::vector<T> requiredVector(const YAML::Node &node, const std::string &name,
                              std::size_t size) {
  if (!node || !node.IsSequence() || node.size() != size) {
    throw std::runtime_error(name + " must have " + std::to_string(size) +
                             " entries");
  }
  return node.as<std::vector<T>>();
}

} // namespace

PolicyBundle PolicyBundle::load(const std::filesystem::path &directory) {
  verifyChecksums(directory);
  const YAML::Node root =
      YAML::LoadFile((directory / "manifest.yaml").string());
  if (root["schema_version"].as<int>() != 1)
    throw std::runtime_error("unsupported policy bundle schema");
  if (root["model"]["format"].as<std::string>() != "onnx" ||
      root["model"]["file"].as<std::string>() != "policy.onnx") {
    throw std::runtime_error(
        "schema 1 requires model.file=policy.onnx and format=onnx");
  }
  PolicyBundle bundle;
  bundle.directory = std::filesystem::canonical(directory);
  bundle.model_path = bundle.directory / "policy.onnx";
  bundle.joint_order =
      root["robot"]["joint_order"].as<std::vector<std::string>>();
  bundle.observation_dimension =
      root["observation"]["dimension"].as<std::size_t>();
  if (root["observation"]["history_length"].as<std::size_t>() != 1 ||
      root["observation"]["corruption"].as<bool>()) {
    throw std::runtime_error(
        "schema 1 runtime requires one uncorrupted observation frame");
  }
  bundle.action_dimension = root["action"]["dimension"].as<std::size_t>();
  bundle.action_scale = requiredVector<double>(
      root["action"]["scale"], "action.scale", bundle.action_dimension);
  bundle.default_joint_positions = requiredVector<double>(
      root["action"]["default_joint_positions"],
      "action.default_joint_positions", bundle.action_dimension);
  bundle.kp =
      requiredVector<double>(root["control"]["motion"]["kp"],
                             "control.motion.kp", bundle.action_dimension);
  bundle.kd =
      requiredVector<double>(root["control"]["motion"]["kd"],
                             "control.motion.kd", bundle.action_dimension);
  bundle.stand_kp =
      requiredVector<double>(root["control"]["stand"]["kp"], "control.stand.kp",
                             bundle.action_dimension);
  bundle.stand_kd =
      requiredVector<double>(root["control"]["stand"]["kd"], "control.stand.kd",
                             bundle.action_dimension);
  const YAML::Node action_clip = root["action"]["clip"];
  if (action_clip && !action_clip.IsNull())
    bundle.action_clip = action_clip.as<double>();
  bundle.control_period_s = root["control"]["period_s"].as<double>();
  bundle.recurrent = root["model"]["recurrent"].as<bool>();
  const auto obs_shape = requiredVector<std::size_t>(
      root["model"]["inputs"]["obs"], "model.inputs.obs", 2);
  const auto action_shape = requiredVector<std::size_t>(
      root["model"]["outputs"]["actions"], "model.outputs.actions", 2);
  if (obs_shape != std::vector<std::size_t>{1, bundle.observation_dimension} ||
      action_shape != std::vector<std::size_t>{1, bundle.action_dimension}) {
    throw std::runtime_error(
        "declared ONNX observation/action shapes disagree with dimensions");
  }
  if (bundle.recurrent) {
    const auto hidden_shape = requiredVector<std::size_t>(
        root["model"]["inputs"]["h_in"], "model.inputs.h_in", 3);
    const auto hidden_output_shape = requiredVector<std::size_t>(
        root["model"]["outputs"]["h_out"], "model.outputs.h_out", 3);
    if (hidden_shape != hidden_output_shape || hidden_shape[1] != 1) {
      throw std::runtime_error(
          "declared recurrent input/output shapes disagree");
    }
    bundle.recurrent_num_layers = hidden_shape[0];
    bundle.recurrent_hidden_size = hidden_shape[2];
  }
  for (const YAML::Node &term : root["observation"]["terms"]) {
    const auto clip =
        requiredVector<double>(term["clip"], "observation term clip", 2);
    if (clip[0] > clip[1])
      throw std::runtime_error("observation term clip is reversed");
    bundle.observation_terms.push_back({term["name"].as<std::string>(),
                                        term["size"].as<std::size_t>(), clip[0],
                                        clip[1], term["scale"].as<double>()});
  }
  const std::vector<std::string> expected_terms = {
      "base_ang_vel",         "projected_gravity", "velocity_command",
      "joint_position_error", "joint_velocity",    "last_action"};
  std::size_t total = 0;
  if (bundle.observation_terms.size() != expected_terms.size())
    throw std::runtime_error("unsupported observation contract");
  for (std::size_t i = 0; i < expected_terms.size(); ++i) {
    if (bundle.observation_terms[i].name != expected_terms[i])
      throw std::runtime_error("unsupported observation term order");
    total += bundle.observation_terms[i].size;
  }
  if (total != bundle.observation_dimension ||
      bundle.joint_order.size() != bundle.action_dimension ||
      bundle.action_dimension == 0 || bundle.control_period_s <= 0.0 ||
      (bundle.action_clip && *bundle.action_clip <= 0.0)) {
    throw std::runtime_error(
        "inconsistent policy bundle dimensions or control values");
  }
  if (bundle.recurrent &&
      (bundle.recurrent_hidden_size == 0 || bundle.recurrent_num_layers == 0)) {
    throw std::runtime_error("invalid recurrent state declaration");
  }
  return bundle;
}

} // namespace sh_dog
