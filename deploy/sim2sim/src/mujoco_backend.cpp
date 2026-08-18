#include "sh_dog/sim2sim/mujoco_backend.hpp"

#include <mujoco/mujoco.h>
#ifdef SH_DOG_WITH_GLFW
#include <GLFW/glfw3.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

namespace sh_dog {
namespace {

std::array<double, 3>
rotateInverse(const std::array<double, 4> &quaternion_wxyz,
              const mjtNum *vector_world) {
  const double w = quaternion_wxyz[0];
  const double x = quaternion_wxyz[1];
  const double y = quaternion_wxyz[2];
  const double z = quaternion_wxyz[3];
  const double tx = 2.0 * (y * vector_world[2] - z * vector_world[1]);
  const double ty = 2.0 * (z * vector_world[0] - x * vector_world[2]);
  const double tz = 2.0 * (x * vector_world[1] - y * vector_world[0]);
  return {vector_world[0] - w * tx + y * tz - z * ty,
          vector_world[1] - w * ty + z * tx - x * tz,
          vector_world[2] - w * tz + x * ty - y * tx};
}

} // namespace

class MujocoBackend::Impl {
public:
  Impl(const std::filesystem::path &scene_path,
       const std::vector<std::string> &names,
       const std::vector<double> &initial_joint_positions, bool enable_viewer)
      : joint_names(names) {
    if (initial_joint_positions.size() != names.size()) {
      throw std::runtime_error("initial joint position dimension mismatch");
    }
    char error[2048]{};
    model = mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
    if (!model)
      throw std::runtime_error("failed to load MuJoCo scene: " +
                               std::string(error));
    data = mj_makeData(model);
    if (!data)
      throw std::runtime_error("failed to allocate MuJoCo data");
    const int free_joint = mj_name2id(model, mjOBJ_JOINT, "floating_base");
    if (free_joint < 0 || model->jnt_type[free_joint] != mjJNT_FREE) {
      throw std::runtime_error(
          "MuJoCo model is missing floating_base free joint");
    }
    base_qpos_adr = model->jnt_qposadr[free_joint];
    base_body_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    for (const std::string &name : joint_names) {
      const int joint_id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
      const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
      if (joint_id < 0 || actuator_id < 0 ||
          model->jnt_type[joint_id] != mjJNT_HINGE) {
        throw std::runtime_error("MuJoCo joint/actuator is missing: " + name);
      }
      qpos_adrs.push_back(model->jnt_qposadr[joint_id]);
      dof_adrs.push_back(model->jnt_dofadr[joint_id]);
      actuator_ids.push_back(actuator_id);
    }
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      data->qpos[qpos_adrs[i]] = initial_joint_positions[i];
    }
    mj_forward(model, data);
#ifdef SH_DOG_WITH_GLFW
    if (enable_viewer)
      initViewer();
#else
    if (enable_viewer)
      throw std::runtime_error(
          "viewer requested but GLFW was unavailable at build time");
#endif
  }

  ~Impl() {
#ifdef SH_DOG_WITH_GLFW
    if (window) {
      mjr_freeContext(&context);
      mjv_freeScene(&scene);
      glfwDestroyWindow(window);
      glfwTerminate();
    }
#endif
    if (data)
      mj_deleteData(data);
    if (model)
      mj_deleteModel(model);
  }

  RobotState read() const {
    RobotState result;
    result.time_s = data->time;
    result.joint_names = joint_names;
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      result.joint_positions.push_back(data->qpos[qpos_adrs[i]]);
      result.joint_velocities.push_back(data->qvel[dof_adrs[i]]);
    }
    result.base_position = {data->qpos[base_qpos_adr],
                            data->qpos[base_qpos_adr + 1],
                            data->qpos[base_qpos_adr + 2]};
    result.base_quaternion_wxyz = {
        data->qpos[base_qpos_adr + 3], data->qpos[base_qpos_adr + 4],
        data->qpos[base_qpos_adr + 5], data->qpos[base_qpos_adr + 6]};
    // MuJoCo's local BODY velocity uses the principal-inertia frame. Isaac Lab
    // defines root_ang_vel_b in the root link frame, so rotate the world-frame
    // angular velocity with the floating-base link quaternion explicitly.
    mjtNum velocity_world[6]{};
    mj_objectVelocity(model, data, mjOBJ_BODY, base_body_id, velocity_world, 0);
    result.base_angular_velocity =
        rotateInverse(result.base_quaternion_wxyz, velocity_world);
    return result;
  }

  void write(const JointCommand &command) {
    if (command.joint_names.size() != command.desired_positions.size() ||
        command.joint_names.size() != command.desired_velocities.size() ||
        command.joint_names.size() != command.kp.size() ||
        command.joint_names.size() != command.kd.size() ||
        command.joint_names.size() != command.feedforward_torques.size()) {
      throw std::runtime_error("JointCommand arrays have inconsistent sizes");
    }
    std::unordered_map<std::string, std::size_t> command_index;
    for (std::size_t i = 0; i < command.joint_names.size(); ++i)
      command_index.emplace(command.joint_names[i], i);
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      const auto found = command_index.find(joint_names[i]);
      if (found == command_index.end())
        throw std::runtime_error("JointCommand is missing joint: " +
                                 joint_names[i]);
      const std::size_t j = found->second;
      data->ctrl[actuator_ids[i]] =
          command.kp[j] *
              (command.desired_positions[j] - data->qpos[qpos_adrs[i]]) +
          command.kd[j] *
              (command.desired_velocities[j] - data->qvel[dof_adrs[i]]) +
          command.feedforward_torques[j];
    }
    mj_step(model, data);
#ifdef SH_DOG_WITH_GLFW
    render();
#endif
  }

#ifdef SH_DOG_WITH_GLFW
  void initViewer() {
    if (!glfwInit())
      throw std::runtime_error("GLFW initialization failed");
    window = glfwCreateWindow(1280, 900, "ShDog sim2sim", nullptr, nullptr);
    if (!window)
      throw std::runtime_error("GLFW window creation failed");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    camera.azimuth = 135.0;
    camera.elevation = -22.0;
    camera.distance = 2.2;
    camera.lookat[2] = 0.25;
    mjv_makeScene(model, &scene, 2000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);
  }

  void render() {
    if (!window || data->time + 1.0e-9 < next_render_s)
      return;
    glfwPollEvents();
    camera.lookat[0] = data->qpos[base_qpos_adr];
    camera.lookat[1] = data->qpos[base_qpos_adr + 1];
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    const mjrRect viewport{0, 0, width, height};
    mjv_updateScene(model, data, &option, nullptr, &camera, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &context);
    glfwSwapBuffers(window);
    next_render_s = data->time + 1.0 / 60.0;
  }
#endif

  mjModel *model{nullptr};
  mjData *data{nullptr};
  std::vector<std::string> joint_names;
  std::vector<int> qpos_adrs, dof_adrs, actuator_ids;
  int base_qpos_adr{-1}, base_body_id{-1};
#ifdef SH_DOG_WITH_GLFW
  GLFWwindow *window{nullptr};
  mjvCamera camera{};
  mjvOption option{};
  mjvScene scene{};
  mjrContext context{};
  double next_render_s{0.0};
#endif
};

MujocoBackend::MujocoBackend(const std::filesystem::path &scene_path,
                             const std::vector<std::string> &joint_names,
                             const std::vector<double> &initial_joint_positions,
                             bool viewer)
    : impl_(std::make_unique<Impl>(scene_path, joint_names,
                                   initial_joint_positions, viewer)) {}
MujocoBackend::~MujocoBackend() = default;
RobotState MujocoBackend::read() const { return impl_->read(); }
void MujocoBackend::write(const JointCommand &command) {
  impl_->write(command);
}
bool MujocoBackend::shouldExit() const {
#ifdef SH_DOG_WITH_GLFW
  return impl_->window && glfwWindowShouldClose(impl_->window);
#else
  return false;
#endif
}
double MujocoBackend::timestep() const { return impl_->model->opt.timestep; }
void MujocoBackend::checkModel(const std::filesystem::path &scene_path) {
  char error[2048]{};
  mjModel *model =
      mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
  if (!model)
    throw std::runtime_error("failed to load MuJoCo scene: " +
                             std::string(error));
  std::cout << "model_ok nq=" << model->nq << " nv=" << model->nv
            << " nu=" << model->nu << " mass=" << mj_getTotalmass(model)
            << "\n";
  if (model->nq != 19 || model->nv != 18 || model->nu != 12) {
    mj_deleteModel(model);
    throw std::runtime_error("unexpected ShDog MuJoCo dimensions");
  }
  mj_deleteModel(model);
}

} // namespace sh_dog
