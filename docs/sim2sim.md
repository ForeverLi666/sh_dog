# MuJoCo sim2sim

第一版 sim2sim 用于验证正式策略包能否通过共享 C++ runtime 驱动 ShDog MuJoCo 模型。它不读取
Isaac Lab checkpoint，也不导入 `training/`；未来 sim2real 复用 `deploy/runtime/`，只替换
`MujocoBackend`。

## 策略包契约

策略包由训练环境导出的 ONNX 和同次生成的 manifest 组成：

```text
policy_bundle/
├── policy.onnx
├── manifest.yaml
└── checksum.sha256
```

`manifest.yaml` 明确声明 ONNX tensor、GRU state、关节顺序、45D observation 的逐项顺序、裁剪、
缩放、历史长度和部署时 corruption 状态，以及 action 后处理、默认关节位置、站立/运动两套 PD、控制周期和
坐标约定。当前训练没有 action clip，因此 manifest 显式写为 `null`，runtime 不自行增加裁剪。
runtime 启动时校验 checksum、字段、维度和实际 ONNX tensor；不一致时直接失败。

现有 RSL-RL 导出文件可打包为正式部署边界。参数必须由对应训练配置明确提供，打包工具不从 ONNX
猜测 recurrent 类型或 hidden size：

```bash
python scripts/package_policy.py \
  artifacts/training/logs/rsl_rl/sh_dog_rough/<run>/exported/policy.onnx \
  artifacts/policies/<policy-name> \
  --recurrent gru --hidden-size 128
```

MLP 策略使用 `--recurrent none`。输出目录必须为空，工具不会覆盖已有策略包。

## MuJoCo 模型

`assets/sh_dog/mujoco/` 由规范化 URDF 和 `assets/sh_dog/model.toml` 确定性生成，不手工编辑：

```bash
python scripts/build_mujoco.py
python scripts/build_mujoco.py --check
```

模型保留当前 CAD 的质量、惯量、visual、primitive collision、关节轴与限位；增加浮动基座、
`model.toml` 中的关节侧 armature 和 12 个力矩执行器。机器人自身 collision 被隔离，只与地面碰撞，
对应训练侧关闭 self-collision。

## 构建与运行

依赖 MuJoCo、ONNX Runtime C++、yaml-cpp、OpenSSL；GLFW 可选：

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure
```

无界面运行 20 秒，固定命令为 `(vx, vy, wz)`：

```bash
./build/sh_dog_sim2sim \
  artifacts/policies/<policy-name> \
  assets/sh_dog/mujoco/scene.xml \
  20 0.5 0.0 0.0
```

在末尾增加 `--viewer` 打开可视化。程序先用站立 PD `abad 40/1.5、hip 60/2、knee 80/3` 稳定
1 秒，再切换为策略训练使用的运动 PD `abad 25/1、hip 30/1.2、knee 40/2` 进入闭环；状态与命令
均按关节名映射，不依赖 MuJoCo 内部数组顺序。

MuJoCo `mj_objectVelocity(..., flg_local=1)` 对 body 使用局部惯性主轴，不等同于 Isaac Lab 的 root
link frame。后端因此读取世界角速度，再用浮动基座的 link quaternion 逆旋转，构造与
`root_ang_vel_b` 相同的输入；禁止直接把 MuJoCo local body velocity 填入 observation。

## 第一版限制

- 当前是平地和固定速度命令，不包含 rough terrain、键盘遥控或 heading hold。
- MuJoCo 执行器使用 `model.toml` 峰值力矩乘安全系数后的静态关节侧限幅，尚未复现训练侧完整
  T-N 曲线、热平衡曲线和共享过载预算。因此可验证策略契约、时序和基本闭环行为，不能替代执行器
  降额一致性验收。
- 当前安全检查覆盖缺失/非有限状态、无效四元数、ONNX 非有限输出和过低 base；实机所需状态超时、
  姿态、机械限位、通信错误、阻尼与释放流程将在 sim2real 前扩展到共享 runtime。
