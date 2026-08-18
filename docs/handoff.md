# 工程交接

## 当前状态

- raw、规范化 URDF、primitive collision 和本地生成 USD 流程已完成。
- 当前 raw 模型已更新为 `URDF-V4-test1`：关节拓扑和零位保持不变，CAD 总质量由约 `15.376 kg`
  更新为约 `16.526 kg`，并使用新版 visual meshes；规范化 URDF 和 USD 已重新生成。
- `model.toml` 是机器人名称、关节顺序、默认站姿、RS06 规格和 collision 的事实来源。
- 上层统一使用关节侧位置、速度、力矩和惯量；MCU 已处理协议与传动换算。
- `SH_DOG_CFG` 使用柔顺运动 PD：abad `25/1`、hip `30/1.2`、knee `40/2`。
- `SH_DOG_STAND_CFG` 使用已验证站起 PD：abad `40/1.5`、hip `60/2`、knee `80/3`。
- RS06 armature：abad/hip `0.012 kg·m²`，knee 最终关节侧 `0.048 kg·m²`。
- `SH_DOG_CFG` 已启用 `ShDogDcMotor`：按厂家最大/热平衡 T-N 曲线裁剪，使用跨扭矩共享的过载预算，
  episode 初始预算从 `[0, 1]` 随机化；原线性 `DCMotorCfg` 配置以注释保留用于楼梯对比。
- PhysX articulation solver 为 position `4`、velocity `2`。
- `stand.py` 已验证从下蹲姿态经 `0.5 s` 插值稳定站起；collision 可作为训练基线。
- 已建立 `ShDog-Velocity-Flat` 平地速度跟踪任务及最小 RSL-RL PPO 配置。
- `ShDog-Velocity-Flat-Play` 当前固定 `ang_vel_z=1.5 rad/s`、关闭 observation corruption，用于验收
  reset、接触、观测和 action；任务只使用 `SH_DOG_CFG`。
- actor 为单帧 45D MLP，不含实机难以稳定获取的 base linear velocity；critic 使用 60D 仿真特权
  observation。PPO 网络和超参数对齐 Unitree Go2 velocity 配置。
- 平地任务直接继承 `ManagerBasedRLEnvCfg`；scene、MDP 配置、奖励和仿真参数集中在
  `sh_dog_flat_env_cfg.py`，不继承 rough task 配置；少量自定义计算按职责位于 `mdp/`。
- 已新增独立 `ShDog-Velocity-Rough` 任务，继承 flat 的命令、动作、奖励、随机化和 termination，
  仅替换官方 rough terrain generator，并为 critic 增加高度扫描。rough 使用独立的 `sh_dog_rough`
  实验目录和 recurrent PPO policy，flat 仍保持原单帧 MLP。
- rough 地形使用官方比例和几何，降低为 stairs `0.05–0.16 m`、boxes `0.05–0.10 m`、random rough
  `0.02–0.05 m`/`0.01 m` step、slopes `0–0.25`。启用官方基于前进距离的地形课程，初始覆盖
  level `0–5`，成功后动态升到更高等级。
- rough 当前从 10k recurrent student 做 heading fine-tune：保持 `vx=0.5–1.0 m/s`、`vy=0`，启用
  官方航向 P 控制器，目标航向覆盖 `[-π, π]`，动态 `wz` 限幅为 `±0.5 rad/s`；关闭 standing
  command 和外部 push，线速度及 yaw rate 跟踪 `std` 均为 `0.4`。actor 仍使用 45D 本体观测和
  单层 128D GRU，不含高度图；recurrent critic 保留干净高度扫描。命令范围课程保持关闭，velocity
  command 的 debug visualization 已全局关闭，训练和 Play 均不加载远端箭头 USD。
- recurrent student 已通过 `64 environments × 2 iterations` Docker 冒烟，确认 actor/critic GRU、
  非对称 observation、hidden state 训练和源码状态记录正常；Event Manager 中无 interval push。
- `sh_dog_baseline` 对齐 Unitree 开源 Go2 velocity 当前启用的速度课程、随机化、奖励、termination
  和 PPO；仅 flat terrain，不引入其注释掉的楼梯、height scan 或 observation history。
- 仓库级可执行工具统一位于 `scripts/`；`sync_training.sh` 可将源码和本地生成 USD 严格镜像到训练
  服务器，同时保护远端 `.git/` 与 `artifacts/`。
- 已完成 `baseline_10k` 的 4096 environments、10000 iterations 训练；最终 checkpoint 为
  `model_9999.pt`。本机 Play 已观察到策略可以全向移动；TensorBoard 日志审计以及 nominal、
  randomized 定量评估均已完成。
- `training/scripts/rsl_rl/eval.py` 从 task 生成可重放评估协议，输出逐 episode 指标和汇总结果；
  `scripts/training.sh eval` 提供 Docker 入口。rough 协议固定覆盖 6 类地形、等级 `0/3/6/9` 和
  `0.5/1.0 m/s` 前进命令，并记录前向进度、横向/yaw 漂移及原有力矩诊断。
- 已建立第一版 MuJoCo sim2sim：`scripts/build_mujoco.py` 从规范化 URDF 和 `model.toml` 生成浮动基座
  MJCF；`scripts/package_policy.py` 将已有 ONNX 封装为带 manifest/checksum 的正式策略包；
  `deploy/runtime/` 提供后端无关的 C++ observation、GRU ONNX 推理和 action/PD 处理，
  `deploy/sim2sim/` 只负责 MuJoCo 状态与关节力矩。
- 当前生成的 MuJoCo 模型为 `nq=19`、`nv=18`、`nu=12`，总质量 `16.5259735 kg`。启动阶段使用
  `SH_DOG_STAND_CFG` 对应的站立 PD，进入策略后切换为 `SH_DOG_CFG` 运动 PD。
- 首次闭环发现 `mj_objectVelocity(..., local=1)` 将角速度表达在 MuJoCo 惯性主轴而非 URDF
  `base_link`，导致 45D observation 的前三维发生轴交换和符号错误；已改为世界角速度经 link
  quaternion 逆旋转，与 Isaac Lab `root_ang_vel_b` 对齐。指定 frozen recurrent student 10k 在修正后
  通过 `vx=0/0.5/1.0 m/s` 各 20 秒 headless 验证，均未倒地；最终 base 高度为
  `0.375/0.390/0.392 m`。第一版仍使用安全系数后的静态峰值力矩限幅，尚未复现 `ShDogDcMotor`
  的完整 T-N、热平衡和过载预算。
- 两套评估均为 21 cases、每 case 8 repeats，共 168 episodes，结果均为 168/168 成功且无 termination。
  randomized 相比 nominal 主要增加 yaw 跟踪和停止恢复误差，但未出现稳定性失效。
- 逐关节力矩诊断显示 flat nominal/randomized 均没有长期饱和：最坏连续裁剪分别为 `0.14 s` 和
  `0.24 s`。限幅主要出现在 hip 和后腿 knee 的短时瞬态，当前 flat 结果不受持续力矩不足支配。
- `v1.0_10k` 的 flat 额定力矩超限主要发生在低于 `100 rpm` 的 hip 和后腿 knee；按允许时间乘
  `0.9` 后的堵转表估计，12 s randomized episode 最坏过载预算消耗约 `1.63%`。该结果支持继续用
  flat 做行为回归，但不能替代楼梯、斜坡和碎石等持续冲击工况验证。
- rough `model_3000.pt` 已完成 192 episode nominal 固定地形评估：存活率 `93.2%`，前进至少 `3 m`
  的穿越率 `85.4%`。正反斜坡与 random rough 全等级均为 `100%` 穿越；困难集中在 level 6/9
  楼梯，level 9 上楼在 `0.5/1.0 m/s` 下穿越率均为 `0%`。
- rough 楼梯失败 episode 的执行器裁剪明显更多，但仍不能单独归因于执行器：全局最坏 computed/
  applied torque 分别为 knee `90.7/64.8 N·m`，最长 applied 额定超限为 hip `1.46 s`，保守堵转
  预算最大消耗 `28.7%`，未见预算耗尽。后续需用相同训练和评估协议对比线性 `DCMotorCfg`。
- `2026-08-17_11-16-50_forward_curriculum_3k/model_2999.pt` 使用干净高度图教师、前进命令和动态
  地形课程；课程平均等级在约 1000 iterations 达到 `6.26`，末段稳定约 `6.0`。固定地形评估为
  `191/192` 存活且完成穿越，level `0/3/6/9` 的上下楼梯均为 `100%`，唯一失败为 level 9 boxes 的
  一次 bad orientation。
- 该教师相对旧 10k rough 策略将穿越率从 `83.3%` 提高到 `99.5%`，但平均横向漂移从 `0.184 m`
  增至 `0.302 m`，平均 yaw 漂移从 `0.076 rad` 增至 `0.094 rad`。level 9 上楼 `1.0 m/s` 的平均
  横向/yaw 漂移为 `1.16 m/0.174 rad`，后续需区分航向闭环与崎岖地形侧向落脚偏差。
- 教师固定评估中 `ShDogDcMotor` 最长连续裁剪为 `0.42 s`，最大 applied 额定超限连续时间为
  `0.14 s`，最坏保守堵转预算消耗为 `5.17%`；level 9 上楼存在明显降额但仍全部通过，未见持续
  峰值力矩或预算耗尽主导行为。
- recurrent student 从 `model_2999.pt` 原配置续训至 `model_9999.pt` 后基本收敛：末 2000 iterations
  平均 episode length 约 `990/1000 steps`、timeout 约 `98.1%`、平均 terrain level 稳定约 `6.01`。
  固定 rough 评估为 `191/192` 存活且完成穿越，level `0/3/6/9` 上下楼均达到 `100%`；唯一失败为
  level 6、`1.0 m/s` 下楼接近 episode 末段的 bad orientation，首次碰阶恢复问题已经解决。
- 10k student 以直线性换取了上楼存活率：固定评估平均横向/yaw 漂移为 `0.479 m/0.158 rad`；
  level 9、`1.0 m/s` 上楼虽 `4/4` 穿越，但平均横向/yaw 漂移达到 `3.32 m/0.712 rad`。高速上楼
  所有 repeat 都存在约 `-0.04` 至 `-0.11 rad/s` 的同向 yaw rate bias，不是随机失稳或过载耗尽。
- 评估入口已支持可重放的 `--heading-hold`，使用 Isaac Lab 官方 heading P 控制器保持命令开始时
  航向，并记录动态 `wz`、signed drift 和最大 heading error。冻结 10k student 在 `kp=1.0`、yaw
  rate 限幅 `0.5/0.2 rad/s` 下均为 `192/192` 穿越，但平均横漂分别为 `0.501/0.491 m`，平均 yaw
  仅降至 `0.137/0.142 rad`；level 9 高速上楼仍为 `3.62 m/0.645 rad` 和 `3.45 m/0.677 rad`。
  策略没有训练过非零 `wz`，无法直接用外部 heading loop 修复，需要从 10k checkpoint fine-tune。
- heading fine-tune 已从 `model_9999.pt` 完成 `64 environments × 2 iterations` Docker 冒烟；保存的
  `env.yaml` 确认为 `heading_command=true`、全航向目标、`wz=±0.5 rad/s` 和两项跟踪 `std=0.4`。
  task 默认闭环后，heading-hold 固定协议的 `48 episodes × 1 repeat` 回归仍为 `48/48` 存活和穿越，
  平均横向/yaw 漂移 `0.499 m/0.135 rad`，与修改前冻结策略诊断一致。
- 本次 run 未保存 ShDog 源码 commit，只能确认训练参数、checkpoint 和 TensorBoard 记录；后续训练
  需要补齐 ShDog commit、dirty 状态、完整命令和镜像摘要。
- Docker 评估首次启动曾在 `omni.platforminfo.plugin` 内偶发 segfault；相同配置随后完成最小启动及
  两套完整评估，GPU 和 CUDA 注入正常。当前不绕开 Docker，也不因单次异常增加自动重试。

## 下一步

干净高度图教师暂不重训，10k recurrent student 固定为盲走反射基线，不再扩大 GRU 或按原配置续训。
下一步从 `2026-08-17_18-03-30_recurrent_student_10k/model_9999.pt` 续训 heading fine-tune 10k：保持
GRU128、rough 地形、执行器、前进范围、PPO、噪声和地形课程不变，采用 `±0.5 rad/s` yaw authority
及 `track_ang_vel_z std=0.4`。至少保留 1k/2k/5k/10k checkpoint，并使用同一 heading-hold 固定
协议检查楼梯穿越率、横向/yaw 漂移、`wz` 跟踪和执行器过载；若前期出现灾难性遗忘，不应等到 10k
才判断。若 Docker 启动崩溃复现，再保留完整 Kit 日志和 crash dump 分析。

部署侧与后台训练并行推进但不读取当前 checkpoint：待 heading fine-tune 选定正式 checkpoint 后，在
训练 Docker 内生成 ONNX，再用 `scripts/package_policy.py` 生成版本化策略包，执行 20 秒 viewer
sim2sim 验收。随后补齐 MuJoCo 侧 RS06 T-N/过载模型并做 Isaac Lab/MuJoCo 同输入定量对齐；通过前
不接 sim2real。

## 检查

```bash
python scripts/normalize_urdf.py
python scripts/build_usd.py
python scripts/build_mujoco.py --check
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure
scripts/training.sh build-usd
python training/scripts/list_envs.py
python training/scripts/stand.py
python training/scripts/zero_agent.py --task ShDog-Velocity-Flat-Play --num_envs 1
scripts/training.sh train rough_smoke \
  --task ShDog-Velocity-Rough --num-envs 64 --max-iterations 2
scripts/training.sh train recurrent_student_3k \
  --task ShDog-Velocity-Rough --num-envs 4096 --max-iterations 3000 --shm-size 8gb
scripts/training.sh eval \
  artifacts/training/logs/rsl_rl/sh_dog_baseline/<run>/model_9999.pt
scripts/training.sh eval \
  artifacts/training/logs/rsl_rl/sh_dog_baseline/<run>/model_9999.pt \
  --task ShDog-Velocity-Flat
scripts/training.sh eval \
  artifacts/training/logs/rsl_rl/sh_dog_rough/<run>/model_2999.pt \
  --task ShDog-Velocity-Rough-Play --shm-size 8gb
```

recurrent student 重点比较固定地形总穿越率、level `0/3/6/9` 上下楼成功率、横向/yaw 漂移和逐关节
computed/applied torque。当前高度图教师的 `99.5%` 穿越率是仿真上限参考，不要求第一版盲走学生
立即追平；首先确认其明显优于停在台阶前或首次接触后无法恢复的行为。

USD 与训练产物不提交 Git。模型约定见 `docs/robot_model.md`，工程边界见 `docs/architecture.md`。
