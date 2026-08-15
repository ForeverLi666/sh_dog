# 工程交接

## 当前状态

- raw、规范化 URDF、primitive collision 和本地生成 USD 流程已完成。
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
  仅替换官方 rough terrain generator、增加 terrain curriculum 与 actor/critic 高度扫描。rough PPO
  参数保持不变，仅使用独立的 `sh_dog_rough` 实验目录。
- rough 地形使用官方比例和几何，降低为 stairs `0.05–0.16 m`、boxes `0.05–0.10 m`、random rough
  `0.02–0.05 m`/`0.01 m` step、slopes `0–0.25`；初始最高 terrain level 为 `2`，课程仍覆盖全部
  `10` 个等级。
- `sh_dog_baseline` 对齐 Unitree 开源 Go2 velocity 当前启用的速度课程、随机化、奖励、termination
  和 PPO；仅 flat terrain，不引入其注释掉的楼梯、height scan 或 observation history。
- 仓库级可执行工具统一位于 `scripts/`；`sync_training.sh` 可将源码和本地生成 USD 严格镜像到训练
  服务器，同时保护远端 `.git/` 与 `artifacts/`。
- 已完成 `baseline_10k` 的 4096 environments、10000 iterations 训练；最终 checkpoint 为
  `model_9999.pt`。本机 Play 已观察到策略可以全向移动；TensorBoard 日志审计以及 nominal、
  randomized 定量评估均已完成。
- `training/scripts/rsl_rl/eval.py` 从 task 命令包络生成 flat 归一化评估协议，输出逐 episode 指标和
  汇总结果；`scripts/training.sh eval` 提供 Docker 入口。当前不预设 rough terrain 评估框架。
- 两套评估均为 21 cases、每 case 8 repeats，共 168 episodes，结果均为 168/168 成功且无 termination。
  randomized 相比 nominal 主要增加 yaw 跟踪和停止恢复误差，但未出现稳定性失效。
- 逐关节力矩诊断显示 flat nominal/randomized 均没有长期饱和：最坏连续裁剪分别为 `0.14 s` 和
  `0.24 s`。限幅主要出现在 hip 和后腿 knee 的短时瞬态，当前 flat 结果不受持续力矩不足支配。
- `v1.0_10k` 的 flat 额定力矩超限主要发生在低于 `100 rpm` 的 hip 和后腿 knee；按允许时间乘
  `0.9` 后的堵转表估计，12 s randomized episode 最坏过载预算消耗约 `1.63%`。该结果支持继续用
  flat 做行为回归，但不能替代楼梯、斜坡和碎石等持续冲击工况验证。
- 本次 run 未保存 ShDog 源码 commit，只能确认训练参数、checkpoint 和 TensorBoard 记录；后续训练
  需要补齐 ShDog commit、dirty 状态、完整命令和镜像摘要。
- Docker 评估首次启动曾在 `omni.platforminfo.plugin` 内偶发 segfault；相同配置随后完成最小启动及
  两套完整评估，GPU 和 CUDA 注入正常。当前不绕开 Docker，也不因单次异常增加自动重试。

## 下一步

当前证据支持接受 baseline 的平地能力，不修改奖励、噪声或 PPO。下一步先完成 rough 冒烟，再使用
完全相同的 rough 配置分别训练注释保留的线性 `DCMotorCfg` 和 `ShDogDcMotor`，比较 computed/applied
torque、额定超限、预算消耗和地形通过率。单 seed 只能用于初筛，确认差异后再使用多个 seed 重复。
若 Docker 启动崩溃复现，再保留完整 Kit 日志和 crash dump 分析。

## 检查

```bash
python scripts/normalize_urdf.py
python scripts/build_usd.py
python training/scripts/list_envs.py
python training/scripts/stand.py
python training/scripts/zero_agent.py --task ShDog-Velocity-Flat-Play --num_envs 1
scripts/training.sh train rough_smoke \
  --task ShDog-Velocity-Rough --num-envs 64 --max-iterations 2
scripts/training.sh eval \
  artifacts/training/logs/rsl_rl/sh_dog_baseline/<run>/model_9999.pt
scripts/training.sh eval \
  artifacts/training/logs/rsl_rl/sh_dog_baseline/<run>/model_9999.pt \
  --task ShDog-Velocity-Flat
```

USD 与训练产物不提交 Git。模型约定见 `docs/robot_model.md`，工程边界见 `docs/architecture.md`。
