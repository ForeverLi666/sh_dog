# 工程交接

## 当前状态

- raw、规范化 URDF、primitive collision 和本地生成 USD 流程已完成。
- `model.toml` 是机器人名称、关节顺序、默认站姿、RS06 规格和 collision 的事实来源。
- 上层统一使用关节侧位置、速度、力矩和惯量；MCU 已处理协议与传动换算。
- `SH_DOG_CFG` 使用柔顺运动 PD：abad `25/1`、hip `30/1.2`、knee `40/2`。
- `SH_DOG_STAND_CFG` 使用已验证站起 PD：abad `40/1.5`、hip `60/2`、knee `80/3`。
- RS06 armature：abad/hip `0.012 kg·m²`，knee 最终关节侧 `0.048 kg·m²`。
- PhysX articulation solver 为 position `4`、velocity `2`。
- `stand.py` 已验证从下蹲姿态经 `0.5 s` 插值稳定站起；collision 可作为训练基线。
- 已建立 `ShDog-Velocity-Flat-v0` 平地速度跟踪任务及最小 RSL-RL PPO 配置。
- `ShDog-Velocity-Flat-Play-v0` 固定零命令、关闭 observation corruption，用于先验收站立、reset、
  接触、观测和 action；任务只使用 `SH_DOG_CFG`。
- actor 为单帧 45D MLP，不含实机难以稳定获取的 base linear velocity；critic 使用 60D 仿真特权
  observation。PPO 网络和超参数对齐 Unitree Go2 velocity 配置。
- 平地任务直接继承 `ManagerBasedRLEnvCfg`；scene、MDP 配置、奖励和仿真参数集中在
  `sh_dog_env_cfg.py`，不继承 rough task 配置；少量自定义计算按职责位于 `mdp/`。
- `sh_dog_baseline` 对齐 Unitree 开源 Go2 velocity 当前启用的速度课程、随机化、奖励、termination
  和 PPO；仅 flat terrain，不引入其注释掉的楼梯、height scan 或 observation history。
- 仓库级可执行工具统一位于 `scripts/`；`sync_training.sh` 可将源码和本地生成 USD 严格镜像到训练
  服务器，同时保护远端 `.git/` 与 `artifacts/`。

## 下一步

先用 Play 环境验收零命令站立、reset、足端接触和零 action。通过后再运行 64 环境、2 iteration
的最小 PPO 冒烟；在这些基础行为确认前不扩大训练规模。

## 检查

```bash
python scripts/normalize_urdf.py
python scripts/build_usd.py
python training/scripts/list_envs.py
python training/scripts/stand.py
python training/scripts/zero_agent.py --task ShDog-Velocity-Flat-Play-v0 --num_envs 1
python training/scripts/rsl_rl/train.py \
  --task ShDog-Velocity-Flat-v0 --headless --num_envs 64 --max_iterations 2
```

USD 与训练产物不提交 Git。模型约定见 `docs/robot_model.md`，工程边界见 `docs/architecture.md`。
