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
- `ShDog-Template-v0` 仍是 Cartpole 链路测试，正式四足任务尚未建立。

## 下一步

参考 Isaac Lab Go2 rough locomotion，建立 ShDog 平地速度跟踪任务和最小 PPO 配置。初始 action scale
按 abad/hip/knee 使用 `0.25/0.20/0.15 rad`；先验证零命令站立、reset、接触、观测和 action，再训练。

## 检查

```bash
python tools/normalize_urdf.py --check
python tools/build_usd.py
python training/scripts/list_envs.py
python training/scripts/stand.py
```

USD 与训练产物不提交 Git。模型约定见 `docs/robot_model.md`，工程边界见 `docs/architecture.md`。
