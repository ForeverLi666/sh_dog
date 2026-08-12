# 机器人模型约定

本文定义 `sh_dog` 在训练、sim2sim 和 sim2real 中共用的模型语义。原始 CAD 导出文件保存在
`assets/sh_dog/raw/`，永久保持不变。

## 坐标系

模型采用右手坐标系：

- `x`：机器人前方；
- `y`：机器人左方；
- `z`：机器人上方。

关节正方向由规范化 URDF 的 `axis` 定义。部署后端负责将电机编码器方向映射到模型方向，不得在
observation 或 action 处理中隐式翻转符号。

## 名称

机器人标准名称为 `sh_dog`。腿部前缀固定为：

| Prefix | Meaning |
| --- | --- |
| `fl` | front left |
| `fr` | front right |
| `rl` | rear left |
| `rr` | rear right |

每条腿保留原模型简洁且一致的 `abad`、`hip`、`knee` 命名。策略关节顺序固定为：

```text
fl_abad_joint
fl_hip_joint
fl_knee_joint
fr_abad_joint
fr_hip_joint
fr_knee_joint
rl_abad_joint
rl_hip_joint
rl_knee_joint
rr_abad_joint
rr_hip_joint
rr_knee_joint
```

实机协议和仿真后端可以使用不同的内部顺序，但必须通过名称显式映射到该顺序。

## 规范化边界

规范化过程只允许执行确定性的结构修正：

- 将机器人名 `urdf-v2-2` 改为 `sh_dog`；
- 将误拼写 `fl_foot_joinf` 改为 `fl_foot_joint`；
- 将 mesh 文件扩展名统一为小写 `.stl`；
- 将 mesh URI 改为规范化资产包内的相对路径；
- 统一 XML 格式，不改变数值精度表达的物理含义。

在没有明确依据时，不得修改：

- link 和其余 joint 名称；
- link 拓扑、joint 类型、原点和轴向；
- 质量、质心和惯量；
- 关节位置限位；
- visual 和 collision 几何。

## 待确认参数

原始 URDF 中 12 个驱动关节的 `effort` 和 `velocity` 均为 `0`，不能作为有效模型参数。建立正式
Isaac Lab 资产前，必须根据电机、减速器和控制侧约束确定：

- 最大持续或允许关节力矩，单位 `N·m`；
- 最大允许关节速度，单位 `rad/s`；
- 各关节是否使用相同约束。

这些值只保留一个可编辑来源，并由资产生成工具写入规范化 URDF；不得在 URDF、训练配置和部署
配置中分别手工维护。
