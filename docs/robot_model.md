# 机器人模型约定

本文定义 `sh_dog` 在训练、sim2sim 和 sim2real 中共用的模型语义。原始 CAD 导出文件保存在
`assets/sh_dog/raw/`，永久保持不变。

规范化模型位于 `assets/sh_dog/urdf/sh_dog.urdf`，由 `tools/normalize_urdf.py` 生成并提交 Git。
它直接引用 raw STL，不维护 mesh 副本。任何模型变更都先修改 raw 输入或 `model.toml`，再重新生成。

## 坐标系

模型采用右手坐标系：

- `x`：机器人前方；
- `y`：机器人左方；
- `z`：机器人上方。

关节正方向由规范化 URDF 的 `axis` 定义。训练和部署入口都使用关节侧的位置、速度和力矩。MCU、
电机驱动与 IMU 数据读取层已经完成方向、单位和传动比处理，策略运行时不得再次缩放或翻转。

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

实机协议和仿真后端可以使用不同的内部顺序，但必须通过名称显式映射到该顺序。进入策略运行时的
数据必须已经符合该关节顺序和关节侧约定。

## 规范化边界

规范化过程只允许执行确定性的结构修正：

- 将机器人名 `urdf-v2-2` 改为 `sh_dog`；
- 将误拼写 `fl_foot_joinf` 改为 `fl_foot_joint`；
- 将 mesh URI 改为指向唯一 raw STL 的相对路径；
- 统一 XML 格式，不改变数值精度表达的物理含义。

在没有明确依据时，不得修改：

- link 和其余 joint 名称；
- link 拓扑、joint 类型、原点和轴向；
- 质量、质心和惯量；
- 关节位置限位；
- visual 和 collision 几何。

## 执行器边界

所有关节使用 `ROBOstride 06`。规格表给出的输出侧额定力矩为 `11 N·m`、峰值力矩为 `36 N·m`、
空载转速为 `480 rpm`。knee 关节在电机模组后还有 `2:1` 减速器。

因此规范化 URDF 使用以下关节侧峰值边界：

| Joint type | Effort | Velocity |
| --- | ---: | ---: |
| `abad` | `36 N·m` | `50.2655 rad/s` |
| `hip` | `36 N·m` | `50.2655 rad/s` |
| `knee` | `72 N·m` | `25.1327 rad/s` |

额定力矩、峰值力矩和训练时允许的命令限幅是不同概念。上述数值描述硬件峰值边界；正式训练配置
可以采用更保守的 effort limit，sim2real 安全限制也必须独立验收。

原始规格与额外传动比只在 `assets/sh_dog/model.toml` 中编辑，资产生成工具据此计算关节侧数值并
写入规范化 URDF，不在训练配置和部署配置中重复手工维护。

## 碰撞体状态

当前规范化 URDF 仍使用 CAD mesh collision，只适合结构验证。下一步根据 ShDog 实际尺寸生成最少
数量的 box、cylinder 和 sphere，并通过可视叠加、全关节范围自碰撞和 Isaac Lab 落地测试验收。
Go2 等成熟模型只用于参考 primitive 选择和拆分方式，不复用尺寸。
