# 机器人模型约定

本文定义 `sh_dog` 在训练、sim2sim 和 sim2real 中共用的模型语义。当前原始 CAD 导出文件保存在
`assets/sh_dog/raw/`，保持原始内容不直接修改；历史版本由 Git 保留。

规范化模型位于 `assets/sh_dog/urdf/sh_dog.urdf`，由 `scripts/normalize_urdf.py` 生成并提交 Git。
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

实机稳定站姿按上述顺序为每条腿 `(0.0, 0.872, -1.512) rad`，名义 base 高度为 `0.40 m`。
这些值保存在 `assets/sh_dog/model.toml`，训练配置引用它们，不另行手工维护。

实机协议和仿真后端可以使用不同的内部顺序，但必须通过名称显式映射到该顺序。进入策略运行时的
数据必须已经符合该关节顺序和关节侧约定。

## 规范化边界

规范化过程只允许执行确定性的结构修正：

- 将当前 CAD 导出的机器人名 `URDF-V4-test1` 改为 `sh_dog`；
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
空载转速为 `480 rpm`，低速端等效惯量为 `0.012 kg·m²`。knee 关节在电机模组后还有 `2:1`
减速器。

因此规范化 URDF 使用以下关节侧峰值边界：

| Joint type | Effort | Velocity |
| --- | ---: | ---: |
| `abad` | `36 N·m` | `50.2655 rad/s` |
| `hip` | `36 N·m` | `50.2655 rad/s` |
| `knee` | `72 N·m` | `25.1327 rad/s` |

训练侧 armature 使用最终关节侧等效惯量：`abad/hip` 为 `0.012 kg·m²`，`knee` 为
`0.012 × 2² = 0.048 kg·m²`。PhysX 将其与 URDF link 惯量共同用于关节动力学，不写回 link inertia。

额定力矩、峰值力矩和训练时允许的命令限幅是不同概念。上述数值描述硬件峰值边界；正式训练配置
可以采用更保守的 effort limit，sim2real 安全限制也必须独立验收。

RS06 厂家 T-N 和过载表同样保存在 `model.toml`，数值均为电机模组输出侧原始事实。48 V 最大
T-N 锚点为 `(280 rpm, 36 N·m)`、`(320, 30)`、`(390, 20)`、`(426, 11)`、`(430, 5)`；
热平衡锚点为 `(100 rpm, 11.2 N·m)`、`(160, 11)`、`(220, 10.5)`、`(280, 9.87)`、
`(340, 9.4)`、`(400, 9.3)`。两条曲线均基于厂家使用的 `200 mm × 200 mm` 铝散热板。

旋转过载表使用 `17/200 s`、`20/36 s`、`25/18 s`、`30/8 s`、`36/4 s`；堵转工况连续边界为
`8 N·m`，并使用 `11/200 s`、`17/15 s`、`25/5 s`、`30/1 s`、`36/1 s`。`ShDogDcMotor`
对表格扭矩和允许时间分别应用 `0.9` 安全系数，原始锚点不改写。

`ShDogDcMotor` 从 `IdealPDActuator` 继承 PD 计算，不使用 `DCMotor` 的线性力矩—速度裁剪。它按绝对
转速插值最大 T-N 瞬时边界；低速持续边界从堵转 `8 N·m` 线性过渡到 `100 rpm` 热平衡锚点。每个
关节维护一个共享过载预算，`0 rpm` 使用堵转消耗率，至 `100 rpm` 线性过渡为旋转消耗率，因此切换
扭矩不会刷新计时。预算耗尽后输出降额至热平衡 T-N 边界，低于该边界时按 `200 s` 空载完全恢复，
预算恢复到 `0.2` 后解除降额。每个训练 episode reset
时预算独立从 `[0, 1]` 均匀采样；PPO iteration 不重置预算。该单状态模型是缺少温度和冷却实测数据
时的保守近似，不作为实机热保护的替代。状态机站起使用的 `SH_DOG_STAND_CFG` 固定从满预算开始，
不引入训练随机化。

站起状态机与运动策略使用不同 PD：站起采用 abad `40/1.5`、hip `60/2`、knee `80/3`；运动采用
abad `25/1`、hip `30/1.2`、knee `40/2`。策略训练、sim2sim 和 sim2real 必须使用相同的运动 PD。

原始规格与额外传动比只在 `assets/sh_dog/model.toml` 中编辑，资产生成工具据此计算关节侧数值并
写入规范化 URDF，不在训练配置和部署配置中重复手工维护。

## 碰撞体状态

正式 collision 由 `assets/sh_dog/model.toml` 定义，生成器按标准腿名前缀完成前后、左右镜像。
每个 link 使用一个或多个 primitive；复合碰撞体用于覆盖外壳和完整腿段，同时保持相邻关节附近的
必要间隙：

| Link | Shape | 设计边界 |
| --- | --- | --- |
| `base_link` | 3 × box | 覆盖中央底盘和前后外壳，不包含两侧关节扫掠空间 |
| `*_abad_link` | 2 × cylinder | 覆盖电机主体和靠近机身的关节壳体 |
| `*_hip_link` | cylinder + box | 覆盖髋关节壳体和完整大腿主体 |
| `*_knee_link` | 2 × cylinder | 分段覆盖带倾角的小腿上下段 |
| `*_foot_link` | sphere | 匹配足端外形和接触点 |

该拆分参考 [Unitree Go2 官方 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf)
的复合 primitive 和避免相邻 link 假碰撞经验，尺寸和 origin 均来自
ShDog STL 与关节坐标，不复用 Go2 数值。visual 继续使用 STL；质量、惯量、拓扑、关节轴和限位
不受 collision 生成影响。

当前已通过 visual/collision 局部叠加、代表性站立与下蹲姿态自碰撞检查，以及 Isaac Sim 5.1 URDF
导入。接近关节极限的姿态仍可能发生真实的相邻腿段接触；正式训练前应结合默认姿态和启用的
self-collision 策略完成落地仿真验收。

## 后端资产

Isaac Sim USD 固定生成到 `assets/sh_dog/usd/`，由 `scripts/build_usd.py` 转换，不提交 Git。转换保留
fixed joint，不写入任意 PD 参数；训练侧负责定义驱动器。USD 仅是 Isaac Sim 后端资产，不是模型
事实来源。
