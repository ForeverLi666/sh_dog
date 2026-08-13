# 工程架构

## 目标

`sh_dog` 是训练、策略导出、sim2sim 和 sim2real 的单仓库工程。仓库统一机器人接口与策略契约，
各运行环境保持构建和依赖隔离。

架构遵循以下原则：

- 根目录代表完整机器人软件工程，不代表某个仿真或训练框架。
- Isaac Lab 只属于训练子工程。
- sim2sim 和 sim2real 共享同一套 C++ 策略运行时。
- checkpoint 不跨训练环境传递，跨环境只传版本化策略包。
- 配置必须被代码、构建或测试消费；纯说明信息属于文档。
- 不为尚未出现的实现设计通用框架。

## 目标结构

```text
sh_dog/
├── assets/                 # 原始资产与提交 Git 的规范化共享模型
├── config/                 # 跨模块共享的机器可读接口配置
├── training/               # Isaac Lab 训练子工程
│   ├── source/sh_dog/
│   └── scripts/
├── deploy/                 # 独立于 Isaac Lab 的 C++ 部署域
│   ├── runtime/            # 共享策略、控制与安全逻辑
│   ├── sim2sim/            # MuJoCo 后端与入口
│   ├── sim2real/           # 实机后端与入口
│   └── interfaces/         # 实机通信接口定义
├── scripts/                # 仓库级生成、同步和部署入口
├── docker/                 # 可移植训练环境
├── docs/                   # 架构、接口与交接说明
├── artifacts/              # 不提交的运行产物与策略包
└── README.md
```

目录按实际需求逐步创建。Isaac Lab 骨架已迁移到 `training/`，任务逻辑保持不变。

## 模块边界

### Repository scripts

根目录 `scripts/` 保存仓库级可执行入口，包括资产生成、训练服务器同步和后续策略打包或部署操作。
`training/scripts/` 只保存 Isaac Lab 专属入口，不承担跨模块操作。

训练服务器同步可以严格镜像工程源码和本地生成的 USD，但必须保护远端 Git 元数据与 `artifacts/`。
后续实机同步采用部署文件白名单，只允许 `deploy/`、部署配置和策略包进入实机，不同步 `training/`、
Docker、USD 或 checkpoint。

### Training

`training/` 包含 Isaac Lab task、环境配置、奖励、训练、播放和导出入口。它可以读取共享资产与
配置，但部署代码不得导入训练包。

正式训练、定量诊断、checkpoint 和策略导出在 Docker 中完成。本机训练环境只用于资产验证、
小规模冒烟和可视化。

### Policy bundle

策略导出是训练与部署之间的唯一边界。策略包至少包含：

```text
policy_bundle/
├── policy.onnx | policy.pt
├── manifest.yaml
└── checksum.sha256
```

`manifest.yaml` 由导出工具生成，不手工维护。它至少冻结：

- 格式版本与模型格式；
- 模型输入、输出及 recurrent state；
- 关节名称和策略向量顺序；
- observation 顺序、缩放和历史；
- action 缩放、裁剪和默认关节位置；
- 控制周期、PD 参数和坐标约定。

部署启动时必须校验策略、manifest、模型维度和 checksum。缺失或不一致时直接失败，不回退到
虚假策略。

### Deployment runtime

`deploy/runtime/` 负责与后端无关的控制闭环：

```text
RobotState
    → command processing
    → observation
    → policy inference
    → action processing
    → safety arbitration
    → JointCommand
```

runtime 不依赖 Isaac Lab、MuJoCo、ROS2 或具体电机驱动。策略推理只有一个明确入口，安全仲裁是
命令下发前的最后一道边界。

### sim2sim 与 sim2real

sim2sim 和 sim2real 使用相同的 runtime 和策略包，只替换后端：

```text
MuJoCo backend ─┐
                ├─ RobotState / JointCommand ─ runtime
Real backend ───┘
```

后端只负责：

- 初始化和关闭资源；
- 读取状态并报告新数据、超时或通信错误；
- 下发已经过安全仲裁的命令；
- 完成模型、协议与内部统一类型之间的转换。

后端不解释 observation、action 或策略关节顺序。sim2real 不依赖 MuJoCo，sim2sim 不依赖实机
通信栈。

## 依赖方向

```text
assets/config
      ↓
   training
      ↓
policy exporter
      ↓
 policy bundle
      ↓
deploy/runtime
   ↙       ↘
sim2sim   sim2real
```

禁止以下依赖：

- runtime 依赖 Isaac Lab；
- sim2sim 导入训练任务；
- sim2real 依赖 MuJoCo；
- 训练代码引用实机通信实现；
- 部署代码读取训练 checkpoint；
- 新工程构建或运行时依赖旧工程路径。

## 配置与事实来源

同一事实只允许一个可编辑来源。关节顺序、默认姿态、限位、控制周期、缩放和 PD 参数不得在
Python、C++、YAML 与模型中分别手工维护。

- 原始 CAD URDF 和 mesh 永久保存且不直接修改。
- 规范化 URDF 是提交 Git 的生成接口，不手工修改；提交前必须验证生成结果一致。
- USD、MJCF 和语言侧配置必须从规范化模型与共享配置可复现生成。
- 策略专属参数随策略包导出。
- MuJoCo 配置只描述仿真后端。
- 实机配置只描述通信、设备和实机运行参数。
- 软件版本由 Dockerfile、镜像摘要和构建断言定义，不另建描述性配置副本。

生成产物必须能够追溯到输入、工具版本和生成命令。`assets/` 保存版本化模型；`artifacts/`
只保存日志、checkpoint、临时转换结果和策略包。

## 实机安全不变量

- C++ 部署进程是电机命令的唯一发布者。
- 状态和命令携带时间信息，陈旧状态不得进入策略闭环。
- 非有限数值、状态超时、姿态异常、关节越界和通信错误必须进入安全状态。
- 急停与程序退出执行明确的阻尼和释放流程。
- 策略频率与电机命令频率分离，两次策略推理之间保持确定命令。
- 编码器零位、正方向、力矩单位和 IMU 坐标系必须与模型契约一致。
- sim2sim 通过后才允许同一策略包进入 sim2real。

具体 topic、消息字段和控制模式属于实机接口文档，不在架构层硬编码。

## 工程约束

- 文档使用中文，代码、日志和配置字段使用英文。
- 入口保持薄，配置解析、状态机、安全和日志不得长期堆积在 `main` 中。
- 不设置无限扩张的 `common/`、`misc/` 或 `utils/` 模块。
- 不引入通用插件系统、复杂依赖注入或无实际使用者的抽象。
- README 只保留安装、构建、运行和排障入口，设计细节放入 `docs/`。
- 旧训练与部署工程保持冻结；只有经明确授权，才针对具体设计问题进行有限参考。

## 迁移顺序

1. 将现有 Isaac Lab 骨架整体迁移到 `training/`，不修改任务逻辑。
2. 更新本机、Docker、VS Code 和文档入口，并完成任务注册及训练冒烟。
3. 定义机器人共享配置和可复现资产流程。
4. 定义策略包格式与导出验收。
5. 建立最小 C++ runtime 和 MuJoCo 后端。
6. 在 sim2sim 通过后接入实机通信与安全流程。

每一步独立验收，不在结构迁移中同时引入正式四足任务或部署实现。
