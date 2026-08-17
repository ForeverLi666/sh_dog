# sh_dog

`sh_dog` 是自研四足机器人的单仓库工程。训练、资产、策略导出、MuJoCo
sim2sim、实机 sim2real 与共享 C++ policy runtime 在同一仓库内保持模块隔离。

## 固定训练栈

- Isaac Sim 5.1.0
- Isaac Lab v2.3.2，commit `37ddf626871758333d6ed89cf64ad702aef127d0`
- Python 3.11
- PyTorch 2.7.0+cu128
- RSL-RL 3.1.2
- PhysX

本机 Conda 环境名为 `sh_dog`，Isaac Lab 源码位于
`/home/lyh/sh_dog_isaaclab`。本机用于资产验证、小规模冒烟和可视化；Docker 用于正式训练、
checkpoint、定量诊断和策略导出。

## 当前状态

Isaac Lab 子工程已建立 ShDog manager-based 平地速度跟踪任务及最小 RSL-RL PPO 配置。机器人 raw
资产、规范化 URDF 和 primitive collision 已纳入 `assets/sh_dog/`；策略包和部署 runtime 尚未实现。
当前进度与下一步见 `docs/handoff.md`。

## 安装扩展

在已配置好的 `sh_dog` 环境中执行：

```bash
python -m pip install --no-deps --editable training/source/sh_dog
```

## 验证任务注册

```bash
python training/scripts/list_envs.py
```

预期列出 flat/rough 各自的训练与 Play 任务：`ShDog-Velocity-Flat`、`ShDog-Velocity-Flat-Play`、
`ShDog-Velocity-Rough` 和 `ShDog-Velocity-Rough-Play`。

## 验证平地任务

使用 Play 环境观察零 action 站立。Play 当前固定 `ang_vel_z=1.5 rad/s` 命令；任务使用运动配置
`SH_DOG_CFG`，不使用状态机站起配置：

```bash
python training/scripts/zero_agent.py --task ShDog-Velocity-Flat-Play --num_envs 1
```

`sh_dog_baseline` 对齐 Unitree 开源 Go2 velocity 的单帧 MLP、45D actor、60D 特权 critic、PPO、
速度课程、随机化和奖励；保留 ShDog 资产、运动 PD、关节顺序及接触名称。action scale 为
`0.25 rad`。`SH_DOG_CFG` 使用 `ShDogDcMotor` 按厂家 T-N 曲线和共享短时过载预算约束力矩；模型细节
见 `docs/robot_model.md`。

最小 PPO 冒烟命令为：

```bash
python training/scripts/rsl_rl/train.py \
  --task ShDog-Velocity-Flat --headless --num_envs 64 --max_iterations 2
```

rough 任务使用降低难度的官方 terrain generator 和地形课程，环境初始覆盖 level `0–5`，再按前进
距离动态升降级；actor/critic 使用 `1.6 m × 1.0 m`、`0.1 m` 分辨率的高度扫描。当前验证阶段仅
采样 `vx=0.5–1.0 m/s`，关闭横移、转向和 standing command，线速度跟踪 `std=0.4`。训练和 Play
均关闭命令箭头，避免加载远端 USD。使用独立实验目录运行冒烟：

```bash
scripts/training.sh train rough_smoke \
  --task ShDog-Velocity-Rough \
  --num-envs 64 \
  --max-iterations 2
```

带高度扫描的 actor 当前用于仿真对比，不直接作为无对应实机感知输入的部署策略。

## 验证站立

先生成 USD，再运行纯位置 PD 站立序列：

```bash
python scripts/build_usd.py
python training/scripts/stand.py
```

该命令使用本机已配置的 Isaac Lab 环境；仅使用 Docker 时按下一节生成 USD。

序列从 `0.40 m` 释放下蹲姿态，保持 `0.5 s` 后以五次平滑插值在 `0.5 s` 内站起。所有执行器参数
均为关节侧数值，上层不处理传动比。

## Docker 训练环境

无代理时构建训练镜像：

```bash
docker compose -f docker/compose.train.yaml build train
```

需要代理时由构建机器提供，不在工程中固定代理地址：

```bash
HTTP_PROXY=http://127.0.0.1:7897 \
HTTPS_PROXY=http://127.0.0.1:7897 \
NO_PROXY=localhost,127.0.0.1 \
docker compose -f docker/compose.train.yaml build \
  --build-arg HTTP_PROXY \
  --build-arg HTTPS_PROXY \
  --build-arg NO_PROXY \
  train
```

USD 是本地生成产物，不提交 Git。首次获取工程或模型发生变化后，使用已构建的训练镜像生成 USD；
命令会在转换完成后恢复宿主目录的文件所有者：

```bash
scripts/training.sh build-usd
```

验证容器内的任务注册：

```bash
docker compose -f docker/compose.train.yaml run --rm train \
  '/workspace/isaaclab/isaaclab.sh -p \
    /workspace/sh_dog/training/scripts/list_envs.py --keyword ShDog'
```

正式训练使用仓库级 Docker 入口。默认运行
`ShDog-Velocity-Flat`、4096 个环境、10000 iterations，并记录 TensorBoard 日志：

```bash
scripts/training.sh train baseline_10k
```

run name 必须显式提供；常用参数可以直接覆盖：

```bash
scripts/training.sh train smoke --num-envs 64 --max-iterations 2 --shm-size 8gb
```

未封装的训练参数放在 `--` 后传给 Isaac Lab，例如恢复指定 run：

```bash
scripts/training.sh train resumed -- --resume --load_run '<run-directory>'
```

TensorBoard 默认读取 `sh_dog_baseline` 实验并只监听本机 `6006` 端口：

```bash
scripts/training.sh tensorboard
```

日志目录、监听地址和端口均可覆盖。日志目录使用仓库相对路径：

```bash
scripts/training.sh tensorboard \
  --logdir artifacts/training/logs/rsl_rl/sh_dog_baseline \
  --host 127.0.0.1 \
  --port 6007
```

完整参数见 `scripts/training.sh --help`。

完成训练后使用同一 Docker 环境做定量评估。默认加载 nominal Play task，从 task 的
`limit_ranges` 生成归一化命令场景，每个场景运行 8 个 episode：

```bash
scripts/training.sh eval \
  artifacts/training/logs/rsl_rl/sh_dog_baseline/<run>/model_9999.pt
```

使用训练 task 评估 observation corruption、动力学随机化和 push 下的性能：

```bash
scripts/training.sh eval \
  artifacts/training/logs/rsl_rl/sh_dog_baseline/<run>/model_9999.pt \
  --task ShDog-Velocity-Flat
```

评估结果写入 checkpoint 所在 run 的 `eval/<checkpoint>/<task>/<timestamp>/`，包括解析后的
`protocol.yaml`、运行元数据、逐 episode CSV、逐关节力矩诊断和汇总 JSON。力矩诊断记录
computed/applied torque、裁剪占比、真实额定力矩超限、超限速度区间和保守过载预算。旧协议可原样重放；
协议的 timing、repeats 和 seed 不允许在重放时覆盖：

```bash
scripts/training.sh eval <checkpoint> --protocol <protocol.yaml>
```

任务名移除 `-v0` 前生成的评估协议仍可由新任务名原样重放。

rough 评估固定使用有序地形生成器，关闭评估期间的地形与命令课程，覆盖 6 类地形、等级
`0/3/6/9`、前进速度 `0.5/1.0 m/s`，每组默认重复 4 次：

```bash
scripts/training.sh eval \
  artifacts/training/logs/rsl_rl/sh_dog_rough/<run>/model_3000.pt \
  --task ShDog-Velocity-Rough-Play
```

结果额外记录地形类型、等级、前向进度、横向漂移和 yaw 漂移；存活且沿初始朝向前进至少 `3 m`
记为完成穿越。`random_rough` 的官方生成器忽略 difficulty 参数，因此其 level 仅用于覆盖不同地形
实例，不解释为难度。

Dockerfile、基础镜像摘要和构建末尾的版本断言共同定义训练软件环境。代理地址、镜像仓库认证
和服务器资源属于运行环境，不写入工程。训练日志、Hydra 输出和 checkpoint 写入
`artifacts/training/`，不混入源码目录；跨环境只导出 `artifacts/policies/` 中的
TorchScript/ONNX 策略包。

## 同步训练服务器

训练服务器使用专用目录接收本地工程的严格镜像。本地生成的 USD 会一并同步；远端 `.git/`、
`artifacts/`、缓存和日志不受同步影响。服务器地址和目标目录统一配置在
`scripts/sync_training.conf`，同步时直接运行：

```bash
scripts/sync_training.sh
```

首次运行会自动初始化不存在或为空的目标目录。目标目录必须以 `/sh_dog` 或 `/sh_dog_sync` 结尾；
非空目录缺少同步 marker 时脚本拒绝执行 `rsync --delete-delay`。本地缺少完整 USD 分层文件时同样
会失败，不会删除服务器上的可用资产。实机同步将在部署模块建立后单独实现，只允许同步部署相关
源码和策略包。

## 约定

- 文档使用中文，代码与配置字段使用英文。
- 原始 CAD URDF 永久保存且不直接修改。
- 规范化 URDF 提交 Git 但不手工修改；USD/MJCF 及语言侧配置必须可复现生成。
- RSL-RL checkpoint 不跨环境传递；跨环境只传 TorchScript/ONNX 策略包。
- 当前 `fastapi==0.115.7` 与 `starlette==0.49.1` 存在已知上游元数据冲突；
  本工程不启用 Isaac Sim HTTP services 或云端 livestream。
