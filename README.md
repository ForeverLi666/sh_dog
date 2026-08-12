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

当前仅保留 Isaac Lab 官方 external project template 的 manager-based 单智能体
骨架。`ShDog-Template-v0` 仍是 Cartpole 占位任务，只用于验证项目注册与运行链路，
不得作为四足训练基线。

## 安装扩展

在已配置好的 `sh_dog` 环境中执行：

```bash
python -m pip install --no-deps --editable training/source/sh_dog
```

## 验证任务注册

```bash
python training/scripts/list_envs.py
```

预期只列出 `ShDog-Template-v0`。

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

验证容器内的任务注册：

```bash
docker compose -f docker/compose.train.yaml run --rm train \
  '/workspace/isaaclab/isaaclab.sh -p \
    /workspace/sh_dog/training/scripts/list_envs.py --keyword ShDog'
```

正式训练可按服务器资源提高共享内存上限：

```bash
SH_DOG_SHM_SIZE=8gb docker compose -f docker/compose.train.yaml run --rm train \
  '<training-command>'
```

Dockerfile、基础镜像摘要和构建末尾的版本断言共同定义训练软件环境。代理地址、镜像仓库认证
和服务器资源属于运行环境，不写入工程。训练日志、Hydra 输出和 checkpoint 写入
`artifacts/training/`，不混入源码目录；跨环境只导出 `artifacts/policies/` 中的
TorchScript/ONNX 策略包。

## 约定

- 文档使用中文，代码与配置字段使用英文。
- 原始 CAD URDF 永久保存且不直接修改。
- URDF 规范化、USD/MJCF 生成及 Python/C++ 配置生成必须可复现。
- RSL-RL checkpoint 不跨环境传递；跨环境只传 TorchScript/ONNX 策略包。
- 当前 `fastapi==0.115.7` 与 `starlette==0.49.1` 存在已知上游元数据冲突；
  本工程不启用 Isaac Sim HTTP services 或云端 livestream。
