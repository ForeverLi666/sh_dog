# 工程交接

## 当前目标

建立训练、策略导出、sim2sim 和 sim2real 共用的机器人模型与策略契约。当前处于架构迁移第 3 步：
定义共享配置和可复现资产流程。

## 已完成

- Isaac Lab external project 已迁入 `training/`，`ShDog-Template-v0` 仍是 Cartpole 链路测试。
- 原始 CAD URDF 和唯一一套 STL 已保存在 `assets/sh_dog/raw/`。
- `model.toml` 定义名称、关节顺序、执行器规格和额外传动比。
- `assets/sh_dog/urdf/sh_dog.urdf` 可复现生成、提交 Git，并由训练和部署共享。
- `model.toml` 定义每类 link 的最小 primitive collision，四腿由生成器镜像。

## 当前边界

- raw 资产不修改；规范化 URDF 不手工修改。
- collision 已完成结构简化和导入验证，尚未结合正式任务完成落地接触验收。
- USD、MJCF、正式四足任务、策略包和 C++ 部署均未开始。
- `artifacts/` 只保存不提交的日志、checkpoint、临时结果和策略包。

## 下一步

1. 建立最小 ShDog 四足资产配置和默认站立姿态。
2. 在 Isaac Lab 中执行落地、关节驱动和接触可视化测试。
3. 根据实际接触结果微调 `model.toml` 中的 collision，不修改 raw mesh 或惯量。
4. collision 验收后建立正式四足任务和训练配置。

## 接手检查

```bash
git status --short
python tools/normalize_urdf.py --check
python training/scripts/list_envs.py
```

设计约束见 `docs/architecture.md`，机器人语义见 `docs/robot_model.md`，常用命令见根目录 `README.md`。
