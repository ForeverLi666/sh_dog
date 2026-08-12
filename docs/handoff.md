# 工程交接

## 当前目标

建立训练、策略导出、sim2sim 和 sim2real 共用的机器人模型与策略契约。当前处于架构迁移第 3 步：
定义共享配置和可复现资产流程。

## 已完成

- Isaac Lab external project 已迁入 `training/`，`ShDog-Template-v0` 仍是 Cartpole 链路测试。
- 原始 CAD URDF 和唯一一套 STL 已保存在 `assets/sh_dog/raw/`。
- `model.toml` 定义名称、关节顺序、执行器规格和额外传动比。
- `assets/sh_dog/urdf/sh_dog.urdf` 可复现生成、提交 Git，并由训练和部署共享。

## 当前边界

- raw 资产不修改；规范化 URDF 不手工修改。
- 当前 collision 仍是 CAD mesh，尚不能作为正式训练碰撞模型。
- USD、MJCF、正式四足任务、策略包和 C++ 部署均未开始。
- `artifacts/` 只保存不提交的日志、checkpoint、临时结果和策略包。

## 下一步

1. 从 ShDog mesh 和 link 坐标系测量各 link 的实际包围尺寸。
2. 在机器可读配置中定义最少 primitive collision，并由规范化工具生成。
3. 验证 visual/collision 叠加、全关节范围自碰撞和 Isaac Lab 导入落地。
4. collision 验收后建立正式四足任务，不修改本步骤以外的训练逻辑。

## 接手检查

```bash
git status --short
python tools/normalize_urdf.py --check
python training/scripts/list_envs.py
```

设计约束见 `docs/architecture.md`，机器人语义见 `docs/robot_model.md`，常用命令见根目录 `README.md`。
