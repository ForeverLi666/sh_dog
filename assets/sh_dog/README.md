# ShDog 资产

```text
sh_dog/
├── model.toml             # 模型与 collision 配置
├── raw/                   # 不修改的 CAD 导出输入
│   ├── urdf/
│   └── meshes/            # 唯一一套 STL
├── urdf/sh_dog.urdf       # 生成并提交的共享模型
└── usd/sh_dog.usd         # 本地生成的 Isaac Sim 资产
```

`model.toml` 定义机器人接口、执行器和碰撞体。`raw/` 保持原始内容；`urdf/sh_dog.urdf` 由工具
生成，供训练和部署共同使用，不手工修改。

在仓库根目录生成或检查模型：

```bash
python tools/normalize_urdf.py
python tools/normalize_urdf.py --check
python tools/build_usd.py
```

规范化 URDF 直接引用 `raw/meshes/`，不复制 STL。提交资产变更前必须执行 `--check`。
USD 及其分层文件固定生成到 `usd/`，不提交 Git；它们由 Isaac Sim 版本和 URDF 唯一确定。
