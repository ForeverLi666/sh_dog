# ShDog 资产

```text
sh_dog/
├── model.toml             # 规范化配置
├── raw/                   # 不修改的 CAD 导出输入
│   ├── urdf/
│   └── meshes/            # 唯一一套 STL
└── urdf/sh_dog.urdf       # 生成并提交的共享模型
```

`model.toml` 是机器人名称、执行器规格、传动比和关节顺序的机器可读来源。`raw/` 保持原始内容；
`urdf/sh_dog.urdf` 由工具生成，供训练和部署共同使用，不手工修改。

在仓库根目录生成或检查模型：

```bash
python tools/normalize_urdf.py
python tools/normalize_urdf.py --check
```

规范化 URDF 直接引用 `raw/meshes/`，不复制 STL。提交资产变更前必须执行 `--check`。
