# ShDog 资产

`model.toml` 是机器人名称、执行器规格、额外传动比和标准关节集合的机器可读来源。原始 CAD
导出位于 `raw/`，不得直接修改。

在仓库根目录使用项目约定的 Python 3.11 环境生成规范化 URDF：

```bash
conda activate sh_dog
python tools/normalize_urdf.py
```

输出位于 `artifacts/assets/sh_dog/`，不提交 Git，可随时由原始资产和配置重新生成。
