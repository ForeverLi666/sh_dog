# 运行产物

此目录只保存不提交 Git 的运行产物和可交付策略包，不保存机器人资产：

```text
artifacts/
├── training/              # Docker 训练日志、Hydra 输出和 checkpoint
└── policies/              # 可交付的 TorchScript/ONNX 策略包
```

`training/` 只属于对应 Docker 训练环境。跨环境只传递 `policies/` 中经过校验的策略包。
