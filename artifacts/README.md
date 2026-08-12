# 运行产物

此目录保存不提交 Git 的训练产物和跨环境策略包：

```text
artifacts/
├── training/              # Docker 训练日志、Hydra 输出和 checkpoint
└── policies/              # 可交付的 TorchScript/ONNX 策略包
```

`training/` 只属于对应 Docker 训练环境。跨环境只传递 `policies/` 中经过校验的策略包。
