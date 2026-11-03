# 软件验证

```bash
python -B sim/run_tests.py
```

需要NumPy，不需要PYNQ板卡。测试源码在`src/sgm_yolo/tests/`，覆盖报警策略、HUD保持、结果提交、颜色、UI以及模拟AXI设备下的驱动接口。

这些测试不配置摄像头、不加载位流，也不证明电气接口、整机时序或板端性能。RTL综合及导出方法见[build](../build/README.md)；通用AXI协议检查器与SystemVerilog测试台位于[skill/assets](../skill/assets/)。
