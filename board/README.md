# PYNQ板端运行

硬件：ALINX AX7020/XC7Z020、双OV5640、HDMI显示器。使用适配该板卡的PYNQ Linux，并准备NumPy。

1. 按[构建说明](../build/README.md)生成匹配的bit/HWH，并编译ARM C++解码器。
2. 将仓库放到板上。bit与HWH保持同名、位于同一目录，不能混用不同版本。
3. 在板上从仓库根目录运行：

```bash
python -B board/demo.py --bitfile src/sgm_yolo/build/lds_net_system/export/sgm_yolo.bit
```

示例启动采集及检测，每秒打印检测结果，按Ctrl+C停止并释放驱动。若将匹配的bit/HWH放入`src/sgm_yolo/overlay/`，也可省略`--bitfile`。

相机编号、I2C总线、引脚和标定与当前固定双目装配有关，移植时须核对。标定文件位于`src/sgm_yolo/hardware/calibration/`；更换装配后不能沿用原测距参数。
