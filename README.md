# SGM + LDS-Net Stereo Perception

基于 Zynq-7020 的双目视觉感知系统。PL 完成双 OV5640 图像采集、双目校正、四路径 SGM、LDS-Net 整数网络与 HDMI 输出；PS 完成设备控制、检测解码、测距融合和界面渲染。

面向 **ALINX AX7020 / XC7Z020**、双 OV5640 摄像头和 PYNQ Linux。逻辑左目对应物理 camera 1。最终网络采用 **LDS-Net、混合 W4A6、320×320 输入**，同时输出目标检测与三类可行驶区域。

## 仓库结构

| 路径 | 内容 |
|---|---|
| [src/sgm_yolo/](src/sgm_yolo/README.md) | 当前 FPGA RTL、Block Design、约束、板端 Python/C++ 驱动及模型参数 |
| [src/navigation/](src/navigation/README.md) | LDS-Net 模型定义、训练/量化/剪枝代码及独立整数推理入口 |
| [build/](build/README.md) | 从源码创建 Vivado 工程、综合、实现、导出和编译 C++ 解码器的入口 |
| [sim/](sim/README.md) | 软件回归入口和验证方法 |
| [board/](board/README.md) | PYNQ 板端运行示例 |
| [data/](data/README.md) | 数据集来源、类别与许可说明 |
| [skill/](skill/README.md) | 可复用的 PYNQ AXI4-Stream 诊断工具与模板 |
| [report/](report/README.md) | 设计报告、大模型协作记录、技能提炼与架构说明 |

这是源码仓库。模型参数、标定映射和 UI 资源作为运行所需输入随源码提供。位流、HWH、原生动态库、训练检查点、完整数据集和构建输出由使用者准备或生成。

## 快速开始

从仓库根目录运行以下命令。`python` 表示当前环境中已安装的 Python。

```bash
# 检查源文件、模型参数及仓库结构；仅使用 Python 标准库
python -B build/check.py

# 软件回归，需要 NumPy，无需连接板卡
python -B sim/run_tests.py

# CPU 整数推理，需要 NumPy、OpenCV 和 PyTorch
python -B src/navigation/runtime/infer.py --image path/to/image.jpg --output build/prediction
```

推理输出检测框 `detections.json`、道路类别图 `road_labels.png` 和五路整数张量 `raw_int8.npz`。CPU 参考用于数值与接口核对，不能代表 FPGA 性能。

## FPGA 构建

需要已经安装的 Vivado 2026.1 和 Python 3.9+。在 Windows 下显式指定实际安装位置：

```powershell
python -B build/build.py --stage project --vivado "<Vivado安装目录>/bin/vivado.bat"
python -B build/build.py --stage route --vivado "<Vivado安装目录>/bin/vivado.bat"
python -B build/build.py --stage export --vivado "<Vivado安装目录>/bin/vivado.bat"
```

构建入口创建工程并执行相应阶段，输出及临时目录均位于仓库内。无需原作者的工程目录、旧 XPR、历史检查点或盘符映射。具体依赖、输出位置与板端步骤见 [构建说明](build/README.md) 和 [上板说明](board/README.md)。

## 状态与限制

- 网络和 SGM 的目标工作时钟为 100 MHz；HDMI 使用项目既有时序配置。
- 网络输入采用中心裁剪与缩放；离线推理入口采用 letterbox。比较输出前应对齐输入预处理方式。
- 训练数据不随仓库分发。训练相关依赖和算法入口见 [模型开发说明](src/navigation/training.md)。

## 许可证

项目原创代码采用 [MIT License](LICENSE)，版权署名为 `Copyright (c) 2026 LDS-Net contributors`。第三方代码、依赖及数据保留各自的许可，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
