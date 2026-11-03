# LDS-Net

共享骨干的目标检测与道路分割网络：37个普通卷积、8个Add、3次最近邻上采样；检测输出P4/P5，分割输出background/direct/alternative。输入固定320×320，采用混合W4A6，首层及五个末端卷积保留8位权重。

| 路径 | 用途 |
|---|---|
| `runtime/infer.py` | 单张图片整数参考推理，默认读取`model/` |
| `runtime/lds_net/` | 整数卷积、稀疏权重解码、预处理及PS后处理实现 |
| `runtime/ps_postprocess.hpp` | 可移植C++后处理参考 |
| `model/dense_integer/` | 最终LDS-Net的稠密整数参数与图 |
| `model/sparse_integer/` | 同一模型的稀疏存储形式 |
| `scripts/lds_net/` | 模型、量化和LDS-Net训练所依赖的Python源码库 |

从仓库根目录执行：

```bash
python -B src/navigation/runtime/infer.py --image path/to/image.jpg --output build/prediction
```

添加`--dense`使用稠密参考；默认使用稀疏存储。两者计算相同网络。推理需要NumPy、OpenCV与PyTorch，不需要加载训练检查点或安装Ultralytics。

完整张量接口见[架构说明](../../report/architecture.md)。训练代码及依赖见[training.md](training.md)。
