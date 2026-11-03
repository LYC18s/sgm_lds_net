# 模型定义、量化与训练代码

`scripts/lds_net/`提供最终LDS-Net模型所用算法及其依赖，包含：

- `model.py`：LDS-Net模型、检测头与分割分支。
- `training.py`：基础训练、数据加载及模型恢复。
- `deploy_graph.py`、`deploy_quant.py`：部署图与混合精度量化算子。
- `unstructured.py`：LDS-Net非结构化剪枝、掩膜约束与QAT；LDS-Net目标比例为50%、65%、65%、40%，网络形状保持不变。
- `integer_reference.py`、`sparse_weights.py`：整数参考与稀疏格式。
- `quant_export.py`：量化导出与数值检查。

部分依赖模块沿用早期研发中的名称，但在当前训练库中承担公共数据、损失或评价函数，不包含历次实验的输出文件。

`requirements-training.txt`记录原训练环境版本。CPU整数推理只需`requirements-runtime.txt`中的三个库。请使用与自身平台匹配的已有环境；仓库不附带虚拟环境。

算法库的调用起点是`LDSNetModel`、`DeployGraph`、`SparseTrainingModel.from_training(..., 'LDS-Net')`及`train_sparse()`。它们不是下载后自动执行的训练队列。LDS-Net QAT需要匹配的稠密W4A6训练模型、FP32教师、校准清单、评价基线和完整数据；本源码仓库仅提供最终整数参数，不分发这些训练检查点。重新训练时应自行准备输入，并显式设置数据及输出位置。

`scripts/lds_net/data.py`以本navigation目录为根目录；数据结构见[数据说明](../../data/README.md)。评价函数从`data/evaluation/attributes.json`读取图像属性，从`data/evaluation/road_evaluation.json`读取选定评价ID列表；使用自己的数据时需相应准备这两个文件。训练和验证划分须保持独立，不能将用于选模的数据称为独立测试集。
