# 数据来源与类别

训练与评估使用COCO驾驶场景检测子集和BDD100K可行驶区域数据。图像、完整标签、下载缓存及历史实验数据不随源码分发。

检测类别依次为person、bicycle、car、motorcycle、bus、truck、traffic light、fire hydrant、stop sign、parking meter。道路标签：0=background，1=direct，2=alternative；255仅用于训练/评价中的忽略区域。

COCO原始标注来源及文件校验信息见[coco_sources.json](coco_sources.json)。BDD100K数据许可见[bdd100k_license.txt](bdd100k_license.txt)，数据许可与代码仓库许可分别适用。

模型训练库默认在`src/navigation/data/`查找：

```text
coco_stuff_driving/data.yaml
coco_stuff_driving/images/{train,val}/
coco_stuff_driving/labels/{train,val}/
lds_net_bdd_drivable/images/{train,val}/
lds_net_bdd_drivable/masks/{train,val}/
lds_net_bdd_drivable/manifest.json
```

数据准备及缓存配对逻辑位于`src/navigation/scripts/lds_net/data.py`。数据准备需要按数据集发布方的渠道自行获取原始图像和标注；仓库中的最终模型参数不能代替训练数据。
