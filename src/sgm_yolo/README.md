# FPGA 与板端软件

`hardware/rtl/`只包含当前Block Design实际引用的RTL及其递归依赖；`hardware/vivado_sgm/`提供Block Design与引脚/时钟/CDC约束，`hardware/vivado_2017/repo/`提供该设计依赖的第三方IP源码。

`dual_ov5640/`是PYNQ驱动，`native/lds_net_post.cpp`是C++检测后处理，`models/lds_net/`是最终LDS-Net模型的整数权重、调度程序、量化图及寄存器契约。`hardware/calibration/`是当前固定双目装配的标定；更换摄像头、镜头或基线后须重新标定。

从仓库根目录使用[统一构建入口](../../build/README.md)。生成的`build/lds_net_system/export/sgm_yolo.bit`和同名HWH须成对使用。驱动默认从本目录`overlay/`加载位流；[板端示例](../../board/demo.py)也允许显式传入位流路径。

Python模块保持原有相对目录，下载后可直接导入，无需目录还原。C++动态库由源码编译产生，不随仓库分发。
