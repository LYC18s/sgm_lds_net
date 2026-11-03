# 规范与实现出处

核对日期：2026-10-24。只保存必要引用与工程化解释，不复制整本规范。现场行为以匹配版本文档和已安装驱动为准。

| 来源 | 用途与版本边界 |
|---|---|
| [Arm AMBA AXI4-Stream IHI 0051A](https://documentation-service.arm.com/static/642583d7314e245d086bc8c9) | 握手、字节限定、边带和包边界的基础；本包检查其适用子集 |
| [AMD PG021：S2MM 状态](https://docs.amd.com/r/en-US/pg021_axi_dma/S2MM_DMASR-S2MM-DMA-Status-Register-Offset-34h) | AXI DMA 7.1 状态、错误、Idle/Halted、IOC 语义 |
| [AMD PG021：MM2S 状态](https://docs.amd.com/r/en-US/pg021_axi_dma/MM2S_DMASR-MM2S-DMA-Status-Register-Offset-04h) | 发送通道状态与模式相关限制 |
| [AMD PG021：接收长度](https://docs.amd.com/r/en-US/pg021_axi_dma/S2MM_LENGTH-S2MM-DMA-Buffer-Length-Register-Offset-58h) | 容量与实际接收字节；不能用小于预期最大包的缓冲区 |
| [AMD UG934：视频行结束](https://docs.amd.com/r/en-US/ug934_axi_videoIP/End-of-Line-Signal) | TLAST 为 EOL，与整帧结束不同 |
| [AMD UG934：视频指南](https://www.amd.com/content/dam/amd/en/documents/products/adaptive-socs-and-fpgas/technologies/axi4-stream-video-ip-and-system-design-guide.pdf) | TUSER[0] 的 SOF 及视频边界约定 |
| [PYNQ Allocate](https://pynq.readthedocs.io/en/latest/pynq_libraries/allocate.html) | 可供设备访问的缓冲区、device_address、coherent、flush/invalidate |
| [PYNQ DMA 文档](https://pynq.readthedocs.io/en/latest/pynq_libraries/dma.html) | 基础使用和 TLAST 完成条件；文档模式说明不能替代现场版本核验 |
| [Xilinx/PYNQ DMA 源码](https://github.com/Xilinx/PYNQ/blob/master/pynq/lib/dma.py) | 核验驱动启动、长度、对齐、缓存和模式分支；master 是变化分支，不作为固定部署版本 |

本包没有复制 PYNQ 驱动代码，直接寄存器模板为独立实现。现场版本的源码路径/文件哈希应作为环境记录附入报告。没有网络也可运行本包离线案例。

