# 接口契约与迁移

## 接手材料

收集实际 bit/HWH/构建清单、板卡与 PYNQ 版本、时钟复位、IP 参数、主机传输程序、波形采样方式、数据参考和目标指标。优先读取已有材料；缺项先列未知，不套用示例数字。

Overlay 加载核验顺序：核对构建哈希与设备 → 审查加载行为及当前硬件所有者 → 在已授权范围加载 → 核对 `ip_dict`/层级名称/参数 → 读取适用的版本/状态寄存器。`Overlay(path, download=False)` 的语义须按现场版本核实；不要把不下载等同于完全没有其他副作用。本包离线 HWH 工具不导入 PYNQ。

`register_map` 依赖元数据及驱动。先查字段访问属性，避免读取具有 clear-on-read 等副作用的寄存器；PG021 本包白名单只用于已识别的 AXI DMA。视频自定义 DMA 即使偏移相同，也不能直接使用。

## CSV/JSON 配置

配置见 `assets/packet.json`、`assets/video.json`。仅 JSON，避免离线依赖 YAML 解析库。

| 字段 | 约定 |
|---|---|
| schema_version | 当前为整数 1 |
| stream.data_width_bits | 正整数、8 的倍数；按实际流接口，而非 NumPy 元素宽度 |
| stream.framing | packet / video / continuous |
| stream.packet_bytes | 可选固定包有效字节数；省略时仅记录各包长度，不验证固定长度 |
| stream.expected_packets / expected_frames | 可选，只有连续、完整、从边界开始的窗口才检验总数 |
| stream.beats_per_line / lines_per_frame | 视频必需；一拍多像素时按实际每行传输拍数填写 |
| stream.max_no_handshake_cycles | 可选应用级无握手期限；省略时不判定死锁/超时 |
| capture.clock_hz | 实际被观测接口频率；未知填 null，不生成带宽数字 |
| capture.contiguous | 是否记录每个时钟边沿；不可因为 CSV 行连续就填 true |
| capture.start | boundary 表示第一包/帧从边界开始；partial 表示半途开始 |
| capture.complete_end | true 表示窗口已覆盖整个待测事务；false 不判定截断尾部为缺失边界 |
| signals | 规范键到实际 CSV 列名；cycle/resetn/valid/ready/data 必需 |

映射键支持 `keep,strb,last,user,id,dest`。packet/video 必须有 last，video 必须有 user。无 TKEEP 时默认全部字节保留；无 TSTRB 时视作所有保留字节都是数据字节。不要删除实际存在、对结论关键的边带来回避错误。

数值统一：十进制、`0x` 十六进制、`0b` 二进制；列名必须精确。cycle 是采样时钟的整数周期索引，递增；在 contiguous 模式不能跳号。数据无效时允许 payload 为 X；握手控制未知则输入无效。有效拍 payload 未知不应计入可信验证。

单流模型支持观察 ID/DEST 阻塞稳定性，但不实现按 ID/DEST 的包交织重组。交织流应按流分组后分别建立连续性与包边界证据，不能简单过滤掉其他流的周期后伪造连续采样。

## 主机与 DMA 契约

板上示例用 uint8 传递已打包字节，避免把元素数当字节数。其他 dtype 在调用前明确：`nbytes=元素数×itemsize`、符号位、定点缩放、字节序、通道/像素布局、尾拍 TKEEP。

`assets/board_contract.json` 中 tx_bytes、接收容量和实际预期输出分别定义；输出可变长的算法应提供长度判定，不能用整个分配缓冲区作数据参考。示例只支持 identity，其他运算传入独立黄金参考及明确输出字节数。

板上适配器读取实际参数：

- `C_INCLUDE_SG`、`C_MICRO_DMA`、`C_SG_LENGTH_WIDTH`、`C_ADDR_WIDTH`；
- 所用方向的 `C_INCLUDE_MM2S/S2MM`、`C_INCLUDE_MM2S_DRE/S2MM_DRE`；
- 所用方向的 `C_M_AXI_MM2S_DATA_WIDTH/C_M_AXI_S2MM_DATA_WIDTH`。

大小写可归一化；缺少参数必须查现场导出结果。某工具版本缺项时需要审查并显式扩展适配器，不用猜测默认值。最大直接长度为 `(1<<长度位宽)-1`，不是无限 NumPy 数组长度；接收容量必须容纳最大可能包。

地址对齐依据实际内存接口和 DRE 能力。TX/RX 不允许重叠，缓冲区须连续且具有设备地址。检查实际可访问的 HP/HPC/ACP 连接及地址范围；地址位宽合法不等于总线映射可达。

## 两类可迁移示例

1. 包流：滤波器、FFT、加速算子。先用单包/多包 identity 确认数据路径，再换独立参考。变更 IP 时复用检查器，只改位宽、长度、打包和启动钩子。
2. 视频：摄像头或视频处理 IP。SOF/EOL 按被观测端口定义；分辨率变换后输出边界应重新计算。板上 VDMA 控制须另行适配，本包 PG021 适配器不用于启动 VDMA。

换板迁移清单：设备与 PYNQ/IP 版本 → 实际接口参数 → 地址映射与一致性 → 时钟/复位 → 单包 → 多包 → 真实源速率/反压 → 正确性 → 性能。记录哪些步骤只经过离线、仿真或实板验证。
