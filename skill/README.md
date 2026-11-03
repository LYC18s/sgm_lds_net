# PYNQ AXI4-Stream诊断工具

适用于单时钟、非交织的AXI4-Stream数据/视频链路，以及普通AXI DMA Simple。提供握手与反压检查、包/行/帧边界检查、黄金字节比对、DMA状态解码、性能统计及可复用RTL/板端模板。

离线工具使用Python 3.8+标准库，不需连接板卡：

```bash
python -B skill/scripts/selftest.py --workspace .
python -B skill/scripts/axis_tool.py --help
```

自测脚本生成合成样例并验证正常/注错用例；输出写入`skill/examples/generated/`和`skill/validation/`，两者均不纳入Git。合成样例只验证诊断工具，不能充当目标板的实测数据。

接口配置模板位于[assets](assets/)，协议与诊断方法见[references](references/)，AI工具入口见[SKILL.md](SKILL.md)。SG、Micro DMA、VDMA、多时钟延迟和交织流需要额外适配，不能直接套用Simple DMA寄存器解释。
