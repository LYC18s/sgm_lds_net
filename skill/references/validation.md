# 验证方法

运行`python -B skill/scripts/selftest.py --workspace .`生成正常包流、合法反压、缺少TLAST、阻塞时改变数据、视频EOL异常等合成用例，并检查诊断结果是否符合预期。测试代码位于`skill/tests/`。

HDL检查器和测试台位于`skill/assets/axis_checker.sv`及`tb_axis.sv`，可使用已有仿真器运行。接入真实算法后仍需要独立的黄金数据、边界条件、ILA/波形及板端测量；工具自测不能证明应用系统正确或已经跨板验证。
