# 历史失败证据

这里是本轮开发过程中产生的失败，不计入最终通过数量。最终冻结源码的结果在 ../ros-matrix/。

- humble-resolve-api.log：Humble API不匹配，后已改正并统一构建4包。
- height-suspend-before.log / native-ctest-height-suspend-before.log：新Runtime反例发现高度门外漏处理跟踪撤销，已修复。
- fc01-step4-native-matrix*：冻结前混合运行，有指令队列积压、故障注入前航点未到、测试期间源码或构建证据变化造成身份门失败。后使用生产keep-last1并修正普通命令安全事件计数，冻结后全部12场景重新运行。
- *repro.cpp：独立审查当时的纯内存反例。它们保留原始失败语义和旧的根因假设，不全部是现版正向回归入口；现版正向回归位于 native/tests/test_horizontal_*.cpp 与 tests/test_vision_step4_node_inputs.py。

不能把这里的运行当成已覆盖其名义故障：部分失败发生于故障注入之前；也不能删除这些记录或用最终PASS覆盖原FAIL。
