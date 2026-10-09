# PIP2027 算法组考核代码

本仓库提供考生使用的题目接口、起始源码、构建配置和题目说明。
T0 是完整的流程示范；T1～T4 的算法需要自行实现。

## 题目入口

| 题目 | 内容 | 拓展总分 |
| --- | --- | ---: |
| [T0](problems/T0_ab/README.md) | A+B 与公开测评流程示范 | 不计分 |
| [T1](problems/T1_bipartite_matching/README.md) | 带权二分图最大权非空匹配 | 50 |
| [T2](problems/T2_dynamic_kth/README.md) | 动态序列区间第 k 大 | 60 |
| [T3](problems/T3_signal_identification/README.md) | 信号参数估计 | 50 |
| [T4](problems/T4_realtime_estimation/README.md) | ROS 2 实时去噪与滚动预测 | 90 |

T1～T4 的基础部分只判定通过与否；每题基础全部通过后才运行该题拓展。
各题独立编译和测评，接口、数据范围、评分及资源限制以对应题目 README 为准。

## 源码与编译约定

- T0～T3 的公开接口位于各题 `include/` 中，须保持类名、函数签名和对象布局不变。
  实现及辅助 `.cpp` 放在该题 `src/` 中，其中不能包含 `main()`。
- T4 的消息接口位于 `msg/SignalPacket.msg`。节点内部可自行设计，
  `src/` 中须恰有一个 `main()`，通信协议按 T4 README 实现。
- 正式测评递归编译当前题目 `src/` 下的全部 `.cpp`，使用官方接口。
  题目根目录的 `main.cpp` 和考生 `CMakeLists.txt` 不参与正式测评。

本地使用 CMake 和 C++17；T4 还需要 ROS 2 Humble。以 T0 为例，在仓库根目录运行：

```bash
cmake -S problems/T0_ab -B /tmp/pip27-t0-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/pip27-t0-build
/tmp/pip27-t0-build/t0_ab_demo
```

其余题目的构建命令见各题 README。编译成功只表示工程可用，
尚未实现的算法和回调需要完成后才能通过测评。

## 公开数据与测评示例

T3、T4 的公开输入、真值和配置保存在各题 `examples/` 中，
每组数据附有 `SHA256SUMS`，仅用于开发和自检，不计入正式成绩。
T4 的预览图保存在各公开点目录内。

[T0 公开测评示例](testing/T0_ab/README.md) 演示源码编译、逐点运行和资源限制。
T1～T4 的正式测评程序、测试点和答案保存在独立的测评机仓库中。
