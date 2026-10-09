# T4：ROS 2 实时信号估计

## 题目目标

测评机以 ROS 2 topic 按时间顺序发布一维连续信号的含噪观测。考生节点需要在线完成当前观测的去噪；在拓展模式下，还要以 200 Hz 持续发布从当前时刻开始的未来预测序列。

题目不限定算法。可以使用滑动窗口回归、局部多项式、频域方法、卡尔曼滤波、其他状态估计方法，或自行组合多种方法。测评只检查消息协议、时间戳、实时性和整体数值误差。

本题使用 ROS 2 Humble。正式测评读取冻结数据；模板仓库提供公开样例、消息接口和可编译运行的节点起始代码，算法及业务回调由考生完成。

## ROS 2 接口

考生节点名须为 `/signal_estimator`。测评机通过以下接口通信：

| 方向 | Topic | 消息类型 |
| --- | --- | --- |
| 测评机 → 考生 | `/signal/observation` | `t4_realtime_estimation/msg/SignalPacket` |
| 考生 → 测评机（基础） | `/signal/estimate` | `t4_realtime_estimation/msg/SignalPacket` |
| 考生 → 测评机（拓展） | `/signal/prediction` | `t4_realtime_estimation/msg/SignalPacket` |

模式通过 ROS 2 标准参数服务单独设置：

```text
/signal_estimator/set_parameters_atomically
```

参数名为 `assessment_mode`，取值为 `0、1、2、3`。模式在第一帧观测到达之前设置一次；运行过程中不会切换模式。

唯一的消息接口文件是 [`msg/SignalPacket.msg`](msg/SignalPacket.msg)：

```text
std_msgs/Header header
float64[] values
```

`header.stamp` 是测评协议中的虚拟时间戳。它来自观测数据本身，不能用系统墙钟替换。`header.frame_id` 不参与数值判定，可以原样传递或留空。

每帧输入观测的 `values.size() == 1`，`values[0]` 为 `header.stamp` 时刻的含噪观测量。测评机只提供这些观测和模式参数，不提供原始信号、噪声标准差或未来观测。

双方 QoS 为 `Reliable`、`Volatile`、`KeepLast(256)`。测评机先等待参数服务和 DDS 通信建立，再发送首帧；每个测试点使用新进程，历史状态不能跨测试点复用。

### 模式和数组长度

| 模式 | `assessment_mode` | 观测频率 | 预测窗口 | 每条预测消息的 `values` 长度 |
| --- | ---: | ---: | ---: | ---: |
| Basic | 0 | 200 Hz | 无 | 1 |
| Extra1 | 1 | 100 Hz | 0.1 s | 21 |
| Extra2 | 2 | 50 Hz | 0.2 s | 41 |
| Extra3 | 3 | 50 Hz | 0.5 s | 101 |

拓展数组的采样间隔固定为 5 ms，数组包含当前时刻，因此点数为 `窗口 / 0.005 + 1`：

- Extra1：`values[0]` 是当前时刻，`values[1..20]` 是之后 20 个点；
- Extra2：`values[0]` 是当前时刻，`values[1..40]` 是之后 40 个点；
- Extra3：`values[0]` 是当前时刻，`values[1..100]` 是之后 100 个点。

### Basic 输出

每收到一帧观测，节点发布一帧 `/signal/estimate`：

```text
values.size() == 1
values[0]   == 当前观测时刻的估计原始值
header.stamp == 对应观测的 header.stamp
```

测评机从发布观测消息的墙钟时刻开始计算等待时间。单帧输出超过 20 ms、缺少输出，或输出时间戳与输入时间戳不完全相同，当前测试点直接无效。

### 拓展输出

拓展模式持续向 `/signal/prediction` 发布完整数组：

```text
values.size() == 当前模式规定的长度
values[0]    == header.stamp 对应的当前估计值
values[i]    == header.stamp + i * 5 ms 对应的预测值
```

观测频率低于 200 Hz 时，节点仍然需要按 200 Hz 发布预测消息。预测消息缺包时，整个测试点记 0 分；消息的起点由 `header.stamp` 给出，不能把时间戳编码到 `values` 中。

预测窗口随每次发布的当前时刻向前移动。整个观测过程中，节点每 5 ms 都需要更新并发布一条完整预测数组；两帧观测之间也要持续发布，只能使用发布当时已经收到的观测。

例如 Extra3 的窗口为 0.5 s，按理想的 5 ms 发布周期举例（时间相对首帧）：

| 当前发布时刻 | 本条消息覆盖的时段 | 数组长度 |
| --- | --- | ---: |
| 3.000 s | 3.000–3.500 s | 101 |
| 3.005 s | 3.005–3.505 s | 101 |
| 3.010 s | 3.010–3.510 s | 101 |

测评机对整个测试点内的每条预测消息，按其 `header.stamp + i × 5 ms` 查找真实值，再汇总所有数组的误差。

设首帧观测的虚拟时间为 `t0`。必需预测包的起点固定为 `t0 + k × 5 ms`，`k=0,1,...`，直到不超过末帧观测时间；首帧就需要输出，不删除启动阶段。输出数组只能使用已经接收且时间戳不晚于本次起点的观测。系统时钟可用于安排发布周期，数组中的信号时间必须沿上述虚拟网格递增。

测评机按首帧的实际发布时刻，将虚拟网格映射到相同速率的真实时间。每个预测包须在对应网格时刻起 20 ms 内送达；提前发送未来起点、重复起点、乱序起点或缺少任意起点，整点无效。末帧之后可以继续正常输出，超过必需起点范围的预测包不计入本点误差。

实时调度可通过首帧订阅回调中的 `rclcpp::MessageInfo` 读取 DDS `source_timestamp`，将首帧发布时间映射到本进程的单调时钟。若把首次回调的到达时刻作为零点，首帧通信或回调延迟会持续平移后面的预测包。DDS 时间只用于安排真实发送时刻；预测数组的数值时间仍由 `header.stamp` 决定，首帧及后续真实迟到包均正常判罚。

## 数据生成

### 连续轨迹

原始信号是一条随机生成的连续平滑曲线，由不同宽度的高斯基函数叠加并标定幅度、基线得到，局部变化速度和曲率随时间改变。

各组局部变化的时间尺度如下：

| 测试组 | 局部事件尺度范围 |
| --- | --- |
| Basic | 0.18–0.45 s |
| Extra1 | 0.45–0.75 s |
| Extra2 | 0.40–0.70 s |
| Extra3 | 0.55–0.90 s |

公开数据中的 `truth.csv` 只用于离线开发和检查；正式测评时考生节点只能使用在线收到的观测。

### 噪声

观测量为：

$$
y(t_i)=x(t_i)+\epsilon_i,\qquad
\epsilon_i\sim\mathcal N(0,\sigma^2)。
$$

噪声独立、零均值，只叠加到最终完整信号上。定义有效变化幅度和实际噪声比例：

$$
A_{\mathrm{robust}}=P_{95}(x)-P_5(x),\qquad
\rho=\frac{\sigma}{A_{\mathrm{robust}}}。
$$

`P95`、`P5` 分别为原始信号的第 95、5 百分位数。正式和公开数据均使用以下五个目标噪声比例档位：

```text
0.02、0.05、0.10、0.15、0.20
```

记目标档位为 `rho_target`，最终使用的噪声标准差为：

$$
\sigma=\min\left(\rho_{\mathrm{target}} A_{\mathrm{robust}},\;0.3\operatorname{RMS}(x)\right)。
$$

实际 `rho` 不超过 0.20，且 `sigma/RMS(x)` 不超过 0.30。公开样例的目录名和 `config.json` 中的 `noise_ratio` 表示目标档位，`effective_noise_ratio` 和 `noise_stddev` 分别记录实际 `rho`、实际 `sigma`；下方样例表使用实际值。这些噪声信息不通过 ROS 2 运行时接口提供给考生。

### 时间戳和抖动

观测按照原速投喂，不人为加速。每个观测周期围绕名义周期加入不超过 ±5% 的输入时间抖动；抖动体现在消息 `header.stamp` 的虚拟时间中。测评机不会向考生额外注入输出抖动，输出周期的实际抖动来自操作系统调度和 ROS 2 通信环境。

## 正式测试矩阵

正式数据固定随机种子生成后保存，不在评分过程中重新生成。四个测试组各 10 点；每组的五个目标 `rho` 各出现两次，重复点使用独立轨迹、时间戳和噪声随机流。触发 RMS 上限后，实际噪声比例以配置中的 `effective_noise_ratio` 为准，高档目标 `rho` 不保证实际噪声严格递增。表中 10 s 是名义时长，实际末帧时间由全部含抖动的采样周期累计得到。

| 测试组 | 模式 | 观测频率 | 虚拟时长 | 观测帧数 | 预测数组 | 测试点 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Basic | 0 | 200 Hz | 10 s | 2001 | 1 点 | 1–10 |
| Extra1 | 1 | 100 Hz | 10 s | 1001 | 21 点 | 11–20 |
| Extra2 | 2 | 50 Hz | 10 s | 501 | 41 点 | 21–30 |
| Extra3 | 3 | 50 Hz | 10 s | 501 | 101 点 | 31–40 |

## 测评和计分

### 测评流程

1. 编译考生节点，并核验冻结测试数据的校验和。
2. 为当前测试点启动新进程，设置 `assessment_mode`，等待 DDS 通信匹配；匹配完成后固定等待 250 ms，再开始投喂。
3. 按观测虚拟时间间隔原速发布含噪数据，同时检查输出协议、完整性、实时性及资源使用。每个测试点显示进度条；非交互终端显示逐行进度。
4. 将输出时间戳与保存的真实信号对齐；不在真值采样网格上的时刻用相邻真值线性插值。汇总整个测试点的 RMSE，并发送 SIGINT，检查节点正常退出。
5. 只有消息协议、包完整性、实时性、整体数值误差和进程资源及退出状态全部达标，当前测试点才通过。输出逐点结果和最终拓展得分。

### 基础门槛

Basic 只判定通过或不通过，不计入拓展分数。10 个 Basic 测试点全部通过后，才运行三个拓展组；否则拓展标记为 `SKIPPED`。

### 拓展分值

| 测试组 | 测试点 | 每点分值 | 小计 |
| --- | ---: | ---: | ---: |
| Extra1 | 10 | 2 | 20 |
| Extra2 | 10 | 3 | 30 |
| Extra3 | 10 | 4 | 40 |
| **拓展合计** | **30** |  | **90** |

拓展测试点逐点计分；某点只要任一通过条件失败，该点得分为 0，其他点继续测评。三个拓展组互不构成前置条件。

### 数值误差

测评机先按消息时间戳将考生输出与隐藏连续轨迹对齐，然后在一个完整测试点内计算整体 RMSE。Basic 只有当前值，使用普通 RMSE。拓展数组采用固定的温和时间权重：

$$
w_j=2^{-h_j/H},\qquad
\widetilde w_j=\frac{w_j}{\sum_{r=0}^{J-1}w_r},\qquad
H=h_{J-1}。
$$

其中 `h_j=j×0.005 s`，`H` 是当前模式的预测窗口长度，窗口末端权重为起点的一半。每个完整预测数组先单独归一化权重，再对所有数组求平均：

$$
\mathrm{RMSE}_{w}=\sqrt{\frac{1}{M}\sum_{i=1}^{M}
\sum_{j=0}^{J-1}\widetilde w_j(\hat x_{i,j}-x_{i,j})^2}。
$$

这种权重只适度降低较远预测点的占比，仍然要求考生完成整个预测窗口。测评报告同时给出不加权的诊断 RMSE：

$$
\mathrm{RMSE}_{\mathrm{uniform}}=\sqrt{\frac{1}{N}\sum_{i=1}^{N}(\hat x_i-x_i)^2}。
$$

不对单个采样点设置独立的数值门槛，正式拓展判定使用 `RMSE_w`。报告同时给出：

```text
RMSE
RMSE / sigma
RMSE / A_robust
```

其中 `RMSE / sigma` 衡量误差相当于观测噪声的多少倍，`RMSE / A_robust` 衡量误差占原始信号有效变化幅度的比例；拓展中的 `RMSE` 指 `RMSE_w`。

数值通过门槛为：

$$
L=\sqrt{(c_{\mathrm{noise}}\sigma)^2+(c_{\mathrm{motion}}A_{\mathrm{robust}})^2},\qquad
\mathrm{RMSE}\le L。
$$

| 模式 | `c_noise` | `c_motion` |
| --- | ---: | ---: |
| Basic | 0.46 | 0 |
| Extra1 | 0.90 | 0.0026 |
| Extra2 | 0.99 | 0.0158 |
| Extra3 | 0.75 | 0.0742 |

测评报告同时列出两种归一化的实际值和通过上限：

$$
\frac{\mathrm{RMSE}}{\sigma}\le\frac{L}{\sigma},\qquad
\frac{\mathrm{RMSE}}{A_{\mathrm{robust}}}\le\frac{L}{A_{\mathrm{robust}}}。
$$

两式表达同一个综合误差门槛。噪声项表示去噪难度，运动项为未知连续曲线的外推误差保留空间，避免低噪声时要求未来预测误差趋近于零。系数由 20 个公开点和另 20 个独立点校准并随正式数据冻结；正式测试点没有参与门槛选择，门槛也不根据考生输出动态调整。

Basic 的整体 RMSE 使用所有当前值；拓展的整体 RMSE 使用所有完整预测数组中的全部点。缺少任意必需消息、时间戳不能匹配、数组长度错误、数值为 NaN/Inf 或超出时间限制时，当前测试点直接无效，不进入数值计分。

### 实时性

- Basic 单帧最大等待时间为 20 ms，起点是测评机发布该观测消息的墙钟时刻；
- 拓展目标发布频率为 200 Hz，目标周期为 5 ms；每个预测包须在对应网格时刻起 20 ms 内送达；
- 单个实际发布周期不设通过阈值，周期最小值和最大值只作诊断；
- 拓展有效输出总帧率不得低于 **190 Hz（200 Hz 的 95%）**，同时报告有效包数占必需包数的比例；任意必需包缺失仍使整点无效；
- 测评机不主动制造输出抖动，所有输出周期波动来自真实运行环境；
- 进程启动、DDS 建立、参数服务、数据投喂、输出等待和退出均计入资源统计。

每个测试点独立启动候选节点。测评脚本在 `stderr` 用进度条显示当前测试点进度；正式结果表输出到 `stdout`，非交互终端退化为逐行进度信息。

发布时刻使用 DDS `source_timestamp`，延迟终点使用 DDS `received_timestamp`；这两个真实时钟字段只用于实时性检查，数值对齐仍只用消息 Header 的虚拟时间戳。总帧率比例为 `(有效发布包数−1)×5 ms / (末包发布时间−首包发布时间)`，并与包覆盖率取较小值。测评机报告发布周期最小值/最大值（仅诊断）、迟到包数、最大延迟和帧率比例。

考生节点可使用 **2 个独立物理核心、RSS 256 MiB、虚拟地址空间 2 GiB、累计 CPU 20 s、总墙钟 17 s**。累计 CPU 时间为全部线程之和。启动和通信建立最多 5 s；测试完成后发送 SIGINT，须在 1 s 内正常退出。ROS 2、DDS、线程、初始化、在线处理和退出均计入节点资源统计，报告的墙钟时间不是单次算法耗时。

超过墙钟上限或退出宽限仍未结束时，测评机强制终止考生进程及其进程组。17 s 墙钟上限不放宽单包 20 ms 的送达期限。

测评整体使用四个独立物理核心：观测投喂、数据校验各占一个，考生节点使用另外两个。考生的全部线程和子进程继承这两个核心的可用集合；ROS 2 执行器和内部线程结构由考生自行设计。测评报告给出实际 CPU 分配，考生不能扩大到投喂或校验核心。同一物理核心的两个超线程不作为两个独立核心，测评环境须提供至少四个可用物理核心。

## 模板代码

[`src/estimator_node.cpp`](src/estimator_node.cpp) 是一个可运行的 ROS 2 起始工程，包含：

- 消息接口生成和 ROS 2 包构建配置；
- 观测订阅、基础发布器、拓展发布器和 5 ms 定时器的创建位置；
- `assessment_mode` 参数服务回调的注册位置；
- 一个保存历史观测的 `OnlineEstimator` 占位类。

回调和算法目前是空实现。考生可以修改、删除或重写 `src/` 下的全部代码，也可以拆分成多个源文件；不要求保留模板中的类、命名空间、回调名或算法结构。测评只依赖上面约定的消息接口、topic、参数和输出行为。

## 公开数据

模板仓库已包含 20 个公开点：四个测试组各 5 个，五个目标 `rho` 各一个。公开点与正式点使用相同的轨迹生成方法、输入时间抖动规则和采样配置，并使用独立的轨迹、时间戳和噪声随机流；公开点仅用于开发，不计入成绩。

每个公开点目录包含：

- `observations.csv`：`timestamp,observation` 两列，记录含噪观测；
- `truth.csv`：`timestamp,signal` 两列，记录原始信号的 1000 Hz 参考采样，覆盖末帧之后的完整预测窗口；
- `config.json`：模式、采样配置、目标噪声档位、实际 `rho`、实际 `sigma` 和生成元数据。

公开根目录的 [`MANIFEST.json`](examples/MANIFEST.json) 列出全部 20 点；[`SHA256SUMS`](examples/SHA256SUMS) 用于校验公开文件。

### Basic

| 实际 `rho` | 实际 `sigma` | 数据 |
| ---: | ---: | --- |
| 0.02 | 0.2277 | [examples/basic/rho_0p02](examples/basic/rho_0p02) |
| 0.05 | 0.4821 | [examples/basic/rho_0p05](examples/basic/rho_0p05) |
| 0.1 | 1.194 | [examples/basic/rho_0p10](examples/basic/rho_0p10) |
| 0.1445 | 1.361 | [examples/basic/rho_0p15](examples/basic/rho_0p15) |
| 0.2 | 1.333 | [examples/basic/rho_0p20](examples/basic/rho_0p20) |

### Extra1

| 实际 `rho` | 实际 `sigma` | 数据 |
| ---: | ---: | --- |
| 0.02 | 0.2251 | [examples/extra1/rho_0p02](examples/extra1/rho_0p02) |
| 0.05 | 0.517 | [examples/extra1/rho_0p05](examples/extra1/rho_0p05) |
| 0.1 | 0.7805 | [examples/extra1/rho_0p10](examples/extra1/rho_0p10) |
| 0.08722 | 0.7501 | [examples/extra1/rho_0p15](examples/extra1/rho_0p15) |
| 0.1357 | 0.9867 | [examples/extra1/rho_0p20](examples/extra1/rho_0p20) |

### Extra2

| 实际 `rho` | 实际 `sigma` | 数据 |
| ---: | ---: | --- |
| 0.02 | 0.1484 | [examples/extra2/rho_0p02](examples/extra2/rho_0p02) |
| 0.05 | 0.3669 | [examples/extra2/rho_0p05](examples/extra2/rho_0p05) |
| 0.1 | 1.114 | [examples/extra2/rho_0p10](examples/extra2/rho_0p10) |
| 0.1194 | 1.347 | [examples/extra2/rho_0p15](examples/extra2/rho_0p15) |
| 0.2 | 1.508 | [examples/extra2/rho_0p20](examples/extra2/rho_0p20) |

### Extra3

| 实际 `rho` | 实际 `sigma` | 数据 |
| ---: | ---: | --- |
| 0.02 | 0.1468 | [examples/extra3/rho_0p02](examples/extra3/rho_0p02) |
| 0.05 | 0.4643 | [examples/extra3/rho_0p05](examples/extra3/rho_0p05) |
| 0.09194 | 0.8171 | [examples/extra3/rho_0p10](examples/extra3/rho_0p10) |
| 0.1053 | 0.9739 | [examples/extra3/rho_0p15](examples/extra3/rho_0p15) |
| 0.09877 | 0.6943 | [examples/extra3/rho_0p20](examples/extra3/rho_0p20) |

## 本地编译和校验

在模板仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
cmake -S problems/T4_realtime_estimation \
      -B /tmp/pip27-t4-build \
      -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/pip27-t4-build --parallel
```

模板只提供 ROS 2 节点起始工程，不包含正式测评机。校验公开数据：

```bash
(cd problems/T4_realtime_estimation/examples && sha256sum -c SHA256SUMS)
```

正式测评由测评仓库递归编译本题 `src/` 中的全部 `.cpp`，其中必须恰有一个节点 `main()`。本题不要求保留不可修改的算法头文件；节点结构由考生自行设计。正式构建使用官方 `SignalPacket.msg`，不会执行考生的 CMake 或重新生成数据。自行拆分接口包时仍须保留相同的包名、消息名和字段定义，保证 ROS 2 类型一致。
