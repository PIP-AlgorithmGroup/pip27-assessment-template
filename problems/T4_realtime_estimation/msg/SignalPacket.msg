# 统一数据包。
# header.stamp 是测评协议中的虚拟时间戳；不要使用系统时间替换它。
# Basic：1 个值；Extra1/Extra2/Extra3：21/41/101 个值，包含当前时刻。
# 拓展数组相邻点间隔 5 ms，预测窗口分别为 0.1/0.2/0.5 s。
std_msgs/Header header
float64[] values
