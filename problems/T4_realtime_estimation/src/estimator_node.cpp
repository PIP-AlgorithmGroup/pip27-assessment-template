#include "t4_realtime_estimation/msg/signal_packet.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

namespace signal_estimator {

using Packet = t4_realtime_estimation::msg::SignalPacket;

constexpr std::int64_t kBasicMode = 0;
constexpr std::int64_t kExtra1Mode = 1;
constexpr std::int64_t kExtra2Mode = 2;
constexpr std::int64_t kExtra3Mode = 3;
// 包含当前时刻；相邻点间隔 5 ms，窗口分别为 0.1 s、0.2 s、0.5 s。
constexpr std::size_t kExtra1PredictionPointCount = 21;
constexpr std::size_t kExtra2PredictionPointCount = 41;
constexpr std::size_t kExtra3PredictionPointCount = 101;
constexpr std::int64_t kPredictionStepNs = 5'000'000;

constexpr char kNodeName[] = "signal_estimator";
constexpr char kModeParameter[] = "assessment_mode";
constexpr char kObservationTopic[] = "/signal/observation";
constexpr char kEstimateTopic[] = "/signal/estimate";
constexpr char kPredictionTopic[] = "/signal/prediction";

struct Sample {
    std::int64_t stamp_ns = 0;
    double value = 0.0;
};

// ================================================================
// 算法接口
// ================================================================
class OnlineEstimator {
public:
    void reset(std::int64_t mode) {
        mode_ = mode;
        history_.clear();
    }

    // TODO: 保存一帧观测并更新在线估计状态。
    void addObservation(std::int64_t stamp_ns, double value) {
        history_.push_back({stamp_ns, value});
        if (history_.size() > kHistoryLimit) {
            history_.pop_front();
        }
    }

    // TODO: 返回当前虚拟时刻的去噪估计值。
    [[nodiscard]] double estimateCurrent() const {
        return history_.empty() ? 0.0 : history_.back().value;
    }

    // TODO: 从首帧虚拟时间开始，每个发布 tick 将预测起点推进 5 ms。
    // 以下返回值只是占位，拓展不能始终使用最近观测的时间戳。
    [[nodiscard]] std::int64_t predictionStartNanoseconds() const {
        return history_.empty() ? 0 : history_.back().stamp_ns;
    }

    // TODO: 返回起点以及当前模式要求的未来预测值。
    // point_count 按模式选择上述点数常量，包含当前时刻。
    // values[i] 对应 prediction_start_ns + i * 5 ms。
    [[nodiscard]] std::vector<double> predict(std::int64_t /*prediction_start_ns*/,
                                              std::size_t point_count) const {
        return std::vector<double>(point_count);
    }

private:
    static constexpr std::size_t kHistoryLimit = 256;

    std::int64_t mode_ = -1;
    std::deque<Sample> history_;
};

// ================================================================
// ROS 2 节点回调
// ================================================================
class EstimatorNode final : public rclcpp::Node {
public:
    EstimatorNode() : Node(kNodeName) {
        mode_ = declare_parameter<std::int64_t>(kModeParameter, -1);

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(256)).reliable().durability_volatile();

        observation_subscription_ = create_subscription<Packet>(
            kObservationTopic, qos,
            std::bind(&EstimatorNode::onObservation, this, std::placeholders::_1));
        estimate_publisher_ = create_publisher<Packet>(kEstimateTopic, qos);
        prediction_publisher_ = create_publisher<Packet>(kPredictionTopic, qos);

        // 测评目标周期是 5 ms；系统调度抖动由测评机测量。
        prediction_timer_ = create_wall_timer(std::chrono::milliseconds(5),
                                              std::bind(&EstimatorNode::onPredictionTimer, this));

        parameter_callback_ = add_on_set_parameters_callback(
            std::bind(&EstimatorNode::onSetParameters, this, std::placeholders::_1));
    }

private:
    // TODO: 实现观测接收逻辑。
    // 至少需要处理：消息合法性、Header 虚拟时间戳、观测顺序、
    // estimator_.addObservation()，以及 Basic 模式的一帧一响应。
    void onObservation(const Packet::ConstSharedPtr& message) {
        (void)message;
    }

    // TODO: 实现 200 Hz 预测发布逻辑。
    // 按当前模式选择 21、41 或 101 个值，包含当前值，
    // 起点为首帧虚拟时间 + tick * 5 ms，不能用系统墙钟替换 Header。
    void onPredictionTimer() {}

    // TODO: 实现 assessment_mode 参数更新和合法性检查（0、1、2、3）。
    // 参数通过 /signal_estimator/set_parameters_atomically 设置。
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& parameters) {
        (void)parameters;

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "not implemented";
        return result;
    }

    std::int64_t mode_ = -1;
    OnlineEstimator estimator_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    rclcpp::Subscription<Packet>::SharedPtr observation_subscription_;
    rclcpp::Publisher<Packet>::SharedPtr estimate_publisher_;
    rclcpp::Publisher<Packet>::SharedPtr prediction_publisher_;
    rclcpp::TimerBase::SharedPtr prediction_timer_;
};

}  // namespace signal_estimator

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<signal_estimator::EstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
