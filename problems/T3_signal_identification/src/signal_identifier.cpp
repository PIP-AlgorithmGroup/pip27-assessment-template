#include "signal_identifier.hpp"

#include <stdexcept>

// 可编译的起始模板；请在 Impl 中定义算法状态或历史观测存储。
struct SignalIdentifier::Impl {
    Impl(SignalMode mode, double noise_stddev) : mode(mode), noise_stddev(noise_stddev) {}

    SignalMode mode;
    double noise_stddev;
};

SignalIdentifier::SignalIdentifier(SignalMode mode, double noise_stddev)
    : impl_(std::make_unique<Impl>(mode, noise_stddev)) {
    // TODO: 指定信号类型和噪声标准差

}

SignalIdentifier::~SignalIdentifier() = default;

void SignalIdentifier::update(double, double) {
    // TODO: 请接收 timestamp、observation，更新状态或保存观测。
}

ParameterEstimate SignalIdentifier::estimate() const {
    // TODO: 请按 mode 的参数顺序返回最终估计；初始模板明确报告尚未实现。
    throw std::logic_error("SignalIdentifier::estimate 尚未实现");
}
