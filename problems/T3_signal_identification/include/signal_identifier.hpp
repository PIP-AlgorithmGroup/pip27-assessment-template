#pragma once

#include <array>
#include <cstdint>
#include <memory>

// T3 固定接口：时间戳的单位为虚拟秒，参数相对于第一帧定义。
enum class SignalMode : std::uint8_t {
    Constant,
    Linear,
    Quadratic,
    Sine,
    QuadraticSine,
};

struct ParameterEstimate {
    // Constant: [c,0,0,0,0,0]；Linear: [c,v,0,0,0,0]。
    // Quadratic: [c,v,a,0,0,0]，二次项为 0.5*a*tau^2。
    // Sine: [b,A,f,phi,0,0]；QuadraticSine: [c,v,a,A,f,phi]。
    std::array<double, 6> parameters{};
};

class SignalIdentifier {
public:
    // 一个对象对应一种固定模式与一条完整观测序列；noise_stddev 为有限非负的 sigma。
    SignalIdentifier(SignalMode mode, double noise_stddev);
    ~SignalIdentifier();

    // 输入有限的时间戳与含噪观测量；时间戳严格递增。
    void update(double timestamp, double observation);

    // 完整非空序列结束后调用一次；未使用项置零，A/f>0，phi 属于 [-pi,pi)。
    [[nodiscard]] ParameterEstimate estimate() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
