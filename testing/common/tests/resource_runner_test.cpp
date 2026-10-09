#include <resource_runner.hpp>

#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <new>
#include <thread>

#include <sched.h>

namespace {

using namespace std::chrono_literals;
using assessment::Limits;
using assessment::Verdict;

constexpr Limits kLimits{50ms, 64ULL * 1024 * 1024, 1, 1};

void expect(bool condition, const char* scenario) {
    if (!condition) {
        std::cerr << "Failed: " << scenario << '\n';
        std::exit(1);
    }
}

}  // namespace

int main() {
    cpu_set_t parent_allowed;
    expect(sched_getaffinity(0, sizeof(parent_allowed), &parent_allowed) == 0,
           "parent CPU affinity is readable");
    const auto passed = assessment::runCase(kLimits, [] { return true; });
    expect(passed.verdict == Verdict::Passed, "correct answer passes");
    expect(passed.elapsed_time.count() > 0 &&
               passed.peak_rss_kib.has_value() && *passed.peak_rss_kib > 0,
           "successful case reports runtime and peak RSS");
    expect(assessment::runCase(
               kLimits,
               [&] {
                   cpu_set_t allowed;
                   return sched_getaffinity(0, sizeof(allowed), &allowed) == 0 &&
                          CPU_COUNT(&allowed) == 1 &&
                          sched_setaffinity(0, sizeof(parent_allowed),
                                            &parent_allowed) == -1 &&
                          errno == EPERM;
               }).verdict == Verdict::Passed,
           "candidate process is limited to one CPU and cannot widen affinity");
    expect(assessment::runCase(kLimits, [] { return false; }).verdict ==
               Verdict::WrongAnswer,
           "wrong answer fails");

    const auto timed_out = assessment::runCase(
        kLimits,
        [] {
            std::this_thread::sleep_for(150ms);
            return true;
        });
    expect(timed_out.verdict == Verdict::TimeLimit, "test case times out");
    expect(timed_out.elapsed_time >= 50ms &&
               timed_out.peak_rss_kib.has_value() &&
               *timed_out.peak_rss_kib > 0,
           "timed-out case reports runtime and peak RSS");

    expect(assessment::runCase(
               kLimits,
               [] {
                   auto* memory = new char[128ULL * 1024 * 1024];
                   // volatile 访问防止优化构建把整个分配及读写删除。
                   volatile char* observed = memory;
                   observed[0] = 1;
                   const bool allocated = observed[0] == 1;
                   delete[] memory;
                   return allocated;
               }).verdict == Verdict::MemoryLimit,
           "address-space limit rejects allocation");

    expect(assessment::runCase(
               kLimits,
               [] {
                   std::raise(SIGSEGV);
                   return true;
               }).verdict == Verdict::RuntimeError,
           "crash is reported");

    // CPU 时间限制与墙钟运行时间限制分别生效。
    const Limits cpu_case{4000ms, kLimits.address_space_bytes, 1, 1};
    expect(assessment::runCase(
               cpu_case,
               [] {
                   volatile std::uint64_t spins = 0;
                   while (true) {
                       ++spins;
                   }
                   return true;
               }).verdict == Verdict::CpuLimit,
           "CPU time limit is reported");

    expect(assessment::runCase(
               kLimits,
               [] {
                   std::_Exit(0);
                   return true;
               }).verdict == Verdict::RuntimeError,
           "bare zero exit is not accepted as a passed test");

    const Limits invalid{0ms, kLimits.address_space_bytes, 1, 1};
    expect(assessment::runCase(invalid, [] { return true; }).verdict ==
               Verdict::SetupError,
           "invalid limits fail before executing candidate code");
    const Limits invalid_cpu_count{50ms, kLimits.address_space_bytes, 1, 0};
    expect(assessment::runCase(invalid_cpu_count, [] { return true; }).verdict ==
               Verdict::SetupError,
           "zero available CPUs is rejected");
}
