// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"

#include <chrono>

namespace BehaviorTree {

namespace {
using namespace std::chrono_literals;
constexpr auto kWatchdogPeriod = 100ms;
constexpr std::int64_t kNanoPerMilli = 1000000LL;
}  // namespace

std::int64_t Application::NowSteadyNs() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

const char* Application::RuntimeFaultCodeToString(const RuntimeFaultCode code) noexcept {
    switch (code) {
        case RuntimeFaultCode::None: return "none";
        case RuntimeFaultCode::LoopStall: return "loop_stall";
        case RuntimeFaultCode::TickStall: return "tick_stall";
        case RuntimeFaultCode::TreeEmpty: return "tree_empty";
        case RuntimeFaultCode::TreeException: return "tree_exception";
        default: return "unknown";
    }
}

void Application::MarkLoopBeat() noexcept {
    runtimeLastLoopBeatNs_.store(NowSteadyNs(), std::memory_order_relaxed);
}

void Application::MarkTickStart() noexcept {
    runtimeTickInProgress_.store(true, std::memory_order_relaxed);
    runtimeTickStartNs_.store(NowSteadyNs(), std::memory_order_relaxed);
}

void Application::MarkTickEnd() noexcept {
    const auto now_ns = NowSteadyNs();
    runtimeTickEndNs_.store(now_ns, std::memory_order_relaxed);
    runtimeTickInProgress_.store(false, std::memory_order_relaxed);
}

void Application::RequestSoftRecovery(const RuntimeFaultCode code) noexcept {
    if (code == RuntimeFaultCode::None) return;

    bool expected = false;
    if (runtimeRecoveryRequested_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel, std::memory_order_relaxed)) {
        runtimeFaultCode_.store(code, std::memory_order_release);
    }
}

bool Application::IsCriticalInputStale() const {
    if (!hasReceivedGimbalAngles_.load(std::memory_order_relaxed)) {
        return true;
    }
    if (lastGimbalAnglesRxTime.time_since_epoch().count() == 0) {
        return true;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastGimbalAnglesRxTime).count();
    return dt_ms > kRuntimeGimbalStaleMs;
}

void Application::PublishSafeControl(const char* reason, const bool from_guard_thread) noexcept {
    try {
        const auto now_ns = NowSteadyNs();
        const auto last_ns = runtimeLastSafePublishNs_.load(std::memory_order_relaxed);
        if (last_ns > 0 &&
            (now_ns - last_ns) < static_cast<std::int64_t>(kRuntimeSafePublishMinIntervalMs) * kNanoPerMilli) {
            return;
        }
        runtimeLastSafePublishNs_.store(now_ns, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(runtimeRecoveryMutex_);

        gimbalControlData.GimbalAngles = gimbalAngles;
        gimbalControlData.FireCode.FireStatus = 0;
        gimbalControlData.FireCode.Rotate = 0;
        gimbalControlData.FireCode.AimMode = 0;
        postureCommand = 0;
        naviVelocityInput = VelocityType{0, 0};
        naviVelocity = VelocityType{0, 0};

        if (pub_gimbal_control_) {
            gimbal_driver::msg::GimbalAngles angle_msg;
            angle_msg.yaw = gimbalControlData.GimbalAngles.Yaw;
            angle_msg.pitch = gimbalControlData.GimbalAngles.Pitch;
            if (node_) {
                angle_msg.header.stamp = node_->now();
            }
            pub_gimbal_control_->publish(angle_msg);
        }

        if (pub_gimbal_firecode_) {
            std_msgs::msg::UInt8 fire_msg;
            fire_msg.data = *reinterpret_cast<std::uint8_t*>(&gimbalControlData.FireCode);
            pub_gimbal_firecode_->publish(fire_msg);
        }

        if (pub_gimbal_vel_) {
            gimbal_driver::msg::Vel vel_msg;
            vel_msg.x = 0;
            vel_msg.y = 0;
            pub_gimbal_vel_->publish(vel_msg);
        }

        if (pub_navi_vel_) {
            gimbal_driver::msg::Vel vel_msg;
            vel_msg.x = 0;
            vel_msg.y = 0;
            pub_navi_vel_->publish(vel_msg);
        }

        if (LoggerPtr && !from_guard_thread) {
            LoggerPtr->Warning("Runtime safe-control published, reason={}", reason ? reason : "unknown");
        }
    } catch (...) {
    }
}

void Application::RuntimeGuardLoop() {
    while (!runtimeGuardStop_.load(std::memory_order_relaxed) && rclcpp::ok()) {
        const auto now_ns = NowSteadyNs();

        const auto last_loop_ns = runtimeLastLoopBeatNs_.load(std::memory_order_relaxed);
        if (last_loop_ns > 0 &&
            (now_ns - last_loop_ns) > static_cast<std::int64_t>(kRuntimeLoopStallMs) * kNanoPerMilli) {
            RequestSoftRecovery(RuntimeFaultCode::LoopStall);
        }

        if (runtimeTickInProgress_.load(std::memory_order_relaxed)) {
            const auto tick_start_ns = runtimeTickStartNs_.load(std::memory_order_relaxed);
            if (tick_start_ns > 0 &&
                (now_ns - tick_start_ns) > static_cast<std::int64_t>(kRuntimeTickStallMs) * kNanoPerMilli) {
                RequestSoftRecovery(RuntimeFaultCode::TickStall);
            }
        }

        std::this_thread::sleep_for(kWatchdogPeriod);
    }
}

void Application::StartRuntimeGuard() {
    if (runtimeGuardThread_.joinable()) return;

    runtimeGuardStop_.store(false, std::memory_order_relaxed);
    runtimeRecoveryRequested_.store(false, std::memory_order_relaxed);
    runtimeFaultCode_.store(RuntimeFaultCode::None, std::memory_order_relaxed);
    runtimeTickInProgress_.store(false, std::memory_order_relaxed);
    const auto now_ns = NowSteadyNs();
    runtimeLastLoopBeatNs_.store(now_ns, std::memory_order_relaxed);
    runtimeTickStartNs_.store(now_ns, std::memory_order_relaxed);
    runtimeTickEndNs_.store(now_ns, std::memory_order_relaxed);
    runtimeRecoveryWindowStart_ = std::chrono::steady_clock::now();
    runtimeRecoveryCountInWindow_ = 0;

    runtimeGuardThread_ = std::thread([this]() { RuntimeGuardLoop(); });
    if (LoggerPtr) {
        LoggerPtr->Info("Runtime guard started. loop_stall={}ms tick_stall={}ms",
                        kRuntimeLoopStallMs, kRuntimeTickStallMs);
    }
}

void Application::StopRuntimeGuard() {
    runtimeGuardStop_.store(true, std::memory_order_relaxed);
    if (runtimeGuardThread_.joinable()) {
        runtimeGuardThread_.join();
    }
}

bool Application::TrySoftReloadBehaviorTree(const RuntimeFaultCode code) {
    const auto now = std::chrono::steady_clock::now();
    if (runtimeLastSoftRecoverTime_.time_since_epoch().count() != 0) {
        const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - runtimeLastSoftRecoverTime_).count();
        if (dt < kRuntimeRecoveryMinIntervalMs) {
            if (LoggerPtr) {
                LoggerPtr->Warning("Soft-reload skipped by min interval: {} ms", dt);
            }
            return false;
        }
    }

    if (now - runtimeRecoveryWindowStart_ > std::chrono::seconds(kRuntimeRecoveryWindowSec)) {
        runtimeRecoveryWindowStart_ = now;
        runtimeRecoveryCountInWindow_ = 0;
    }

    if (runtimeRecoveryCountInWindow_ >= kRuntimeRecoveryLimit) {
        if (LoggerPtr) {
            LoggerPtr->Error(
                "Soft-reload blocked: reach limit {}/{}s, keep safe mode.",
                kRuntimeRecoveryLimit,
                kRuntimeRecoveryWindowSec);
        }
        return false;
    }

    if (LoggerPtr) {
        LoggerPtr->Warning("Soft-reload begin, reason={}", RuntimeFaultCodeToString(code));
    }

    try {
        btGrootPublisher_.reset();
        btFileLogger_.reset();
        BTree = BT::Tree{};

        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }
        ResetTickBlackboard();
        GlobalBlackboard_->set("TickBlackboard", TickBlackboard_);

        if (!LoadBehaviorTree()) {
            if (LoggerPtr) {
                LoggerPtr->Error("Soft-reload failed: LoadBehaviorTree returned false.");
            }
            return false;
        }

        isFindTargetAtomic.store(false, std::memory_order_relaxed);
        postureCommand = 0;
        lastFoundEnemyTime = now;
        runtimeLastSoftRecoverTime_ = now;
        runtimeRecoveryCountInWindow_++;

        if (LoggerPtr) {
            LoggerPtr->Warning("Soft-reload success, reason={}, count={}",
                               RuntimeFaultCodeToString(code),
                               runtimeRecoveryCountInWindow_);
        }
        return true;
    } catch (const std::exception& ex) {
        if (LoggerPtr) {
            LoggerPtr->Error("Soft-reload exception: {}", ex.what());
        }
        return false;
    } catch (...) {
        if (LoggerPtr) {
            LoggerPtr->Error("Soft-reload exception: <unknown>");
        }
        return false;
    }
}

bool Application::TryHandleSoftRecovery() {
    if (!runtimeRecoveryRequested_.load(std::memory_order_acquire)) {
        return false;
    }

    RuntimeFaultCode code = runtimeFaultCode_.load(std::memory_order_acquire);
    if (code == RuntimeFaultCode::None) {
        code = RuntimeFaultCode::TreeException;
    }

    {
        std::lock_guard<std::mutex> lock(runtimeRecoveryMutex_);
        if (runtimeRecovering_) {
            return true;
        }
        runtimeRecovering_ = true;
    }

    PublishSafeControl(RuntimeFaultCodeToString(code), false);
    const bool recovered = TrySoftReloadBehaviorTree(code);
    if (recovered) {
        runtimeRecoveryRequested_.store(false, std::memory_order_release);
        runtimeFaultCode_.store(RuntimeFaultCode::None, std::memory_order_release);
    } else {
        runtimeRecoveryRequested_.store(false, std::memory_order_release);
    }

    {
        std::lock_guard<std::mutex> lock(runtimeRecoveryMutex_);
        runtimeRecovering_ = false;
    }
    return true;
}

}  // namespace BehaviorTree
