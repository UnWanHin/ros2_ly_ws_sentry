// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <array>
#include <cstdint>

namespace BehaviorTree {

enum class SentryPosture : std::uint8_t {
    Unknown = 0,
    Attack = 1,
    Defense = 2,
    Move = 3
};

inline constexpr bool IsValidPosture(const SentryPosture posture) noexcept {
    return posture == SentryPosture::Attack ||
           posture == SentryPosture::Defense ||
           posture == SentryPosture::Move;
}

inline constexpr bool IsValidPostureValue(const std::uint8_t posture) noexcept {
    return posture >= static_cast<std::uint8_t>(SentryPosture::Attack) &&
           posture <= static_cast<std::uint8_t>(SentryPosture::Move);
}

inline constexpr SentryPosture ToPosture(const std::uint8_t posture) noexcept {
    return IsValidPostureValue(posture) ? static_cast<SentryPosture>(posture) : SentryPosture::Unknown;
}

inline constexpr std::uint8_t ToPostureValue(const SentryPosture posture) noexcept {
    return IsValidPosture(posture) ? static_cast<std::uint8_t>(posture) : 0U;
}

inline constexpr const char* PostureToString(const SentryPosture posture) noexcept {
    switch (posture) {
        case SentryPosture::Attack: return "Attack";
        case SentryPosture::Defense: return "Defense";
        case SentryPosture::Move: return "Move";
        default: return "Unknown";
    }
}

struct PostureDecision {
    std::uint8_t Command{0}; // 0 = this tick does not send command
    bool Sent{false};
    const char* Reason{"hold"};
};

struct PostureRuntime {
    SentryPosture Current{SentryPosture::Move};
    SentryPosture Desired{SentryPosture::Move};
    SentryPosture Pending{SentryPosture::Unknown};
    std::array<double, 4> AccumSec{};  // index = posture value(1..3)
    std::array<bool, 4> Degraded{};    // index = posture value(1..3)
    bool HasPending{false};
    bool FeedbackStale{false};
    int RetryCount{0};
};

}  // namespace BehaviorTree

