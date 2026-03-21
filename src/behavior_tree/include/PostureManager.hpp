// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <chrono>
#include <cstdint>

#include "../module/BasicTypes.hpp"
#include "PostureTypes.hpp"

namespace BehaviorTree {

class PostureManager {
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    void Configure(const LangYa::PostureSetting& setting);
    void Reset(TimePoint now, SentryPosture initial_posture = SentryPosture::Move);

    PostureDecision Tick(
        TimePoint now,
        SentryPosture desired_posture,
        std::uint8_t feedback_posture_value);

    const PostureRuntime& Runtime() const noexcept { return runtime_; }

private:
    LangYa::PostureSetting config_{};
    PostureRuntime runtime_{};

    bool initialized_{false};
    bool has_feedback_{false};
    TimePoint last_tick_{};
    TimePoint last_switch_{};
    TimePoint pending_since_{};
    TimePoint last_command_{};
    TimePoint last_feedback_{};

    void accumulate_time(double dt_seconds);
    void update_feedback(TimePoint now, std::uint8_t feedback_posture_value);
    SentryPosture choose_alternative_posture(SentryPosture avoid) const;
};

}  // namespace BehaviorTree

