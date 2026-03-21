// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <cstdint>

namespace ly_auto_aim::inline controller
{
    enum class ControlInvalidReason : std::uint8_t
    {
        None = 0,
        NoPrediction = 1,
        InvalidCar = 2,
        InvalidCarAfterFlyTime = 3,
        NoArmorFallbackBallisticFail = 4,
        InvalidArmor = 5,
        ArmorBallisticFail = 6,
        UnstableTrack = 7
    };

    struct ControlResult
    {
        uint8_t shoot_flag;  /// depprecated in sentry
        float pitch_setpoint;
        float yaw_setpoint;
        float pitch_actual_want;
        float yaw_actual_want;
        bool valid=true;  /// deprecated in sentry
        ControlInvalidReason invalid_reason{ControlInvalidReason::None};
    };
}
