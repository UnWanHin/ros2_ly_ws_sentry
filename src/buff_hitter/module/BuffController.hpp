// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <cstdint>
#include <functional>
#include "driver/type.hpp"
#include "predictor/type.hpp"
#include "solver/type.hpp"

namespace power_rune {
    using driver::ParsedSerialData;
    struct BuffControlResult {
        uint8_t shoot_flag;
        float pitch_setpoint;
        float yaw_setpoint;
        float pitch_actual_want;
        float yaw_actual_want;
        bool valid = true;
    };

    class BuffController {
        public:
            BuffController();
            ~BuffController();

            BuffControlResult buff_control(const ParsedSerialData& parsedData, std::shared_ptr<float> buff_pitch, std::shared_ptr<float> buff_yaw) const;
        };

}