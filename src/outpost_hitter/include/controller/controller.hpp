// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

// #include <driver.hpp>
#include <predictor/predictor.hpp>
#include <utils/utils.h>

namespace CONTROLLER
{
    struct BoardInformation
    {
        double aim_yaw;
        double aim_pitch;
        double aim_distance;
        double face_cos;
    };

    typedef std::vector<BoardInformation> BoardInformations;

} // namespace CONTROLLER