// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <detector/detector.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
// #include "detector.hpp"
#include "utils/utils.h"

using namespace LY_UTILS;

namespace SOLVER
{
    struct PYD
    {
        float pitch;
        float yaw;
        float distance;
    };

    struct ArmorPose
    {
        PYD pyd;
        double theta_world;
    };

    typedef std::vector<ArmorPose> ArmorPoses;

    // struct SolutionPackage
    // {
    //     ArmorPoses armor_poses;
    //     DRIVER::SerialReadData::IMU_Flag imu_flag;
    //     long time_stamp;

    // };
} // namespace SOLVER