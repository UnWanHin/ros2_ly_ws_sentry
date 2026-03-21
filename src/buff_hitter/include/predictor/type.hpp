// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <Eigen/Core>
#include <vector>
#include <array>
#include <Location/location.hpp>

namespace predictor
{
    struct Armor {
        enum armor_status {
            NONEXIST,//temply not used
            UNSEEN,
            AVAILABLE
        };
        XYZ center;
        double yaw;
        double theta;// like yaw
        int id;
        armor_status status = NONEXIST;
    };

    struct Prediction {
        XYZ center;
        int id;
        double vx, vy;
        double z1, z2;
        double theta, omega;
        double r1, r2;
        std::array<Armor, 4> armors;
        bool stable = true;
    };

    typedef std::vector<Prediction> Predictions;
};