// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <Eigen/Core>
#include <vector>
#include <array>
#include "auto_aim_common/Location.hpp"

namespace Eigen {
    // define Eigen5d
    typedef Eigen::Matrix<double, 5, 1> Vector5d;
}
// 前向声明 driver::ParsedSerialData
// namespace driver {
//     struct ParsedSerialData;
// }

namespace ly_auto_aim::inline solver
{
    /// ImuData这个接口和之前的GimbalAngleType不太一样，先暂时使用ImuData这个名称
    // struct ImuData
    // {
    //     float pitch;
    //     float yaw;
    //     float roll;
    //     ImuData() {};
    //     ImuData(const driver::ParsedSerialData& x);
    //     operator PYD() const
    //     {
    //         return PYD(pitch * M_PI / 180, yaw * M_PI / 180, 0);
    //     };
    // };

    struct GimbalAngleType{
        float pitch;
        float yaw;

        GimbalAngleType() {};
        GimbalAngleType(const float& Pitch, const float& Yaw) : yaw(Yaw), pitch(Pitch) {}
        
        operator PYD() const {
            return PYD(pitch * M_PI / 180, yaw * M_PI / 180, 0);
        }
    };
    
}