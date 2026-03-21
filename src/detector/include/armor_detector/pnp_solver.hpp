// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <ranges>
#include <span>
//#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <fmt/color.h>
//#include "Gimbals.hpp"
#include <vector>
#include <opencv2/core.hpp>

/// 暂时弃用
#pragma region pnp_aim_result
namespace ly_auto_aim::inline pnp_solver
{
template <typename T>
bool Contains(const T value, const T min, const T max)
{
    return min <= value && value < max;
}

/// @brief 从PNP的相对位置计算出 yaw pitch
struct PNPAimResult
{
    using RadianType = float;
    using AngleType = float;

    float Distance{};
    AngleType DeltaYaw{};
    AngleType DeltaPitch{};

    /// @brief 从相对三维位置计算角度
    bool FromTranslation(const std::span<const float> relative)
    {
        // 此处距离的单位都是 m

        auto x = relative[0]; // 相对左右距离 相机面向对象 左负右正
        auto y = relative[1]; // 相对上下距离 相机面向对象 下负上正
        auto z = relative[2]; // 相对前后距离 相机面向对象 后负前正

        constexpr std::ranges::min_max_result<float> x_reasonable_range{-5, 5};
        constexpr std::ranges::min_max_result<float> y_reasonable_range{-5, 5};
        constexpr std::ranges::min_max_result<float> z_reasonable_range{0, 30};

        if (!Contains(x, x_reasonable_range.min, x_reasonable_range.max)
            || !Contains(y, y_reasonable_range.min, y_reasonable_range.max)
            || !Contains(z, z_reasonable_range.min, z_reasonable_range.max))
        {
//          spdlog::warn("PNPAimResult> relative translation value not reasonable!");
            fmt::print(fg(fmt::color::yellow), "PNPAimResult> relative translation value not reasonable!\n");
            return false;
        }

        // 将 pnp 的结果从相机坐标系转换到云台坐标系
        constexpr auto z_offset = 0.0496f;
        constexpr auto x_offset = 0.005f;

        z += z_offset;
        x += x_offset;

        // 计算得到水平角和竖直角的相对值
        Distance = std::sqrt(x * x + z * z);

        DeltaYaw = AngleType{RadianType{std::atan2(-x, z)}};
        DeltaPitch = AngleType{RadianType{std::atan2(-y, Distance)}};

        return true;
    }
};
}
#pragma endregion pnp_aim_result

#pragma region pose_solver
namespace ly_auto_aim::inline pnp_solver {
    struct CameraIntrinsicsParameterPack
    {
        float FocalLength[2]{1331.1f, 1330.1f};
        float PrincipalPoint[2]{624.5817f, 526.3662f};
        float RadialDistortion[3]{-0.1059f, -0.3427f, 1.4125f};
        float TangentialDistortion[2]{0.0072f, 0};

        void GetCameraMatrix(cv::Mat& matrix) const;

        void GetDistortionCoefficients(cv::Mat& matrix) const;
    };

    struct ArmorTransform
    {
        std::vector<float> Rotation{};
        std::vector<float> Translation{};
    };

    struct PoseSolver
    {
        bool SolveLargeArmor(
                const std::vector<cv::Point2f>& orderedPoints,
                ArmorTransform& transform);
        bool SolveSmallArmor(
                const std::vector<cv::Point2f>& orderedPoints,
                ArmorTransform& transform);

        /*bool SolveOutpostArmor(
            const std::vector<cv::Point2f>& orderedPoints,
            ArmorTransform& transform
        );*/

        bool SolveArmor(
                const std::vector<cv::Point2f>& orderedPicturePoints,
                const std::vector<cv::Point3f>& worldPoints,
                ArmorTransform& transform);

        bool SolveArmor(const std::vector<cv::Point2f>& orderedPoints, ArmorTransform& transform, const bool isLarge);

        explicit PoseSolver(const CameraIntrinsicsParameterPack& parameter);

    private:
        cv::Mat CameraMatrix{};
        cv::Mat DistortionCoefficients{};
        cv::Mat RVec{};
        cv::Mat TVec{};
    };
}
#pragma endregion pose_solver
