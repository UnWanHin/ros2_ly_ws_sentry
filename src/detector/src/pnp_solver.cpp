// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include <sstream>
#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
//#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <iostream>
#include <fmt/color.h>
#include "armor_detector/pnp_solver.hpp"


namespace ly_auto_aim::inline pnp_solver{
    inline constexpr auto SmallArmorHalfWidth = 0.0675f;
    inline constexpr auto SmallArmorHalfHeight = 0.0275f;
    inline constexpr auto SmallArmorWidthRatio = 1.0f;
    inline constexpr auto SmallArmorHeightRatio = 1.0f;
    inline constexpr auto SAHW = SmallArmorHalfWidth * SmallArmorWidthRatio;
    inline constexpr auto SAHH = SmallArmorHalfHeight * SmallArmorHeightRatio;
    const std::vector SmallArmorPoints =	// 装甲板放在地上
            {
                cv::Point3f(-SAHW, -SAHH, 0.0f),
                cv::Point3f(SAHW, -SAHH, 0.0f),
                cv::Point3f(SAHW, SAHH, 0.0f),
                cv::Point3f(-SAHW, SAHH, 0.0f)
            };
    /*const std::vector SmallArmorPoints =	// 装甲板侧着放置
    {
        cv::Point3f(-SAHW, 0.0f, SAHH),
        cv::Point3f(-SAHW, 0.0f, -SAHH),
        cv::Point3f(SAHW, 0.0f, -SAHH),
        cv::Point3f(SAHW, 0.0f, SAHH)
    };*/
    /*const std::vector OPSmallArmorPoints =	// 装甲板正着放置
    {
        cv::Point3f(-SAHW, -SAHH, 0.0f),
        cv::Point3f(SAHW, -SAHH, 0.0f),
        cv::Point3f(SAHW, SAHH, 0.0f),
        cv::Point3f(-SAHW, SAHH, 0.0f)
    };*/


    inline constexpr auto LargeArmorHalfWidth = 0.116f;
    inline constexpr auto LargeArmorHalfHeight = 0.0275f;
    inline constexpr auto LargeArmorWidthRatio = 0.87f;
    inline constexpr auto LargeArmorHeightRatio = 1.0f;
    inline constexpr auto LAHW = LargeArmorHalfWidth * LargeArmorWidthRatio;
    inline constexpr auto LAHH = LargeArmorHalfHeight * LargeArmorHeightRatio;
    inline constexpr auto LargeArmorXRatio = 0.87f;
    const std::vector LargeArmorPoints =
            {
                cv::Point3f(-LAHW, -LAHH, 0.0f),
                cv::Point3f(LAHW, -LAHH, 0.0f),
                cv::Point3f(LAHW, LAHH, 0.0f),
                cv::Point3f(-LAHW, LAHH, 0.0f)
            };

    void CameraIntrinsicsParameterPack::GetCameraMatrix(cv::Mat& matrix) const
    {
        matrix = (cv::Mat_<float>(3, 3) << // NOLINT(modernize-return-braced-init-list)
                                        FocalLength[0], 0, PrincipalPoint[0],
                0, FocalLength[1], PrincipalPoint[1],
                0, 0, 1);
    }

    void CameraIntrinsicsParameterPack::GetDistortionCoefficients(cv::Mat& matrix) const
    {
        matrix = (cv::Mat_<float>(5, 1) << // NOLINT(modernize-return-braced-init-list)
                                        RadialDistortion[0],
                RadialDistortion[1],
                TangentialDistortion[0],
                TangentialDistortion[1],
                RadialDistortion[2]);
    }

    bool PoseSolver::SolveLargeArmor(const std::vector<cv::Point2f>& orderedPoints, ArmorTransform& transform)
    {
        return SolveArmor(orderedPoints, LargeArmorPoints, transform);
    }

    bool PoseSolver::SolveSmallArmor(const std::vector<cv::Point2f>& orderedPoints, ArmorTransform& transform)
    {
        return SolveArmor(orderedPoints, SmallArmorPoints, transform);
    }

    /*bool PoseSolver::SolveOutpostArmor(
            const std::vector<cv::Point2f>& orderedPoints,
            ArmorTransform& transform) {
        return SolveArmor(orderedPoints, OPSmallArmorPoints, transform);
    }*/

    bool PoseSolver::SolveArmor(
            const std::vector<cv::Point2f>& orderedPicturePoints,
            const std::vector<cv::Point3f>& worldPoints,
            ArmorTransform& transform)
    {
        if (!solvePnP(
                worldPoints,
                orderedPicturePoints,
                CameraMatrix,
                DistortionCoefficients,
                transform.Rotation,
                transform.Translation,
                false,
                cv::SOLVEPNP_ITERATIVE)
                )
        {
            std::stringstream stream{};
            stream << "picture points: ";
            for (const auto& point: orderedPicturePoints) stream << point;
            stream << "; world points: ";
            for (const auto& point: worldPoints) stream << point;
//            spdlog::error("PoseSolver> cannot solve armor with points({})", stream.str());
            fmt::print(fg(fmt::color::crimson), "PoseSolver> cannot solve armor with points({})", stream.str());
            return false;
        }
        return true;
    }

    bool PoseSolver::SolveArmor(
            const std::vector<cv::Point2f>& orderedPoints,
            ArmorTransform& transform,
            const bool isLarge)
    {
        return isLarge ? SolveLargeArmor(orderedPoints, transform) : SolveSmallArmor(orderedPoints, transform);
    }

    PoseSolver::PoseSolver(const CameraIntrinsicsParameterPack& parameter)
    {
        parameter.GetCameraMatrix(CameraMatrix);
        parameter.GetDistortionCoefficients(DistortionCoefficients);
    }
}
