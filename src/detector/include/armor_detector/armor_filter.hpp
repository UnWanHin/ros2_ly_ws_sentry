// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <auto_aim_common/DetectionType.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <fmt/format.h>

namespace ly_auto_aim::inline detector {

    class ArmorFilter final {
    public:
        bool is_team_red;
        ArmorFilter(bool is_team_red = true);

        [[nodiscard]] static float GetArmorRatio(const cv::Point2f points[4]);

        [[nodiscard]] bool ShouldHitRed() const;
        [[nodiscard]] bool ShouldHitBlue() const;

        [[nodiscard]] bool Filter(
                const std::vector<ArmorObject>& armors,
                ArmorType& target,
                std::vector<ArmorObject>& filtered_armors) const;

    private:
        [[nodiscard]] static std::uint64_t FindClosestArmor(
                const std::vector<ArmorObject>& armors,
                const cv::Point2f& target);


        std::vector<int> BalanceUnitIDList;
    };

} // namespace LangYa::ArmorDetectors