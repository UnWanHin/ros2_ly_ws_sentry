// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "armor_detector/armor_filter.hpp"

namespace ly_auto_aim::inline detector {

    ArmorFilter::ArmorFilter(bool is_team_red)
            : is_team_red(is_team_red) {}

    [[nodiscard]] std::uint64_t ArmorFilter::FindClosestArmor(
            const std::vector<ArmorObject>& armors,
            const cv::Point2f& target) {
        std::uint64_t min_offset_index{};
        float min_offset{99999.9f};
        for (std::uint64_t i{0}; i < armors.size(); i++) {
            const auto& points = armors[i].apex;
            const auto center = (points[0] + points[1] + points[2] + points[3]) / 4;
            const auto offset_vector = center - target;
            const auto offset = offset_vector.dot(offset_vector);
            if (offset >= min_offset) continue;

            min_offset = offset;
            min_offset_index = i;
        }

        return min_offset_index;
    }

    [[nodiscard]] float ArmorFilter::GetArmorRatio(const cv::Point2f points[4]) {
        const auto width = std::max(
                std::abs(points[0].x - points[1].x),
                std::abs(points[1].x - points[2].x)
        );

        const auto height = std::max(
                std::abs(points[0].y - points[1].y),
                std::abs(points[1].y - points[2].y)
        );

        return width / height;
    }

    [[nodiscard]] bool ArmorFilter::ShouldHitRed() const {
        return !is_team_red;
    }

    [[nodiscard]] bool ArmorFilter::ShouldHitBlue() const {
        return is_team_red;
    }

    [[nodiscard]] bool ArmorFilter::Filter(
            const std::vector<ArmorObject>& armors,
            ArmorType& target,
            std::vector<ArmorObject>& filtered_armors) const {

        filtered_armors.clear();

        for (const auto& armor : armors) {
            if (ShouldHitBlue() && armor.ActualColor() != ArmorObject::Blue) continue;
            if (ShouldHitRed() && armor.ActualColor() != ArmorObject::Red) continue;
            // if (armor.ActualColor() == ArmorObject::Gray) continue;

            static constexpr auto engineer = 2;

            const auto sqrt_size = std::sqrt(armor.area);
            // if (armor.type == engineer) {
            //     constexpr auto size_threshold = 24;
            //     if (sqrt_size < size_threshold) {
            //         continue;
            //     }
            // } else {
            //     constexpr auto size_threshold = 20;
            //     if (sqrt_size < size_threshold) {
            //         continue;
            //     }
            // }

            filtered_armors.push_back(armor);
        }

        if (filtered_armors.empty()) return false;

        static const cv::Point2f Center{640, 512};
        // target = filtered_armors[FindClosestArmor(filtered_armors, Center)];

        return true;
    }

} // namespace LangYa::ArmorDetectors