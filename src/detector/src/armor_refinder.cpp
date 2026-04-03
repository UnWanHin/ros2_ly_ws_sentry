// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "armor_detector/armor_refinder.hpp"



namespace ly_auto_aim::inline detector {
    
    bool ArmorRefinder::ReFindAndSolveAll(PoseSolver& solver,
                                          const std::vector<ArmorObject>& armors,
                                          const ArmorType& target,
                                          ArmorObject& target_armor,
                                          auto_aim_common::msg::Armors& armors_msg,
                                          auto_aim_common::msg::Armors* high_armors_msg,
                                          float high_height_threshold_m) noexcept{
        if (armors.empty()) return false;

        min_offset = FLT_MAX;
        min_index = 100; // 表示一个不存在的下标
        constexpr auto thread_hold = 100;
        auto_aim_common::msg::Armor temp_armor_msg;
        armors_msg.armors.clear();
        if (high_armors_msg) {
            high_armors_msg->armors.clear();
            high_armors_msg->is_available_armor_for_predictor = false;
            high_armors_msg->target_armor_index_for_predictor = -1;
        }
        std::vector<cv::Point2f> pnp_points{};


        // 遍历armors，找到唯一符合条件的下标
        std::uint64_t selected_source_index = 0;
        for(std::uint64_t i{0}; i < armors.size(); ++i){
            const auto& armor = armors[i];
            pnp_points.clear();

            temp_armor_msg.corners_x.clear();
            temp_armor_msg.corners_y.clear();

            temp_armor_msg.type = armor.type;
            temp_armor_msg.color = armor.ActualColor();
            // temp_armor_msg.rotation.clear();
            // temp_armor_msg.translation.clear();

            ArmorTransform target_transform{};
            std::ranges::copy(armor.apex, std::back_inserter(pnp_points));
            if (!(armor.IsLarge() 
                ? solver.SolveLargeArmor(pnp_points, target_transform)
                : solver.SolveSmallArmor(pnp_points, target_transform))) {
                continue;
            }
            auto &t = target_transform.Translation;
            temp_armor_msg.distance = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
            temp_armor_msg.translation = {t[0], t[1], t[2]};
            // temp_armor_msg.rotation = std::move(target_transform.Rotation);

            /// 提醒: apex是顺时针排布，从左上开始
            temp_armor_msg.corners_x = {armor.apex[0].x, armor.apex[1].x, armor.apex[2].x, armor.apex[3].x};
            temp_armor_msg.corners_y = {armor.apex[0].y, armor.apex[1].y, armor.apex[2].y, armor.apex[3].y};

            const bool is_high_armor = std::abs(t[1]) > high_height_threshold_m;
            if (is_high_armor && high_armors_msg) {
                temp_armor_msg.type = static_cast<int>(ArmorType::Outpost);
                high_armors_msg->armors.emplace_back(temp_armor_msg);
                continue;
            }

            const auto current_msg_index = static_cast<std::uint64_t>(armors_msg.armors.size());
            if(armor.type == static_cast<int>(target)){
                const auto offset = CalDistance(armor);
                temp_armor_msg.distance_to_image_center = offset;
                if(offset < min_offset){
                    // if(offset < last_min_distance && min_offset - offset > thread_hold)
                    // {
                        min_offset = offset;
                        min_index = current_msg_index;
                        selected_source_index = i;
                        last_min_distance = offset;
                    // }
                }
            }
            armors_msg.armors.emplace_back(std::move(temp_armor_msg));
        }
        /// TODO 正在考虑要不要把Yaw和Pitch也初始化一下
        armors_msg.is_available_armor_for_predictor = false;
        if(min_offset == FLT_MAX) return false;
        target_armor = armors[selected_source_index];
        armors_msg.is_available_armor_for_predictor = true;
        armors_msg.target_armor_index_for_predictor = min_index;
        
        return true;
    }

    float ArmorRefinder::CalDistance(const ArmorObject& armor) const noexcept{
        static const cv::Point2f Center(640, 512);
        const auto& points = armor.apex;
        const auto center = (points[0] + points[1] + points[2] + points[3]) / 4;
        const auto offset_vector = center - Center;
        const auto offset = std::sqrt(offset_vector.dot(offset_vector));
        return offset;
    }
}