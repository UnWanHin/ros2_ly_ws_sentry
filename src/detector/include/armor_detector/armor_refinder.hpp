// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once 
#include <vector>
#include <auto_aim_common/DetectionType.hpp>
#include <auto_aim_common/msg/armor.hpp>
#include <auto_aim_common/msg/armors.hpp>
#include "pnp_solver.hpp"

namespace ly_auto_aim::inline detector{
    class ArmorRefinder{
        public:
        ArmorRefinder()  = default;
        ~ArmorRefinder() = default;

        /**
         * @brief 重新寻找指定类型的装甲板, 解算并将消息放入armor_msg, 主要识别装甲板重复问题
         * @param armors 传入的过滤装甲板
         * @param target_armor 传出的目标装甲板
         * @param target 传入的决策目标
         */
        [[nodiscard]] bool ReFindAndSolveAll(PoseSolver& solver, const std::vector<ArmorObject>& armors, const ArmorType& target, ArmorObject& target_armor, auto_aim_common::msg::Armors& armors_msg) noexcept;

        /**
         * 暂时弃用的函数
         */
        // void SendMsg(const ArmorObject& armor, auto_aim_common::Armor& armor_msg) const noexcept;

        /**
         * @brief 计算装甲板到图像中心的距离
         * @param armor 装甲板
         * @return float 距离
         */
        [[nodiscard]] float CalDistance(
            const ArmorObject& armor) const noexcept;

        [[nodiscard]] float GetMinOffset() const noexcept{
            return min_offset;
        }

        [[nodiscard]] std::uint64_t GetMinIndex() const noexcept{
            return min_index;
        }

        private:
        /// @brief 用于寻找最近的装甲板
        mutable float min_offset;
        mutable std::uint64_t min_index;
        float last_min_distance{FLT_MAX};
    };
}