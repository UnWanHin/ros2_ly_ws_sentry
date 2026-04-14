// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"


namespace BehaviorTree {

    void Application::PublishMessageAll() {
        // 发布顺序固定：
        // 先模式/云台/姿态/目标，再发导航速度和导航目标，减少下游状态抖动。
        PubAimModeEnableData();
        PubGimbalControlData();
        PubPostureControlData();
        PubAimTargetData();
        PubNaviControlData();
        const bool enable_relative_target_topic =
            config.ChaseSettings.Enable &&
            config.ChaseSettings.UseRelativeTargetTopic;
        if (enable_relative_target_topic) {
            PubNaviRelativeTarget();
        }
        if(publishNaviGoal_ && naviCommandRateClock.trigger()) {
            naviCommandRateClock.tick();
            const bool use_tf_goal_bridge =
                config.NaviSettings.UseXY &&
                config.NaviSettings.UseTfGoalBridge &&
                enable_relative_target_topic;
            // 导航目标按模式二选一：
            // UseXY=true 走 /ly/navi/goal_pos；否则走 /ly/navi/goal。
            // 追击+relative_topic+tf bridge 场景下，由 bridge 独占 /ly/navi/goal_pos。
            if(config.NaviSettings.UseXY && !use_tf_goal_bridge) PubNaviGoalPos();
            else PubNaviGoal();
        }
    }

    void Application::PubAimModeEnableData() {
        {
            std_msgs::msg::Bool msg;
            msg.data = (aimMode == AimMode::AutoAim || aimMode == AimMode::RotateScan);
            pub_aa_enable_->publish(msg);
        }
        {
            std_msgs::msg::Bool msg;
            msg.data = (aimMode == AimMode::Buff);
            pub_ra_enable_->publish(msg);
        }
        {
            std_msgs::msg::Bool msg;
            msg.data = (aimMode == AimMode::Outpost);
            pub_outpost_enable_->publish(msg);
        }
    }

    /**
     * @brief 发布云台角度控制数据和火控数据 \n
     * @brief 云台数据可以随便修改，修改完之后发布即可，火控数据只有在接收到辐瞄的目标数据之后才会自动翻转
     * @param gimbalControlData 云台控制数据
     */
    void Application::PubGimbalControlData() {
        {
            gimbal_driver::msg::GimbalAngles msg;
            msg.yaw   = gimbalControlData.GimbalAngles.Yaw;
            msg.pitch = gimbalControlData.GimbalAngles.Pitch;
            msg.header.stamp = node_->now();
            pub_gimbal_control_->publish(msg);
        }
        {
            std_msgs::msg::UInt8 msg;
            msg.data = *reinterpret_cast<std::uint8_t *>(&gimbalControlData.FireCode);
            pub_gimbal_firecode_->publish(msg);
        }
    }

    void Application::PubPostureControlData() {
        // 0 作为“当前决策层不下发姿态”的保留值，避免影响现有链路。
        if (postureCommand < 1 || postureCommand > 3) {
            return;
        }
        std_msgs::msg::UInt8 msg;
        msg.data = postureCommand;
        pub_gimbal_posture_->publish(msg);
    }

    /**
     * @brief 发布自瞄应该击打的目标
     */
    void Application::PubAimTargetData() {
        {
            std_msgs::msg::UInt8 msg;
            msg.data = static_cast<uint8_t>(targetArmor.Type);
            pub_bt_target_->publish(msg);
        }
    }

    /**
     * @brief 发布导航的底盘速度控制数据
     * @param naviVelocity 导航速度X, Y
     */
    void Application::PubNaviControlData() {
        {
            gimbal_driver::msg::Vel msg;
            msg.x = naviVelocity.X;
            msg.y = naviVelocity.Y;
            // 桥接到 gimbal_driver 控制口，恢复 navi->BT->control_vel 老链路。
            pub_gimbal_vel_->publish(msg);
            // 兼容保留：继续发布到 /ly/navi/vel，避免影响外部联调工具。
            //pub_navi_vel_->publish(msg);
        }
    }

    void Application::PubNaviRelativeTarget() {
        if (!pub_navi_target_rel_) {
            return;
        }
        auto_aim_common::msg::RelativeTarget msg;
        msg.header.stamp = node_->now();
        msg.valid = naviRelativeTargetValid;
        msg.x = naviRelativeTargetX;
        msg.y = naviRelativeTargetY;
        msg.z = naviRelativeTargetZ;
        msg.distance_m = naviRelativeTargetDistance;
        msg.yaw_error_deg = naviRelativeTargetYawErrorDeg;
        msg.pitch_error_deg = naviRelativeTargetPitchErrorDeg;
        msg.armor_type = naviRelativeTargetArmorType;
        msg.aim_mode = naviRelativeTargetAimMode;
        pub_navi_target_rel_->publish(msg);
    }

    /**
     * @brief 发布给导航的目标点
     */
    void Application::PubNaviGoal() {
        {
            std_msgs::msg::UInt8 msg;
            msg.data = naviCommandGoal;
            // 语义：导航目标点 ID（不是坐标）
            pub_navi_goal_->publish(msg);
        }
        {
            std_msgs::msg::UInt8 msg;
            msg.data = speedLevel;
            pub_navi_speed_level_->publish(msg);
        }
    }

    void Application::PubNaviGoalPos() {
        std_msgs::msg::UInt16MultiArray msg;
        std::vector<uint16_t> data = {
            static_cast<uint16_t>(naviGoalPosition.x),
            static_cast<uint16_t>(naviGoalPosition.y)
        };
        msg.data = data;
        if (config.NaviSettings.UseTfGoalBridge && pub_navi_goal_pos_raw_) {
            // 统一由 navi_tf_bridge 输出 /ly/navi/goal_pos，BT 仅发布 raw 点位输入。
            pub_navi_goal_pos_raw_->publish(msg);
            return;
        }
        pub_navi_goal_pos_->publish(msg);
    }
}
