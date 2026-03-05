#include "../include/Application.hpp"


namespace BehaviorTree {

    void Application::PublishMessageAll() {
        
        PubAimModeEnableData();
        PubGimbalControlData();
        PubPostureControlData();
        PubAimTargetData();
        PubNaviControlData();
        if(naviCommandRateClock.trigger()) {
            naviCommandRateClock.tick();
            // PubNaviGoal();
            if(config.NaviSettings.UseXY) PubNaviGoalPos();
            else PubNaviGoal();
        }
    }

    void Application::PubAimModeEnableData() {
        {
            std_msgs::msg::Bool msg;
            if (aimMode != AimMode::Buff) {
                msg.data = true;
            } else msg.data = false;
            pub_aa_enable_->publish(msg);
        }
        {
            std_msgs::msg::Bool msg;
            if (aimMode == AimMode::Buff) {
                msg.data = true;
            } else msg.data = false;
            pub_ra_enable_->publish(msg);
        }
        {
            std_msgs::msg::Bool msg;
            if (aimMode == AimMode::Outpost) {
                msg.data = true;
            } else msg.data = false;
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
            pub_navi_vel_->publish(msg);
        }
    }

    /**
     * @brief 发布给导航的目标点
     */
    void Application::PubNaviGoal() {
        {
            std_msgs::msg::UInt8 msg;
            msg.data = naviCommandGoal;
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
        pub_navi_goal_pos_->publish(msg);
    }
}
