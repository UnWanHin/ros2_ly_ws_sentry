// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
/*
 * behavior_tree 统一话题定义
 *
 * 作用：
 * - 以 LY_DEF_ROS_TOPIC 统一 topic 名称和消息类型
 * - 作为 behavior_tree 与其他模块的通信契约中心
 *
 * 维护建议：
 * - 新增或变更话题时，同时更新 docs/architecture/message_and_link_flow.md
 */

// #include <ros/ros.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/UInt16.h>
// #include <std_msgs/UInt32.h>
// #include <std_msgs/UInt8.h>
// #include <std_msgs/UInt16MultiArray.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>


// #include "auto_aim_common/Armor.h"
// #include "auto_aim_common/Armors.h"
// #include "auto_aim_common/Target.h"
// #include "gimbal_driver/GameData.h"
// #include "gimbal_driver/GimbalAngles.h"
// #include "gimbal_driver/Health.h"
// #include "gimbal_driver/UWBPos.h"
// #include "gimbal_driver/Vel.h"
// #include "gimbal_driver/BuffData.h"
// #include "gimbal_driver/PositionData.h"

#include "auto_aim_common/msg/armor.hpp"
#include "auto_aim_common/msg/armors.hpp"
#include "auto_aim_common/msg/target.hpp"
#include "auto_aim_common/msg/relative_target.hpp"
#include "gimbal_driver/msg/game_data.hpp"
#include "gimbal_driver/msg/gimbal_angles.hpp"
#include "gimbal_driver/msg/d_vel.hpp"
#include "gimbal_driver/msg/health.hpp"
#include "gimbal_driver/msg/uwb_pos.hpp"
#include "gimbal_driver/msg/vel.hpp"
#include "gimbal_driver/msg/buff_data.hpp"
#include "gimbal_driver/msg/position_data.hpp"


#include "../module/ROSTools.hpp"


namespace BehaviorTree { 
    LY_DEF_ROS_TOPIC(ly_control_angles, "/ly/control/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_control_firecode, "/ly/control/firecode", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_control_vel, "/ly/control/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_control_posture, "/ly/control/posture", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_game_all, "/ly/game/all", gimbal_driver::msg::GameData);
    
    LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_gimbal_firecode, "/ly/gimbal/firecode", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_gimbal_vel, "/ly/gimbal/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_gimbal_d_vel, "/ly/gimbal/d_vel", gimbal_driver::msg::DVel);
    LY_DEF_ROS_TOPIC(ly_gimbal_posture, "/ly/gimbal/posture", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_gimbal_capV, "/ly/gimbal/capV", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_game_eventdata, "/ly/gimbal/eventdata", std_msgs::msg::UInt32);

    LY_DEF_ROS_TOPIC(ly_me_is_precaution, "/ly/me/is_precaution", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_me_is_at_home, "/ly/me/is_at_home", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_me_is_team_red, "/ly/me/is_team_red", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_me_hp, "/ly/me/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_me_op_hp, "/ly/me/op_hp", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_me_base_hp, "/ly/me/base_hp", std_msgs::msg::UInt16);
    
    LY_DEF_ROS_TOPIC(ly_me_ammo_left, "/ly/me/ammo_left", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_me_uwb_pos, "/ly/me/uwb_pos", std_msgs::msg::UInt16MultiArray);
    
    LY_DEF_ROS_TOPIC(ly_game_is_start, "/ly/game/is_start", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_game_time_left, "/ly/game/time_left", std_msgs::msg::UInt16);
    
    LY_DEF_ROS_TOPIC(ly_enemy_hp, "/ly/enemy/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_enemy_op_hp, "/ly/enemy/op_hp", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_enemy_base_hp, "/ly/enemy/base_hp", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_aa_enable, "/ly/aa/enable", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_ra_enable, "/ly/ra/enable", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_outpost_enable, "/ly/outpost/enable", std_msgs::msg::Bool);
    
    LY_DEF_ROS_TOPIC(ly_detector_armors, "/ly/detector/armors", auto_aim_common::msg::Armors);
    LY_DEF_ROS_TOPIC(ly_predictor_target, "/ly/predictor/target", auto_aim_common::msg::Target);  // 辅瞄云台数据数据
    LY_DEF_ROS_TOPIC(ly_back_cam_target, "/ly/back_cam/target", auto_aim_common::msg::Target); // 后置相机数据
    LY_DEF_ROS_TOPIC(ly_bt_target, "/ly/bt/target", std_msgs::msg::UInt8);

    LY_DEF_ROS_TOPIC(ly_buff_target, "/ly/buff/target", auto_aim_common::msg::Target); // 打符云台数据
    LY_DEF_ROS_TOPIC(ly_outpost_target, "/ly/outpost/target", auto_aim_common::msg::Target); // 打前哨站云台数据
    
    LY_DEF_ROS_TOPIC(ly_navi_vel, "/ly/navi/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_navi_target_rel, "/ly/navi/target_rel", auto_aim_common::msg::RelativeTarget);
    LY_DEF_ROS_TOPIC(ly_navi_goal, "/ly/navi/goal", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_navi_goal_pos, "/ly/navi/goal_pos", std_msgs::msg::UInt16MultiArray);
    LY_DEF_ROS_TOPIC(ly_navi_speed_level, "/ly/navi/speed_level", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_navi_lower_head, "/ly/navi/lower_head", std_msgs::msg::UInt8);

    LY_DEF_ROS_TOPIC(ly_team_buff, "/ly/team/buff", gimbal_driver::msg::BuffData);
    LY_DEF_ROS_TOPIC(ly_me_rfid, "/ly/me/rfid", std_msgs::msg::UInt32);
    LY_DEF_ROS_TOPIC(ly_position_data, "/ly/position/data", gimbal_driver::msg::PositionData);

}
