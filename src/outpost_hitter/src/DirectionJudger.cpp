// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "predictor/DerectionJudger.hpp"
#include <cmath>
#include <numeric>
#include <iostream>

#define ARMOR_DISTANCE_THRESHOLD 0.2 // 装甲板距离最大差值
#define MAX_HISTORY_SIZE 10          // 最大历史记录大小

namespace PREDICTOR
{
    DirectionJudger::DirectionJudger()
    {
        derection_judged = false;
        direction = 0; // 未判断
    }

    DirectionJudger::~DirectionJudger() {}

    void DirectionJudger::updateWorldPYD(const double &world_pitch, const double &world_yaw, const double &world_dis)
    {
        if (is_first_update) // 第一次更新
        {
            last_world_pitch = world_pitch;
            last_world_yaw = world_yaw;
            last_world_dis = world_dis;
            is_first_update = false;
            return;
        }
        else
        {
            // 首先判断两次识别之间是不是切换了装甲板
            double last_flat_dis = last_world_dis * cos(last_world_pitch * M_PI / 180.0);
            double current_flat_dis = world_dis * cos(world_pitch * M_PI / 180.0);

            double dis = sqrt(pow(last_flat_dis, 2) + pow(current_flat_dis, 2) - 2 * last_flat_dis * current_flat_dis * cos(abs(world_yaw - last_world_yaw) * M_PI / 180.0)); // 三角形余弦定理
            if (dis > ARMOR_DISTANCE_THRESHOLD)                                                                                                                               // 如果距离变化超过0.1米，则认为切换了装甲板
            {
                clearWorldYawDiff();    // 清空世界平面差值
                is_first_update = true; // 重新开始更新
                return;
            }
            else
            {

                double yaw_diff = world_yaw - last_world_yaw; // 计算yaw差值
                // 将yaw差值归一化到-pi到pi之间
                yaw_diff = std::remainder(yaw_diff, 2 * M_PI);
                world_yaw_diff_history.push_back(yaw_diff);
                last_world_pitch = world_pitch;
                last_world_yaw = world_yaw;
                last_world_dis = world_dis;
            }
        }

        if (world_yaw_diff_history.size() > MAX_HISTORY_SIZE + 1)
        {
            world_yaw_diff_history.erase(world_yaw_diff_history.begin()); // 清除最旧纪录
        }

        double avg_yaw_diff;
        if (world_yaw_diff_history.size() >= MAX_HISTORY_SIZE)
        {
            avg_yaw_diff = std::accumulate(world_yaw_diff_history.begin(), world_yaw_diff_history.end(), 0.0) / world_yaw_diff_history.size(); // 计算平均yaw差值
            if (avg_yaw_diff < -0.0001) // 逆时针
            {
                direction = 1;
                derection_judged = true;
            }
            else if (avg_yaw_diff > 0.0001) // 顺时针
            {
                direction = -1;
                derection_judged = true;
            }
            else // 静止
            {
                direction = 0;
                derection_judged = false;
            }
        }
    }
}