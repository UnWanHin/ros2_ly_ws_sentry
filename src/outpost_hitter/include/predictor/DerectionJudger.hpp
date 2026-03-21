// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <vector>

namespace PREDICTOR
{

    class DirectionJudger
    {
    public:
        DirectionJudger();
        ~DirectionJudger();

        void updateWorldPYD(const double &world_pitch, const double &world_yaw, const double &world_dis);
        void clearWorldYawDiff()
        {
            world_yaw_diff_history.clear();
            direction = 0;
            derection_judged = false;
            is_first_update = true; // 重置标志位
        }
        bool isDerectionJudged() const
        {
            return derection_judged;
        }
        int getDirection() const
        {
            return direction;
        }

    private:
        double last_world_pitch; // 上一次世界坐标系下的pitch角
        double last_world_yaw;   // 上一次世界坐标系下的yaw角
        double last_world_dis;   // 上一次世界坐标系下的距离

        bool derection_judged;       // 是否已判断方向
        int direction;               // 方向：-1为顺时针，1为逆时针，0为未判断
        bool is_first_update = true; // 是否第一次更新

        std::vector<double> world_yaw_diff_history; // 世界坐标系下的平面坐标差值滑窗
    };
} // namespace PREDICTOR