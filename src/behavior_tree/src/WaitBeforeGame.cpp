#include "../include/Application.hpp"

using namespace LangYa;

namespace BehaviorTree {
    /**
     * @brief 等待比赛开始前的预操作 \n
     * @brief 1. 等待云台数据 \n
     * @brief 2. 等待比赛开始 \n
     * @brief 3. 设置云台角度 \n
     * @brief 4. 设置导航目标 \n
     * @brief 5. 设置底盘速度 \n
     */
    void Application::WaitForGameStart() {
        /// 设置云台角度
        gimbalControlData.GimbalAngles.Yaw = gimbalAngles.Yaw;
        gimbalControlData.GimbalAngles.Pitch = AngleType{0};
        gimbalControlData.FireCode.FireStatus = 0;
        gimbalControlData.FireCode.Rotate = 0;
        gimbalControlData.FireCode.AimMode = 0;
        naviVelocity = VelocityType{0, 0};
        postureCommand = 0;

        LoggerPtr->Info("Waiting For Game Start!");

        const auto wait_begin = std::chrono::steady_clock::now();
        auto last_wait_log = wait_begin;
        bool bypass_logged = false;

        // [ROS 2] 不再依賴文件系統判斷，直接等待 is_game_begin 標誌
        while (rclcpp::ok()) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
            const auto now_steady = std::chrono::steady_clock::now();

            SET_POSITION(Home, team);

            if(naviCommandRateClock.trigger()) {
                naviCommandRateClock.tick();
                if(config.NaviSettings.UseXY) PubNaviGoalPos();
                else PubNaviGoal();
            }

            // 开赛门控期间持续压零速度，避免下位机沿用上一拍底盘控制量。
            naviVelocity.X = 0;
            naviVelocity.Y = 0;
            PubNaviControlData();
            PubGimbalControlData();

            if (debugBypassGameStart_) {
                if (!bypass_logged) {
                    LoggerPtr->Warning(
                        "debug_bypass_is_start=true, skip waiting for /ly/game/is_start.");
                    bypass_logged = true;
                }
                break;
            }

            if (waitForGameStartTimeoutSec_ > 0 &&
                (now_steady - wait_begin) > std::chrono::seconds(waitForGameStartTimeoutSec_)) {
                LoggerPtr->Warning(
                    "WaitForGameStart timeout after {}s, continue without is_start gate.",
                    waitForGameStartTimeoutSec_);
                break;
            }

            if (now_steady - last_wait_log > std::chrono::seconds(2)) {
                if (!hasReceivedGameStartFlag_) {
                    LoggerPtr->Warning("Waiting /ly/game/is_start message...");
                } else {
                    LoggerPtr->Debug("Waiting /ly/game/is_start=true...");
                }
                last_wait_log = now_steady;
            }

            if (is_game_begin) {
                LoggerPtr->Info("!!!!Game !! start!!!!");
                break;
            }
        }
        LoggerPtr->Info("Stop Waiting For Game");
    }

    void Application::WaitBeforeGame() {
        LoggerPtr->Info("Waiting Before Game");
        PublishSafeControl("wait_before_game_reset");

        /// 取得第一个云台数据包（不能用 yaw/pitch 非 0 判定，0 也是合法角度）
        const auto wait_begin = std::chrono::steady_clock::now();
        auto last_wait_log = wait_begin;
        constexpr auto kMaxWait = std::chrono::seconds(10);
        while (rclcpp::ok()) {
            rclcpp::spin_some(node_);

            if (hasReceivedGimbalAngles_.load()) {
                LoggerPtr->Info("First gimbal message received.");
                break;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - wait_begin > kMaxWait) {
                LoggerPtr->Warning("No gimbal message within {}s, continue with current angles (Yaw={}, Pitch={}).",
                    std::chrono::duration_cast<std::chrono::seconds>(kMaxWait).count(),
                    gimbalAngles.Yaw, gimbalAngles.Pitch);
                break;
            }
            if (now - last_wait_log > std::chrono::seconds(2)) {
                LoggerPtr->Warning("Waiting for first gimbal message...");
                last_wait_log = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        LoggerPtr->Info("Stop waiting for first gimbal data.");
        
        WaitForGameStart();

        LoggerPtr->Info("Stop Waiting Before Game");
    }
}
