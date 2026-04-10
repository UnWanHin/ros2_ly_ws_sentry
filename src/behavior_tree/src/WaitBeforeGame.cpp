// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include <algorithm>

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
        naviVelocityInput = VelocityType{0, 0};
        naviVelocity = VelocityType{0, 0};
        postureCommand = 0;

        LoggerPtr->Info("Waiting For Game Start!");

        const auto wait_begin = std::chrono::steady_clock::now();
        auto last_wait_log = wait_begin;
        auto last_league_gate_log = wait_begin;
        bool bypass_logged = false;
        const bool league_damage_open_gate_enabled =
            IsLeagueProfile() && config.LeagueStrategySettings.EnableDamageOpenGate;
        const std::uint16_t league_damage_open_gate_threshold =
            std::max<std::uint16_t>(1, config.LeagueStrategySettings.DamageOpenGateThreshold);
        bool league_health_baseline_initialized = false;
        std::uint16_t league_health_peak = 0;

        if (league_damage_open_gate_enabled) {
            LoggerPtr->Info(
                "League extra start gate enabled: open by health drop >= {}.",
                static_cast<int>(league_damage_open_gate_threshold));
        }

        // [ROS 2] 不再依賴文件系統判斷，直接等待 is_game_begin 標誌
        while (rclcpp::ok()) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
            const auto now_steady = std::chrono::steady_clock::now();

            SET_POSITION(Home, team);

            if(publishNaviGoal_ && naviCommandRateClock.trigger()) {
                naviCommandRateClock.tick();
                if(config.NaviSettings.UseXY) PubNaviGoalPos();
                else PubNaviGoal();
            }

            // 开赛门控期间持续压零速度，避免下位机沿用上一拍底盘控制量。
            naviVelocityInput.X = 0;
            naviVelocityInput.Y = 0;
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

            if (league_damage_open_gate_enabled) {
                const auto is_health_input_ready = [&]() -> bool {
                    if (!hasReceivedMyselfHealth_) {
                        return false;
                    }
                    if (lastMyselfHealthRxTime.time_since_epoch().count() == 0) {
                        return false;
                    }
                    if (leagueRefereeStaleTimeoutMs_ <= 0) {
                        return true;
                    }
                    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now_steady - lastMyselfHealthRxTime).count();
                    return age_ms <= leagueRefereeStaleTimeoutMs_;
                };

                if (is_health_input_ready()) {
                    if (!league_health_baseline_initialized) {
                        league_health_peak = myselfHealth;
                        league_health_baseline_initialized = true;
                        LoggerPtr->Info(
                            "League gate baseline health captured: {}.",
                            static_cast<int>(league_health_peak));
                    } else {
                        if (myselfHealth > league_health_peak) {
                            league_health_peak = myselfHealth;
                        }
                        const std::uint16_t health_drop = league_health_peak > myselfHealth
                            ? static_cast<std::uint16_t>(league_health_peak - myselfHealth)
                            : 0;
                        if (health_drop >= league_damage_open_gate_threshold) {
                            LoggerPtr->Warning(
                                "League gate opened by health drop: peak={} current={} drop={} threshold={}.",
                                static_cast<int>(league_health_peak),
                                static_cast<int>(myselfHealth),
                                static_cast<int>(health_drop),
                                static_cast<int>(league_damage_open_gate_threshold));
                            break;
                        }
                    }
                } else if (now_steady - last_league_gate_log > std::chrono::seconds(2)) {
                    LoggerPtr->Warning(
                        "League gate waiting for fresh /ly/game/all.selfhealth (stale_timeout_ms={}).",
                        leagueRefereeStaleTimeoutMs_);
                    last_league_gate_log = now_steady;
                }
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
