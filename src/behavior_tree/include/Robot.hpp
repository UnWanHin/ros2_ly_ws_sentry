// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include "../module/BasicTypes.hpp"
// [新增] 必須包含 Area 定義，因為下面用到了 BehaviorTree::Area::...
#include "../module/Area.hpp" 

#include <iostream>
#include <chrono>
#include <array> // [新增] 建議加上這個，因為用了 std::array

using namespace LangYa;

namespace BehaviorTree {

    enum class TeamedLocation {
        Home, Recovery, Castle, CastleLeft, CastleRight, 
        FlyDefense, OutpostArea, MidShoot, LeftShoot, 
        BuffShoot, CastleRight1
    };

    // 2. 定義 ID 映射函數 (這就是你在 Node.hpp 裡呼叫的東西)
    // 這些函數回傳 uint8_t (ID)，而不是 Point (座標)
    constexpr std::uint8_t Home(UnitTeam) { return 1; }
    constexpr std::uint8_t Recovery(UnitTeam) { return 2; }
    constexpr std::uint8_t MidShoot(UnitTeam) { return 10; } // 假設 ID 是 10
    constexpr std::uint8_t Castle(UnitTeam) { return 30; }
    constexpr std::uint8_t CastleLeft(UnitTeam) { return 31; }
    constexpr std::uint8_t CastleRight(UnitTeam) { return 32; }
    constexpr std::uint8_t CastleRight1(UnitTeam) { return 33; }
    constexpr std::uint8_t BuffShoot(UnitTeam) { return 40; }
    constexpr std::uint8_t FlyDefense(UnitTeam) { return 50; }
    constexpr std::uint8_t OutpostArea(UnitTeam) { return 60; }
    constexpr std::uint8_t LeftShoot(UnitTeam) { return 70; }

    class Robot {
    public:
        Robot() {
            maxHealth_ = 100;
            currentHealth_ = 100;
            distance_ = 30;
            invulnerableStartTime_ = std::chrono::steady_clock::now();
        }
        
        // 修正構造函數初始化順序警告 (按聲明順序)
        Robot(int maxHealth, double initialDistance)
            : team_(UnitTeam::Blue), // 建議給個默認值防止未初始化
              maxHealth_(maxHealth), 
              currentHealth_(maxHealth), 
              distance_(initialDistance), 
              invulnerableStartTime_() {}

        // 检查是否处于无敌状态
        bool isInvulnerable() const {
            if (currentHealth_ == 0) return false;
            
            auto now = std::chrono::steady_clock::now();
            
            // [優化] 修復運算符優先級問題，並讓代碼更易讀
            bool has_valid_start_time = invulnerableStartTime_.time_since_epoch().count() != 0;
            if (!has_valid_start_time) return false;

            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - invulnerableStartTime_).count();
            return (duration < 10);
        }

        // 判断是否位于特殊区域
        // 這些調用依賴 Area.hpp，請確保該文件存在且正確
        bool inCastleMyself() {
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::CastleRed.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::CastleBlue.isPointInside(position_.X, position_.Y);
        } 
        bool inCastleEnemy() {
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::CastleBlue.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::CastleRed.isPointInside(position_.X, position_.Y);
        } 
        bool inCentralHighLandRedMysself() {
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::CentralHighLandRed.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::CentralHighLandBlue.isPointInside(position_.X, position_.Y);
        } 
        bool inCentralHighLandEnemy() {
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::CentralHighLandBlue.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::CentralHighLandRed.isPointInside(position_.X, position_.Y);
        } 
        bool inRoadLandMyself() { 
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::RoadLandRed.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::RoadLandBlue.isPointInside(position_.X, position_.Y);
        } 
        bool inRoadLandEnemy() { 
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::RoadLandBlue.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::RoadLandRed.isPointInside(position_.X, position_.Y);
        } 
        bool inFlyLandMyself() { 
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::FlyLandRed.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::FlyLandBlue.isPointInside(position_.X, position_.Y);
        } 
        bool inFlyLandEnemy() { 
            if (team_ == UnitTeam::Red) return BehaviorTree::Area::FlyLandBlue.isPointInside(position_.X, position_.Y);
            return BehaviorTree::Area::FlyLandRed.isPointInside(position_.X, position_.Y);
        }


        void setCurrentHealth(int health) {
            if (currentHealth_ == 0 && health > 0 && health != maxHealth_) { // 判断是否无敌
                invulnerableStartTime_ = std::chrono::steady_clock::now();
            }
            currentHealth_ = health;
        }

        void SetDistance(double distance) {
            distance_ = distance;
        }
        void SetPosition(UWBPositionType position) {
            position_ = position;
        }
        void setTeamColor(UnitTeam team) {
            team_ = team;
        }
        
    public:
        // [建議] 如果 UnitTeam 是 enum class，這裡可能需要初始化
        UnitTeam team_;
        std::uint16_t maxHealth_;                     
        std::uint16_t currentHealth_;                 
        float distance_;                              
        UWBPositionType position_;                    
        std::chrono::steady_clock::time_point invulnerableStartTime_; 
    };

    class Robots {
    public:
        Robots() {
            // 初始化邏輯保持不變
            for (auto& robot : robots_) {
                robot = Robot(100, 30);
            }
            // 確保 UnitType 被包含在 BasicTypes.hpp 中
            robots_[static_cast<size_t>(UnitType::Hero)] = Robot(200, 30);
            robots_[static_cast<size_t>(UnitType::Engineer)] = Robot(250, 30);
            robots_[static_cast<size_t>(UnitType::Sentry)] = Robot(400, 30);
        }
        Robot& operator[](UnitType type) {
            return robots_[static_cast<size_t>(type)];
        }

        const Robot& operator[](UnitType type) const {
            return robots_[static_cast<size_t>(type)];
        }

        Robot& operator[](int type) {
            return robots_[type];
        }
        const Robot& operator[](int type) const {
            return robots_[type];
        }

    private:
        std::array<Robot, 10> robots_; 
    };

}