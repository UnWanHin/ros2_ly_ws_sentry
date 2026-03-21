// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

// #include "../include/Application.hpp"

// using namespace LangYa;

// namespace BehaviorTree {

//     void Application::SetPositionRepeat() {
//         if(config.GameStrategySettings.HitSentry) SetPositionHitSentry();
//         else if(config.GameStrategySettings.TestNavi) SetPositionNaviTest();
//         else SetPositionProtect();
//     }

//     bool Application::CheckPositionRecovery() {
//         UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
//         int now_time = 420 - timeLeft;
//         // 复活
//         if(naviCommandGoal == Recovery(MyTeam)) {
//             if(myselfHealth < 380) {
//                 naviCommandIntervalClock.reset(Seconds{1});
//                 return true;
//             }
//         }
//         // 回家
//         // 条件为：血量低于150 或者 弹药为0且距离上一次回家已经过去90秒
//         if(myselfHealth < 150 || (ammoLeft <= 30 && recoveryClock.trigger())) {
//             naviCommandGoal = Recovery(MyTeam);
//             recoveryClock.tick();
//             naviCommandIntervalClock.reset(Seconds{1});
//             return true;
//         }
//         return false;
//     }

//     void Application::SetPositionProtect() {
//         UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
//         int now_time = 420 - timeLeft;
        
//         // 检测是否需要回家
//         if(CheckPositionRecovery()) {
//             // if(teamBuff.RemainingEnergy) speedLevel = 1;
//             // else speedLevel = 2;
//             LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
//             LoggerPtr>Info("> Go Recovery");
//             return;
//         }

//         // 防止高速切换指令
//         if(!naviCommandIntervalClock.trigger()) {
//             return;
//         }        
//         if(aimMode == AimMode::Buff) { 
//             naviCommandGoal = BuffShoot(MyTeam); // 打符
//             naviCommandIntervalClock.reset(Seconds(2));
//             // speedLevel = 2; //加速
//         }else if (aimMode == AimMode::Outpost) {
//             naviCommandGoal = OutpostShoot(MyTeam); // 打前哨站
//             naviCommandIntervalClock.reset(Seconds(2));
//             // speedLevel = 1; // 正常
//         }else { //普通模式
//             Random random;
            
//                 int random_number = random.Get(0, 6);
//                 if (random_number == 0) naviCommandGoal = CastleLeft(MyTeam);
//                 else if (random_number == 1) naviCommandGoal = CastleRight1(MyTeam);
//                 else if (random_number == 2) naviCommandGoal = CastleRight2(MyTeam);
//                 else naviCommandGoal = BuffShoot(MyTeam);
//                 if(naviCommandGoal == BuffShoot(MyTeam)) naviCommandIntervalClock.reset(Seconds{30});
//                 else naviCommandIntervalClock.reset(Seconds(10));
//             // 时间超过5分钟 或 底盘能量低于5%
//             if(now_time > 300 || 
//                 teamBuff.RemainingEnergy == 0b11100 || teamBuff.RemainingEnergy == 0b11000 || 
//                 teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
//                 LoggerPtr->Info("!!! Low Energy !!!");
//                 naviCommandGoal = Castle(MyTeam);
//                 // speedLevel = 0;
//             } //else speedLevel = 1;
//             LoggerPtr->Debug("RemainingEnergy: {}", teamBuff.RemainingEnergy);
//             LoggerPtr->Info("> NaviCommandGoal: {}", static_cast<int>(naviCommandGoal));
//         }
//     }

//     void Application::SetPositionNaviTest() {
//         UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
//         int now_time = 420 - timeLeft;
        
//         if(now_time < 10) naviCommandGoal = BuffShoot(MyTeam);
//         else if(now_time < 20) naviCommandGoal = LeftHighLand(MyTeam);
//         else if(now_time < 30) naviCommandGoal = CastleLeft(MyTeam);
//         else if(now_time < 40) naviCommandGoal = CastleRight1(MyTeam);
//         else if(now_time < 50) naviCommandGoal = CastleRight2(MyTeam);
//         else if(now_time < 60) naviCommandGoal = FlyRoad(MyTeam);
//         else if(now_time < 70) naviCommandGoal = OutpostArea(MyTeam);
//         else if(now_time < 80) naviCommandGoal = MidShoot(MyTeam);
//         else if(now_time < 90) naviCommandGoal = LeftShoot(MyTeam);
//         else if(now_time < 100) naviCommandGoal = OutpostShoot(MyTeam);
//         else if(now_time < 110) naviCommandGoal = FlyRoad(EnemyTeam);
//         else if(now_time < 120) naviCommandGoal = CastleRight1(EnemyTeam);
//         else if(now_time < 130) naviCommandGoal = CastleRight2(EnemyTeam);
//         else if(now_time < 140) naviCommandGoal = CastleLeft(EnemyTeam);
//         else if(now_time < 150) naviCommandGoal = LeftHighLand(EnemyTeam);
//         else if(now_time < 160) naviCommandGoal = BuffShoot(EnemyTeam);
//         else if(now_time < 170) naviCommandGoal = OutpostArea(EnemyTeam);
//         else naviCommandGoal = Castle(MyTeam);
//     }

//     void Application::SetPositionHitSentry() {
//         UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
//         int now_time = 420 - timeLeft;

//         if(CheckPositionRecovery()) {
//             // if(teamBuff.RemainingEnergy) speedLevel = 1;
//             // else speedLevel = 2;
//             LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
//             LoggerPtr>Info("> Go Recovery");
//             return;
//         }
//         // 防止高速切换指令
//         if(!naviCommandIntervalClock.trigger()) {
//             return;
//         }
//         if(aimMode == AimMode::Buff) { 
//             naviCommandGoal = BuffShoot(MyTeam); // 打符
//             naviCommandIntervalClock.reset(Seconds(2));
//             // speedLevel = 2;
//         }else if (aimMode == AimMode::Outpost) {
//             naviCommandGoal = OutpostShoot(MyTeam); // 打前哨站
//             naviCommandIntervalClock.reset(Seconds(2));
//             // speedLevel = 1;
//         }else { //普通模式
//             Random random;
//             if(now_time < 300) { // 第1-5分钟
//                 if(!hitableTargets.empty()) { // 视野里存在目标
//                     bool low_health_enemy = false;
//                     // 检测低血量敌人
//                     for(auto robot : hitableTargets) {
//                         if(Robots[robot].currentHealth_ < 50) {
//                             low_health_enemy = true;
//                         }
//                     }
//                     if(low_health_enemy) naviCommandIntervalClock.reset(Seconds(1));
//                 }else {
//                     if(now_time < 60) naviCommandGoal = OutpostArea(MyTeam);
//                     else {
//                         int random_number = random.Get(0, 8);
//                         if(random_number == 0) naviCommandGoal = MidShoot(MyTeam);
//                         else if(random_number == 1) naviCommandGoal = BuffAround1(MyTeam);
//                         else if(random_number == 2) naviCommandGoal = BuffAround2(MyTeam);
//                         else if(random_number == 3) naviCommandGoal = RightShoot(MyTeam);
//                         else if(random_number == 4) naviCommandGoal = MidShoot(EnemyTeam);
//                         else if(random_number == 5) naviCommandGoal = BuffAround1(EnemyTeam);
//                         else if(random_number == 6) naviCommandGoal = BuffAround2(EnemyTeam);
//                         else if(random_number == 7) naviCommandGoal = RightShoot(EnemyTeam);
//                         else if(random_number == 8) naviCommandGoal = LeftShoot(EnemyTeam);
//                     }
//                     if(naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam))
//                         naviCommandIntervalClock.reset(Seconds(8));
//                     else naviCommandIntervalClock.reset(Seconds(10));
//                 }
//             } else { // 6-7 分钟
//                 int random_number = random.Get(0, 3);
//                 if (random_number == 0) naviCommandGoal = CastleLeft(MyTeam);
//                 else if (random_number == 1) naviCommandGoal = CastleRight1(MyTeam);
//                 else if (random_number == 2) naviCommandGoal = BuffShoot(MyTeam);
//                 else naviCommandGoal = CastleRight2(MyTeam);
//                 if(naviCommandGoal == BuffShoot(MyTeam)) naviCommandIntervalClock.reset(Seconds{30});
//                 else naviCommandIntervalClock.reset(Seconds(10));
//             }
//             // 底盘能量低于5%
//             if(now_time > 300 || 
//                 teamBuff.RemainingEnergy == 0b11100 || teamBuff.RemainingEnergy == 0b11000 || 
//                 teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
//                 LoggerPtr->Info("!!! Low Energy !!!");
//                 naviCommandGoal = Castle(MyTeam);
//                 // speedLevel = 0;
//             } // else speedLevel = 1;
//             LoggerPtr->Info("> NaviCommandGoal: {}", static_cast<int>(naviCommandGoal));
//             LoggerPtr->Debug("RemainingEnergy: {}", teamBuff.RemainingEnergy);
//         }
//     }



// }