// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include "../module/ROSTools.hpp" //add 

#include <random>
#include <algorithm>

#include "../module/BasicTypes.hpp"
#include "../include/Robot.hpp"

// using namespace BehaviorTree;

namespace LangYa{

    static constexpr char BTName[] = "BehaviorTree";
    constexpr std::uint16_t LowSelfHealthThreshold = 180;
    constexpr std::uint16_t LowAmmoThreshold = 10;

/* 
 * 已经完成的节点
 * GoToPosition
 * TODO 列出节点
 */


/*
 * 按照顺序导航到各个点位
 * <SetPositionRepeat
 *     Count="{Count}"
 *     MyTeam="{MyTeam}"
 *     DestinationID="{DestinationID}"
 * />
 */
    class SetPositionRepeat : public BT::SyncActionNode {
    public:
        SetPositionRepeat(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<int>("Count"),
                    BT::InputPort<UnitTeam>("MyTeam"),
                    BT::BidirectionalPort<std::uint8_t>("DestinationID")
            };
        }

        BT::NodeStatus tick() override {
            /// spdlog::info("{}> SetPositionRepeat", BTName);
            int Count{};
            UnitTeam MyTeam{};
            std::uint8_t DestinationID{};
            if (!getInput("Count", Count)) {
                // throw BT::RuntimeError("Missing required input [Count]");
                /// spdlog::error("{}> Missing required input [Count]", BTName);
            }
            if (!getInput("MyTeam", MyTeam)) {
                // throw BT::RuntimeError("Missing required input [MyTeam]");
                /// spdlog::error("{}> Missing required input [MyTeam]", BTName);
            }
            if (!getInput("DestinationID", DestinationID)) {
                // throw BT::RuntimeError("Missing required input [DestinationID]");
                /// spdlog::error("{}> Missing required input [DestinationID]", BTName);
            }
            /// spdlog::info("Count: {}", Count);
            UnitTeam EnemyTeam = MyTeam == UnitTeam::Red ? UnitTeam::Blue : UnitTeam::Red;

            if (DestinationID != MidShoot(MyTeam)) DestinationID = MidShoot(MyTeam);
            else {
                std::random_device rd;
                std::mt19937 rng(static_cast<unsigned int>(rd()));
                std::uniform_int_distribution<int> dist(0, 3);
                int random_number = dist(rng);
                if (random_number == 0) DestinationID = MidShoot(MyTeam);
                else if (random_number == 1) DestinationID = Castle(MyTeam);
                else if (random_number == 2) {
                    rng = std::mt19937(static_cast<unsigned int>(rd()));
                    random_number = dist(rng);
                    if (random_number == 0) DestinationID = Castle(MyTeam);
                    else if (random_number == 1) DestinationID = CastleLeft(MyTeam);
                    else if (random_number == 2) DestinationID = CastleRight1(MyTeam);
                    else DestinationID = BuffShoot(MyTeam);
                }
                else DestinationID = MidShoot(MyTeam);
            }

            setOutput("DestinationID", DestinationID);
            return BT::NodeStatus::SUCCESS;
        }
    };

    class SetPositionRepeatFired : public BT::SyncActionNode {
        public:
            SetPositionRepeatFired(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    
            static BT::PortsList providedPorts() {
                return {
                        BT::InputPort<UnitTeam>("MyTeam"),
                        BT::InputPort<std::uint16_t>("SelfHealth"),
                        BT::InputPort<HealthMyselfData>("MyTeamHealth"),
                        BT::InputPort<HealthEnemyData>("EnemyHealth"),
                        BT::InputPort<std::uint16_t>("SelfOutpostHealth"),
                        BT::InputPort<std::uint16_t>("EnemyOutpostHealth"),
                        BT::InputPort<std::uint16_t>("TimeLeft"),
                        BT::InputPort<std::uint16_t>("AmmoLeft"),
                        BT::BidirectionalPort<std::chrono::seconds>("CommandInterval"),
                        BT::BidirectionalPort<std::chrono::steady_clock::time_point>("LastCommandTime"),
                        BT::BidirectionalPort<std::uint8_t>("DestinationID")
                };
            }
            BT::NodeStatus tick() override {
                UnitTeam MyTeam{};
                std::uint16_t SelfHealth;
                std::uint8_t DestinationID{};
                HealthMyselfData MyTeamHealth;
                HealthEnemyData EnemyHealth;
                std::uint16_t SelfOutpostHealth;
                std::uint16_t EnemyOutpostHealth;
                std::uint16_t TimeLeft{};
                std::uint16_t AmmoLeft{};
                std::chrono::seconds CommandInterval{1s};
                std::chrono::steady_clock::time_point LastCommandTime{};
                if (!getInput("MyTeam", MyTeam)) {
                    // throw BT::RuntimeError("Missing required input [MyTeam]");
                    /// spdlog::error("{}> Missing required input [MyTeam]", BTName);
                }
                if(!getInput("SelfHealth", SelfHealth)) {
                    
                }
                if (!getInput("DestinationID", DestinationID)) {
                    // throw BT::RuntimeError("Missing required input [DestinationID]");
                    /// spdlog::error("{}> Missing required input [DestinationID]", BTName);
                }
                if (!getInput("MyTeamHealth", MyTeamHealth)) {
                    // throw BT::RuntimeError("Missing required input [MyTeamHealth]");
                    /// spdlog::error("{}> Missing required input [MyTeamHealth]", BTName);
                }
                if (!getInput("EnemyHealth", EnemyHealth)) {
                    // throw BT::RuntimeError("Missing required input [EnemyHealth]");
                    /// spdlog::error("{}> Missing required input [EnemyHealth]", BTName);
                }
                if (!getInput("SelfOutpostHealth", SelfOutpostHealth)) {
                    // throw BT::RuntimeError("Missing required input [SelfOutpostHealth]");
                    /// spdlog::error("{}> Missing required input [SelfOutpostHealth]", BTName);
                }
                if (!getInput("EnemyOutpostHealth", EnemyOutpostHealth)) {
                    // throw BT::RuntimeError("Missing required input [EnemyOutpostHealth]");
                    /// spdlog::error("{}> Missing required input [EnemyOutpostHealth]", BTName);
                }
                if (!getInput("TimeLeft", TimeLeft)) {
                    // throw BT::RuntimeError("Missing required input [TimeLeft]");
                    /// spdlog::error("{}> Missing required input [TimeLeft]", BTName);
                }
                if (!getInput("AmmoLeft", AmmoLeft)) {
                    // throw BT::RuntimeError("Missing required input [AmmoLeft]");
                    /// spdlog::error("{}> Missing required input [AmmoLeft]", BTName);
                }
                if (!getInput("CommandInterval", CommandInterval)) {
                    // throw BT::RuntimeError("Missing required input [CommandInterval]");
                    /// spdlog::error("{}> Missing required input [CommandInterval]", BTName);
                }
                if (!getInput("LastCommandTime", LastCommandTime)) {
                    // throw BT::RuntimeError("Missing required input [LastCommandTime]");
                    /// spdlog::error("{}> Missing required input [LastCommandTime]", BTName);
                }
    
                /// spdlog::info("占点策略");
                auto now_time = std::chrono::steady_clock::now();
                auto time_diff = now_time - LastCommandTime;
                time_diff = std::chrono::duration_cast<std::chrono::seconds>(time_diff);
                UnitTeam Enemy = MyTeam == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    
                /// spdlog::info("血量数据：{}", SelfHealth);
                /// spdlog::info("英雄血量{}， 步兵血量{}", HeroHealth, Infantry1Health);
    
                // 复活
                if (DestinationID == Recovery(MyTeam)) {
                    /// spdlog::info("复活中");
                    if(SelfHealth < 380) {
                        CommandInterval = 1s;
                        setOutput("CommandInterval", CommandInterval);
                        setOutput("LastCommandTime", now_time);
                        return BT::NodeStatus::SUCCESS;
                    }
                }
                // 回家
                if (SelfHealth < 150) {
                    /// spdlog::info("回家中");
                    DestinationID = Recovery(MyTeam);
                    CommandInterval = 1s;
                    setOutput("CommandInterval", CommandInterval);
                    setOutput("LastCommandTime", now_time);
                    setOutput("DestinationID", DestinationID);
                    return BT::NodeStatus::SUCCESS;
                }
    
                // 防止高速切换指令
                if (time_diff <= CommandInterval) {
                    /// spdlog::info("time diff{}, CommandInterval{}",time_diff.count(), CommandInterval.count());
                    return BT::NodeStatus::SUCCESS;
                }
                // if(TimeLeft >= 240) { // 第1-3分钟
                //     std::random_device rd;
                //     std::mt19937 rng(static_cast<unsigned int>(rd()));
                //     std::uniform_int_distribution<int> dist(0, 6);
                //     int random_number = dist(rng);
                //     if(random_number == 0) DestinationID = OutpostArea(MyTeam);
                //     else if(random_number == 1) DestinationID = MidShoot(MyTeam);
                //     else if(random_number == 2) DestinationID = LeftShoot(MyTeam);
                //     else if(random_number == 3) DestinationID = OutpostArea(Enemy);
                //     else if(random_number == 4) DestinationID = MidShoot(Enemy);
                //     else if(random_number == 5) DestinationID = LeftShoot(Enemy);
                //     else if(random_number == 6) DestinationID = OutpostArea(MyTeam);
                //     if(DestinationID == OutpostArea(MyTeam)) CommandInterval = 15s;
                //     else CommandInterval = 10s;
                // }else {
                    std::random_device rd;
                    std::mt19937 rng(static_cast<unsigned int>(rd()));
                    std::uniform_int_distribution<int> dist(0, 6);
                    int random_number = dist(rng);
                    if (random_number == 0) DestinationID = CastleLeft(MyTeam);
                    else if (random_number == 1) DestinationID = CastleRight1(MyTeam);
                    else if (random_number == 2) DestinationID = BuffShoot(MyTeam);
                    else DestinationID = Castle(MyTeam);
                    CommandInterval = 15s;

                // }
                // if(SelfOutpostHealth < 120) {
                //     std::random_device rd;
                //     std::mt19937 rng(static_cast<unsigned int>(rd()));
                //     std::uniform_int_distribution<int> dist(0, 6);
                //     int random_number = dist(rng);
                //     if (random_number == 0) DestinationID = CastleLeft(MyTeam);
                //     else if (random_number == 1) DestinationID = CastleRight(MyTeam);
                //     else if (random_number == 2) DestinationID = FlyDefense(MyTeam);
                //     else DestinationID = Castle(MyTeam);
                //     CommandInterval = 15s;
                // }
    
    
    //        DestinationID = MidShoot(MyTeam);
                // CommandInterval = 10s;
    
                setOutput("CommandInterval", CommandInterval);
                setOutput("LastCommandTime", now_time);
                setOutput("DestinationID", DestinationID);
                return BT::NodeStatus::SUCCESS;
            }
    };
    


/*
 * 从AimData中获取目标并设置击打目标，使用方法如下
 * <SetAimTargetFromAim
 *     ArmorList="{ArmorList}"
 *     AimTarget="{AimTarget}"
 * />
 */
class SetAimTargetFromAim : public BT::SyncActionNode {
    public:
        SetAimTargetFromAim(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    // BT::InputPort<std::array<ArmorData, 10>>("ArmorList"),
                    BT::InputPort<std::vector<ArmorData>>("ArmorList"), //改成用vector去接收
                    BT::InputPort<HealthEnemyData>("EnemyHealth"),
                    BT::OutputPort<ArmorType>("AimTarget")
            };
        }

        BT::NodeStatus tick() override {

            /// spdlog::info("{}> SetAimTargetFromAim", BTName);

            // std::array<ArmorData, 10> ArmorList{};
            std::vector<ArmorData> ArmorList; //改成用vector去接收
            HealthEnemyData EnemyHealth{};
            if (!getInput("ArmorList", ArmorList)) {
                // throw BT::RuntimeError("Missing required input [ArmorList]");
                /// spdlog::error("{}> Missing required input [ArmorList]", BTName);
                return BT::NodeStatus::FAILURE;
            }
            if (!getInput("EnemyHealth", EnemyHealth)) {
                // throw BT::RuntimeError("Missing required input [EnemyHealth]");
                /// spdlog::error("{}> Missing required input [EnemyHealth]", BTName);
            }
            bool hero_find{false}, infantry1_find{false}, infantry2_find{false}, sentry_find{false}, engineer_find{false};
            bool infantry_find{false};
            ArmorType infantry_target{ArmorType::Infantry1}, aim_target{ArmorType::Hero};
            // for (auto Armor : ArmorList) {
            //     if (Armor.Type == ArmorType::UnKnown) continue;

            //     /// spdlog::info("ArmorList: {}", static_cast<int>(Armor.Type));

            //     infantry1_find = (Armor.Type == ArmorType::Infantry1);
            //     infantry2_find = (Armor.Type == ArmorType::Infantry2);
            //     engineer_find = (Armor.Type == ArmorType::Engineer);
            //     hero_find = (Armor.Type == ArmorType::Hero);
            //     sentry_find = (Armor.Type == ArmorType::Sentry);

            // }
            for (const auto& Armor : ArmorList) {
            if (Armor.Type == ArmorType::UnKnown) continue;

            // [關鍵修改] 使用 if 判斷，只要找到就標記為 true，不要用賦值覆蓋
            if (Armor.Type == ArmorType::Infantry1) infantry1_find = true;
            else if (Armor.Type == ArmorType::Infantry2) infantry2_find = true;
            else if (Armor.Type == ArmorType::Engineer) engineer_find = true;
            else if (Armor.Type == ArmorType::Hero) hero_find = true;
            else if (Armor.Type == ArmorType::Sentry) sentry_find = true;
    }
            // std::sort(ArmorList, ArmorList + 5);
        //     /**/std::sort(sortedArmors.begin(), sortedArmors.end(),
        // /**/          [](const auto_aim_common::Armor& a, const auto_aim_common::Armor& b) {
        // /**/              return a.distance < b.distance;
        // /**/          });
            infantry_find = infantry1_find || infantry2_find;
            if (hero_find) {
                aim_target = ArmorType::Hero;
            }else if(infantry_find){
                if(infantry1_find && infantry2_find){
                    std ::uint16_t infantry1_health{200}, infantry2_health{200};
                    infantry1_health = EnemyHealth.Infantry1Enemy;
                    infantry2_health = EnemyHealth.Infantry2Enemy;
                    if(infantry1_health > infantry2_health) infantry_target = ArmorType::Infantry1;
                    else infantry_target = ArmorType::Infantry2;
                    aim_target = infantry_target;
                } else if (infantry1_find && !infantry2_find) {
                    aim_target = ArmorType::Infantry1;
                } else {
                    aim_target = ArmorType::Infantry2;
                }
            }
            else if(sentry_find) aim_target = ArmorType::Sentry;
            else if(engineer_find) aim_target = ArmorType::Engineer;
            else aim_target = ArmorType::Hero;
            /// spdlog::info("SetAimTargetFromAim: {}", static_cast<int>(aim_target));
            setOutput("AimTarget", aim_target);
            return BT::NodeStatus::SUCCESS;
        }
    };

    class SetNaviPosition : public BT::SyncActionNode {
        public:
            SetNaviPosition(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    
            static BT::PortsList providedPorts() {
                return {
                        BT::InputPort<UnitTeam>("MyTeam"),
                        BT::InputPort<std::uint16_t>("TimeLeft"),
                        BT::InputPort<std::uint16_t>("SelfHealth"),
                        BT::InputPort<std::uint16_t>("AmmoLeft"),
                        BT::InputPort<BehaviorTree::Robots>("FriendRobots"),
                        BT::InputPort<BehaviorTree::Robots>("EnemyRobots"),
                        BT::InputPort<std::uint16_t>("SelfOutpostHealth"),
                        BT::InputPort<std::uint16_t>("EnemyOutpostHealth"),
                        BT::InputPort<BuffType>("TeamBuff"),
                        BT::BidirectionalPort<std::chrono::seconds>("CommandInterval"),
                        BT::BidirectionalPort<std::chrono::steady_clock::time_point>("LastCommandTime"),
                        BT::BidirectionalPort<std::uint8_t>("DestinationID")
                };
            }
            BT::NodeStatus tick() override {
                UnitTeam MyTeam{};
                std::uint16_t TimeLeft{};
                std::uint16_t SelfHealth;
                std::uint16_t AmmoLeft{};
                BehaviorTree::Robots FriendRobots;
                BehaviorTree::Robots EnemyRobots;
                std::uint16_t SelfOutpostHealth;
                std::uint16_t EnemyOutpostHealth;
                BuffType TeamBuff{};
                std::chrono::seconds CommandInterval{1s};
                std::chrono::steady_clock::time_point LastCommandTime{};
                std::uint8_t DestinationID{};
                if (!getInput("MyTeam", MyTeam)) {
                    // throw BT::RuntimeError("Missing required input [MyTeam]");
                    /// spdlog::error("{}> Missing required input [MyTeam]", BTName);
                }
                if (!getInput("TimeLeft", TimeLeft)) {
                    // throw BT::RuntimeError("Missing required input [TimeLeft]");
                    /// spdlog::error("{}> Missing required input [TimeLeft]", BTName);
                }
                if(!getInput("SelfHealth", SelfHealth)) {
                    
                }
                if (!getInput("AmmoLeft", AmmoLeft)) {
                    // throw BT::RuntimeError("Missing required input [AmmoLeft]");
                    /// spdlog::error("{}> Missing required input [AmmoLeft]", BTName);
                }
                if (!getInput("FriendRobots", FriendRobots)) {
                    // throw BT::RuntimeError("Missing required input [FriendRobots]");
                    /// spdlog::error("{}> Missing required input [FriendRobots]", BTName);
                }
                if (!getInput("EnemyRobots", EnemyRobots)) {
                    // throw BT::RuntimeError("Missing required input [EnemyRobots]");
                    /// spdlog::error("{}> Missing required input [EnemyRobots]", BTName);
                }
                if (!getInput("TeamBuff", TeamBuff)) {
                    // throw BT::RuntimeError("Missing required input [TeamBuff]");
                    /// spdlog::error("{}> Missing required input [TeamBuff]", BTName);
                }
                if (!getInput("DestinationID", DestinationID)) {
                    // throw BT::RuntimeError("Missing required input [DestinationID]");
                    /// spdlog::error("{}> Missing required input [DestinationID]", BTName);
                }
                if (!getInput("SelfOutpostHealth", SelfOutpostHealth)) {
                    // throw BT::RuntimeError("Missing required input [SelfOutpostHealth]");
                    /// spdlog::error("{}> Missing required input [SelfOutpostHealth]", BTName);
                }
                if (!getInput("EnemyOutpostHealth", EnemyOutpostHealth)) {
                    // throw BT::RuntimeError("Missing required input [EnemyOutpostHealth]");
                    /// spdlog::error("{}> Missing required input [EnemyOutpostHealth]", BTName);
                }
                
                
                if (!getInput("CommandInterval", CommandInterval)) {
                    // throw BT::RuntimeError("Missing required input [CommandInterval]");
                    /// spdlog::error("{}> Missing required input [CommandInterval]", BTName);
                }
                if (!getInput("LastCommandTime", LastCommandTime)) {
                    // throw BT::RuntimeError("Missing required input [LastCommandTime]");
                    /// spdlog::error("{}> Missing required input [LastCommandTime]", BTName);
                }
    
                /// spdlog::info("占点策略");
                auto now_time = std::chrono::steady_clock::now();
                auto time_diff = now_time - LastCommandTime;
                time_diff = std::chrono::duration_cast<std::chrono::seconds>(time_diff);
                UnitTeam Enemy = MyTeam == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    
                /// spdlog::info("血量数据：{}", SelfHealth);
                /// spdlog::info("英雄血量{}， 步兵血量{}", HeroHealth, Infantry1Health);
    
                // 复活
                if (DestinationID == Recovery(MyTeam)) {
                    /// spdlog::info("复活中");
                    if(SelfHealth < 380) {
                        CommandInterval = 1s;
                        setOutput("CommandInterval", CommandInterval);
                        setOutput("LastCommandTime", now_time);
                        return BT::NodeStatus::SUCCESS;
                    }
                }
                // 回家
                if (SelfHealth < 150) {
                    /// spdlog::info("回家中");
                    DestinationID = Recovery(MyTeam);
                    CommandInterval = 1s;
                    setOutput("CommandInterval", CommandInterval);
                    setOutput("LastCommandTime", now_time);
                    setOutput("DestinationID", DestinationID);
                    return BT::NodeStatus::SUCCESS;
                }
    
                // 防止高速切换指令
                if (time_diff <= CommandInterval) {
                    /// spdlog::info("time diff{}, CommandInterval{}",time_diff.count(), CommandInterval.count());
                    return BT::NodeStatus::SUCCESS;
                }
                // if(TimeLeft >= 240) { // 第1-3分钟
                //     std::random_device rd;
                //     std::mt19937 rng(static_cast<unsigned int>(rd()));
                //     std::uniform_int_distribution<int> dist(0, 6);
                //     int random_number = dist(rng);
                //     if(random_number == 0) DestinationID = OutpostArea(MyTeam);
                //     else if(random_number == 1) DestinationID = MidShoot(MyTeam);
                //     else if(random_number == 2) DestinationID = LeftShoot(MyTeam);
                //     else if(random_number == 3) DestinationID = OutpostArea(Enemy);
                //     else if(random_number == 4) DestinationID = MidShoot(Enemy);
                //     else if(random_number == 5) DestinationID = LeftShoot(Enemy);
                //     else if(random_number == 6) DestinationID = OutpostArea(MyTeam);
                //     if(DestinationID == OutpostArea(MyTeam)) CommandInterval = 15s;
                //     else CommandInterval = 10s;
                // }else {
                    std::random_device rd;
                    std::mt19937 rng(static_cast<unsigned int>(rd()));
                    std::uniform_int_distribution<int> dist(0, 6);
                    int random_number = dist(rng);
                    if (random_number == 0) DestinationID = CastleLeft(MyTeam);
                    else if (random_number == 1) DestinationID = CastleRight1(MyTeam);
                    else if (random_number == 2) DestinationID = BuffShoot(MyTeam);
                    else DestinationID = Castle(MyTeam);
                    CommandInterval = 15s;

                // }
                // if(SelfOutpostHealth < 120) {
                //     std::random_device rd;
                //     std::mt19937 rng(static_cast<unsigned int>(rd()));
                //     std::uniform_int_distribution<int> dist(0, 6);
                //     int random_number = dist(rng);
                //     if (random_number == 0) DestinationID = CastleLeft(MyTeam);
                //     else if (random_number == 1) DestinationID = CastleRight(MyTeam);
                //     else if (random_number == 2) DestinationID = FlyDefense(MyTeam);
                //     else DestinationID = Castle(MyTeam);
                //     CommandInterval = 15s;
                // }
    
    
    //        DestinationID = MidShoot(MyTeam);
                // CommandInterval = 10s;
    
                setOutput("CommandInterval", CommandInterval);
                setOutput("LastCommandTime", now_time);
                setOutput("DestinationID", DestinationID);
                return BT::NodeStatus::SUCCESS;
            }
    };


/*
 * 用于设置导航地点，使用方法如下
 * <GoToPosition
 *     navi_position="Home;Myself"
 *     MyTeam="{MyTeam}"
 *     DestinationID="{DestinationID}"
 * />
 */
    class GoToPosition : public BT::SyncActionNode {
    public:
        GoToPosition(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<NaviPosition>("navi_position"),
                    BT::InputPort<UnitTeam>("MyTeam"),
                    BT::OutputPort<std::uint8_t>("DestinationID")
            };
        }

        BT::NodeStatus tick() override {

            /// spdlog::info("{}> GoToPosition", BTName);

            NaviPosition navi_position{};
            UnitTeam MyTeam{};
            std::uint8_t DestinationID{};
            if (!getInput("navi_position", navi_position)) {
                // throw BT::RuntimeError("Missing required input [navit_position]");
                /// spdlog::error("{}> Missing required input [navit_position]", BTName);
            }
            if (!getInput("MyTeam", MyTeam)) {
                // throw BT::RuntimeError("Missing required input [MyTeam]");
                /// spdlog::error("{}> Missing required input [MyTeam]", BTName);
            }
            if (navi_position.Team == NaviTeam::Enemy) {
                if (MyTeam == UnitTeam::Red) DestinationID = navi_position.Location(static_cast<const UnitTeam>(UnitTeam::Blue));
                if (MyTeam == UnitTeam::Blue) DestinationID = navi_position.Location(static_cast<const UnitTeam>(UnitTeam::Red));
            }else {
                DestinationID = navi_position.Location(static_cast<const UnitTeam>(MyTeam));
            }
            setOutput("DestinationID", DestinationID);
            return BT::NodeStatus::SUCCESS;
        }
    };

/*
 * 用于设置辅瞄击打目标，使用方法如下
 * <SetAimTarget
 *     target="Hero"
 *     AimTarget="{AimTarget}"
 * />
 */
    class SetAimTarget : public BT::SyncActionNode {
    public:
        SetAimTarget(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<ArmorType>("target"),
                    BT::OutputPort<ArmorType>("AimTarget")
            };
        }

        BT::NodeStatus tick() override {

            /// spdlog::info("{}> SetAimTarget", BTName);

            ArmorType target{};
            if (!getInput("target", target)) {
                // throw BT::RuntimeError("Missing required input [target]");
                /// spdlog::error("{}> Missing required input [target]", BTName);
            }
            /// spdlog::debug("SetAimTarget: {}", static_cast<int>(target));
            setOutput("AimTarget", target);
            return BT::NodeStatus::SUCCESS;
        }
    };


/*
 * 用于设置命令间隔，使用方法如下
 * <SetCommandInterval
 *     command_interval="30"
 *     CommandInterval="{CommandInterval}"
 *     LastCommandTime="{LastCommandTime}"
 * />
 */
    class SetCommandInterval : public BT::SyncActionNode {
    public:
        SetCommandInterval(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<int>("command_interval"),
                    BT::OutputPort<std::chrono::seconds>("CommandInterval"),
                    BT::OutputPort<std::chrono::steady_clock::time_point>("LastCommandTime")
            };
        }

        BT::NodeStatus tick() override {

            /// spdlog::info("{}> SetCommandInterval", BTName);

            int command_interval{};
            if (!getInput("command_interval", command_interval)) {
                // throw BT::RuntimeError("Missing required input [command_interval]");
                /// spdlog::error("{}> Missing required input [command_interval]", BTName);
            }
            setOutput("CommandInterval", std::chrono::seconds{command_interval});
            setOutput("LastCommandTime", std::chrono::steady_clock::now());
            return BT::NodeStatus::SUCCESS;
        }
    };

/*
 * 用于计数加法，使用方法如下
 * <CountPlus
 *     count_plus="1"
 *     Count="{Count}"
 * />
 */
    class CountPlus : public BT::SyncActionNode {
    public:
        CountPlus(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<int>("count_plus"),
                    BT::BidirectionalPort<int>("Count"),
            };
        }

        BT::NodeStatus tick() override {

            /// spdlog::info("{}> CountPlus", BTName);

            int count_plus{}, Count{};
            if (!getInput("count_plus", count_plus)) {
                // throw BT::RuntimeError("Missing required input [count_plus]");
                /// spdlog::error("{}> Missing required input [count_plus]", BTName);
            }
            if (!getInput("Count", Count)) {
                // throw BT::RuntimeError("Missing required input [Count]");
                /// spdlog::error("{}> Missing required input [Count]", BTName);
            }

            /// spdlog::info("{}> CountPlus: Count={}, count_plus={}", BTName, Count, count_plus);

            int count = Count + count_plus;

            setOutput<int>("Count", count);
            return BT::NodeStatus::SUCCESS;
        }
    };


    class RecoverState : public BT::SyncActionNode {
    public:
        RecoverState(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<std::uint8_t>("DestinationID"),
                    BT::InputPort<std::uint16_t>("SelfHealth"),
                    BT::InputPort<UnitTeam>("MyTeam"),
                    BT::OutputPort<std::chrono::seconds>("CommandInterval")
            };
        }

        BT::NodeStatus tick() override {
            std::uint8_t DestinationID{};
            std::uint16_t SelfHealth{};
            UnitTeam MyTeam{};
            if (!getInput("DestinationID", DestinationID)) {
                // throw BT::RuntimeError("Missing required input [DestinationID]");
                /// spdlog::error("{}> Missing required input [DestinationID]", BTName);
            }
            if (!getInput("SelfHealth", SelfHealth)) {
                // throw BT::RuntimeError("Missing required input [SelfHealth]");
                /// spdlog::error("{}> Missing required input [SelfHealth]", BTName);
            }
            if (!getInput("MyTeam", MyTeam)) {
                // throw BT::RuntimeError("Missing required input [MyTeam]");
                /// spdlog::error("{}> Missing required input [MyTeam]", BTName);
            }
            if (DestinationID != Home(MyTeam)) return BT::NodeStatus::SUCCESS;
            if (SelfHealth > 380) return BT::NodeStatus::SUCCESS;
//        if (SelfHealth < 300){
//            setOutput("CommandInterval", std::chrono::seconds{5});
//            return BT::NodeStatus::FAILURE;
//        }
            return BT::NodeStatus::FAILURE;
        }
    };


/*
 * 用于计数除法，使用方法如下
 * <CountDiv
 *     count_div="2"
 *     Count="{Count}"
 * />
 */
// TODO 逻辑需要修改
    class CountDiv : public BT::ConditionNode {
    public:
        CountDiv(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<int>("count_div"),
                    BT::InputPort<int>("Count")
            };
        }
        BT::NodeStatus tick() override {

            /// spdlog::info("{}> CountDiv", BTName);

            int count_div{}, Count{};
            if (!getInput("count_div", count_div)) {
                // throw BT::RuntimeError("Missing required input [count_div]");
                /// spdlog::error("{}> Missing required input [count_div]", BTName);
            }
            if (!getInput("Count", Count)) {
                // throw BT::RuntimeError("Missing required input[Count]");
            }
            return Count % count_div == 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
    };


/*
 * 用于检查命令间隔，使用方法如下
 * <CheckCommandInterval
 *     LastCommandTime="{LastCommandTime}"
 *     CommandInterval="{CommandInterval}"
 * />
 */
    class CheckCommandInterval : public BT::ConditionNode {
    public:
        CheckCommandInterval(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<std::chrono::steady_clock::time_point>("LastCommandTime"),
                    BT::InputPort<std::chrono::seconds>("CommandInterval")
            };
        }

        BT::NodeStatus tick() override {

            /// spdlog::info("{}> CheckCommandInterval", BTName);

            std::chrono::steady_clock::time_point LastCommandTime{};
            std::chrono::seconds CommandInterval{};
            if (!getInput("LastCommandTime", LastCommandTime)) {
                // throw BT::RuntimeError("Missing required input [LastCommandTime]");
                /// spdlog::error("{}> Missing required input [LastCommandTime]", BTName);
            }
            if (!getInput("CommandInterval", CommandInterval)) {
                // throw BT::RuntimeError("Missing required input [CommandInterval]");
                /// spdlog::error("{}> Missing required input [CommandInterval]", BTName);
            }

            auto time_diff = std::chrono::steady_clock::now() - LastCommandTime;

            time_diff = std::chrono::duration_cast<std::chrono::seconds>(time_diff);
            std::cout << "time_diff: " << time_diff.count() << "CommandInterval" << CommandInterval.count() << std::endl;

            if (time_diff >= CommandInterval){
                std::cout << "Success" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }
    };


/*
 * 用于判断当前环境是否安全，使用方法如下
 * <IsSafe
 *     AmmoLeft="{AmmoLeft}"
 *     TimeLeft="{TimeLeft}"
 *     SelfHealth="{SelfHealth}"
 * />
 */
    class ConditionIsSafe : public BT::ConditionNode {
    public:
        ConditionIsSafe(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<std::uint16_t>("AmmoLeft"),
                    BT::InputPort<std::uint16_t>("TimeLeft"),
                    BT::InputPort<std::uint16_t>("SelfHealth")
            };
        }

        BT::NodeStatus tick() override {
            // TODO: 添加判断逻辑
            return BT::NodeStatus::SUCCESS;
        }
    };


/*
 * 用于判断是否低血量，使用方法如下
 * <IsLowHealth
 *     SelfHealth="{SelfHealth}"
 * />
 */
    class IsLowHealth : public BT::ConditionNode {
    public:
        IsLowHealth(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<std::uint16_t>("SelfHealth")
            };
        }

        BT::NodeStatus tick() override {
            std::uint16_t SelfHealth{};
            if (!getInput("SelfHealth", SelfHealth)) {
                // throw BT::RuntimeError("Missing required input [SelfHealth]");
                /// spdlog::error("{}> Missing required input [SelfHealth]", BTName);
            }
            /// spdlog::info("{}> IsLowHealth: 当前血量为{}", BTName, SelfHealth);
            if(SelfHealth <= LowSelfHealthThreshold) return BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::FAILURE;
        }
    };


/*
 * 用于判断是否低弹药，使用方法如下
 * <IsLowAmmo
 *     AmmoLeft="{AmmoLeft}"
 * />
 */
    class IsLowAmmo : public BT::ConditionNode {
    public:
        IsLowAmmo(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<std::uint16_t>("AmmoLeft")
            };
        }

        BT::NodeStatus tick() override {
            std::uint16_t AmmoLeft{};
            if (!getInput("AmmoLeft", AmmoLeft)) {
                // throw BT::RuntimeError("Missing required input [AmmoLeft]");
                /// spdlog::error("{}> Missing required input [AmmoLeft]", BTName);
            }
            if(AmmoLeft <= LowAmmoThreshold) return BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::FAILURE;
        }
    };

/*
 * 用于判断是否剩余时间过少，使用方法如下
 * <IsLowTime
 *     TimeLeftThreshold="60"
 *     TimeLeft="{TimeLeft}"
 * />
 */
    class IsLowTime : public BT::ConditionNode {
    public:
        IsLowTime(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}

        static BT::PortsList providedPorts() {
            return {
                    BT::InputPort<std::uint16_t>("TimeLeftThreshold"),
                    BT::InputPort<std::uint16_t>("TimeLeft")
            };
        }

        BT::NodeStatus tick() override {
            std::uint16_t TimeLeft{}, TimeLeftThreshold{};
            if (!getInput("TimeLeft", TimeLeft)) {
                //throw BT::RuntimeError("Missing required input [TimeLeft]");
                /// spdlog::error("{}> Missing required input [TimeLeft]", BTName);
            }
            if (!getInput("TimeLeftThreshold", TimeLeftThreshold)) {
                //throw BT::RuntimeError("Missing required input [TimeLeftThreshold]");
                /// spdlog::error("{}> Missing required input [TimeLeftThreshold]", BTName);
            }
            if(TimeLeft <= TimeLeftThreshold) return BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::FAILURE;
        }
    };

}


namespace BT {

    using namespace LangYa;

// 模板化类型转换函数

    template<> inline UnitType convertFromString(StringView str) {
        if (str == "Hero") return UnitType::Hero;
        if (str == "Engineer") return UnitType::Engineer;
        if (str == "Infantry1") return UnitType::Infantry1;
        if (str == "Infantry2") return UnitType::Infantry2;
        if (str == "Infantry3") return UnitType::Infantry3;
        if (str == "Drone") return UnitType::Drone;
        if (str == "Sentry") return UnitType::Sentry;
        if (str == "Dart") return UnitType::Dart;
        if (str == "Radar") return UnitType::Radar;
        return UnitType::Unknown;
    }

    template<> inline ArmorType convertFromString(StringView str) {
        if (str == "Base") return ArmorType::Base;
        if (str == "Hero") return ArmorType::Hero;
        if (str == "Engineer") return ArmorType::Engineer;
        if (str == "Infantry1") return ArmorType::Infantry1;
        if (str == "Infantry2") return ArmorType::Infantry2;
        if (str == "Infantry3") return ArmorType::Infantry3;
        if (str == "Sentry") return ArmorType::Sentry;
        if (str == "Outpost") return ArmorType::Outpost;
        return ArmorType::UnKnown;
    }

    template<> inline UnitTeam convertFromString(StringView str) {
        if (str == "Red") return UnitTeam::Red;
        if (str == "Blue") return UnitTeam::Blue;
        if (str == "Other") return UnitTeam::Other;
        return UnitTeam::Unknown;
    }

    template<> inline NaviTeam convertFromString(StringView str) {
        if (str == "Enemy") return NaviTeam::Enemy;
        if (str == "Myself") return NaviTeam::Myself;
        if (str == "Other") return NaviTeam::Other;
        return NaviTeam::Unknown;
    }
    template<> inline TeamedLocation convertFromString(StringView str) {
        if (str == "Home") return LangYa::Home;
        if (str == "Recovery") return LangYa::Recovery;
        if (str == "Castle") return LangYa::Castle;
        if (str == "CastleLeft") return LangYa::CastleLeft;
        if (str == "CastleRight") return LangYa::CastleRight1; // CastleRight 已重命名為 CastleRight1
        if (str == "FlyDefense") return LangYa::FlyRoad;       // FlyDefense 已重命名為 FlyRoad
        if (str == "OutpostArea") return LangYa::OutpostArea;
        if (str == "MidShoot") return LangYa::MidShoot;
        if (str == "LeftShoot") return LangYa::LeftShoot;
        if (str == "OccupyArea") return LangYa::OccupyArea;
        return LangYa::Home; // 默認返回 Home
    }

// position="Home;Myself"
// position="Outpost;Enemy"
    template<> inline NaviPosition convertFromString(StringView str) {
        auto part = splitString(str, ';');
        if (part.size() != 2) {
            throw std::invalid_argument("Invalid string format");
        } else {
            NaviPosition position;
            position.Location = convertFromString<TeamedLocation>(part[0]);
            position.Team = convertFromString<NaviTeam>(part[1]);
            return position;
        }
    }   

}
