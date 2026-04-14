// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

// [BT v4] 
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
// bt_zmq_publisher.h 在 BT.CPP v4 已移除，改用 groot2_publisher
#include <behaviortree_cpp/loggers/groot2_publisher.h>

// [ROS 2]
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <deque>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>

#include "Utils/Logger.hpp"

#include "../module/BasicTypes.hpp"
#include "../module/SineWave.hpp"
#include "../module/json.hpp"
#include "../module/Rate.hpp"
#include "../module/Counter.hpp"
#include "../module/Random.hpp"
#include "../module/Area.hpp"
#include "../module/ROSTools.hpp"

#include "Node.hpp"
#include "Topic.hpp"
#include "Robot.hpp"
#include "PostureManager.hpp"

using namespace BT;
using namespace LangYa;
using namespace Utils::Logger;
using json = nlohmann::json;

namespace BehaviorTree {

enum class RuntimeFaultCode : std::uint8_t {
    None = 0,
    LoopStall = 1,
    TickStall = 2,
    TreeEmpty = 3,
    TreeException = 4
};

enum class StrategyMode : std::uint8_t {
    HitSentry = 0,
    HitHero = 1,
    Protected = 2,
    NaviTest = 3,
    LeagueSimple = 4
};

inline const char* StrategyModeToString(const StrategyMode mode) {
    switch (mode) {
        case StrategyMode::HitSentry: return "HitSentry";
        case StrategyMode::HitHero: return "HitHero";
        case StrategyMode::Protected: return "Protected";
        case StrategyMode::NaviTest: return "NaviTest";
        case StrategyMode::LeagueSimple: return "LeagueSimple";
        default: return "Unknown";
    }
}

enum class CompetitionProfile : std::uint8_t {
    Regional = 0,
    League = 1
};

inline const char* CompetitionProfileToString(const CompetitionProfile profile) {
    switch (profile) {
        case CompetitionProfile::Regional: return "regional";
        case CompetitionProfile::League: return "league";
        default: return "regional";
    }
}

    #define SET_POSITION(area, team) \
    do { \
        naviCommandGoal = LangYa::area(team); \
        naviGoalPosition = BehaviorTree::Area::area(team); \
    } while(0)


class Application {
public:
    inline static constexpr const char nodeName[] = "behavior_tree";
    
    // [修改] 改為 std::string，在 .cpp 構造函數中賦值，解決相對路徑問題
    std::string behavior_tree_file_;
    std::string config_file_;

private:
    // [ROS 2] 節點指針
    std::shared_ptr<rclcpp::Node> node_;

    SineWave pitch_wave{5.0f, 0.0f, 500ms, std::chrono::steady_clock::now()};

    int buff_shoot_count = 0; // 打符的击打次数

    /// 决策需要的数据
    UnitTeam team{UnitTeam::Red}; // 当前队伍颜色
    std::uint16_t enemyOutpostHealth{0}; // 敌方前哨站血量
    std::uint16_t selfOutpostHealth{0}; // 我方前哨站血量
    std::uint16_t enemyBaseHealth{0};  // 基地血量
    std::uint16_t selfBaseHealth{0};
    std::uint16_t ammoLeft{0}; // 剩余子弹数
    std::uint16_t timeLeft{0}; // 比赛剩余时间
    std::uint16_t myselfHealth{0}; // 自己的血量
    bool hasReceivedMyselfHealth_{false};
    bool hasReceivedAmmoLeft_{false};
    bool hasReceivedGameStartFlag_{false};
    std::chrono::steady_clock::time_point lastMyselfHealthRxTime{};
    std::chrono::steady_clock::time_point lastAmmoLeftRxTime{};
    std::chrono::steady_clock::time_point lastGameStartRxTime{};
    
    Robots friendRobots; // 己方机器人的信息
    Robots enemyRobots; // 敌方机器人的信息
    BuffType teamBuff{0}; // 当前的增益情况
    std::uint32_t rfidStatus{0}; // 当前的RFID识别状态
    std::uint32_t extEventData{};
    std::array<ArmorData, 10> armorList; // 辅瞄返回的装甲板序列
    bool is_game_begin{false}; // 比赛开始的标志
    FireCodeType RecFireCode{}; // 云台的火控数据
    std::uint8_t postureState{0}; // 云台/下位机回传姿态: 0=未知, 1=进攻, 2=防御, 3=移动
    std::uint8_t capV;
    std::uint8_t naviLowerHead{false};


    /// 决策修改控制数据需要的前置数据
    std::chrono::steady_clock::time_point gameStartTime{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point lastFoundEnemyTime{std::chrono::steady_clock::now()}; 
    GimbalAnglesType gimbalAngles{0, 0}; 
    std::int16_t gimbalYawVelRaw{0};   // from /ly/gimbal/d_vel.x (new protocol raw int16)
    float gimbalYawVelDegPerSec{0.0f};
    std::int16_t gimbalYawAngleRaw{0}; // from /ly/gimbal/d_vel.y (new protocol raw int16)
    float gimbalYawAngleDeg{0.0f};
    std::atomic<bool> hasReceivedGimbalAngles_{false};
    std::chrono::steady_clock::time_point lastGimbalAnglesRxTime{};
    std::vector<UnitType> reliableEnemyPosuition; 
    std::vector<UnitType> hitableTargets; 

    // ==========================================
    // [恢復] 核心決策控制變數
    // ==========================================
    /// 决策的控制数据: 云台，火控, 导航, 击打目标
    AimMode aimMode{AimMode::RotateScan};
    // ArmorType targetArmor{ArmorType::Hero}; // 目标装甲板
    ArmorData targetArmor{}; // 目标装甲板，包括距离
    AimData autoAimData{}; // 定义回调，接收的辅瞄云台角度数据
    AimData buffAimData{}; // 定义回调，接收的打符云台角度数据
    AimData outpostAimData{}; // 定义回调，接收的打哨站云台角度数据
    GimbalControlData gimbalControlData{}; /// 发送给云台的角度控制数据，火控数据等
    std::uint8_t postureCommand{0}; // 姿态控制指令: 0=不下发, 1=进攻, 2=防御, 3=移动
    std::atomic<bool> isFindTargetAtomic{false}; // 在回调函数中，每接收一次消息就会被置为true，然后在发送完控制数据之后置为false
    std::chrono::steady_clock::time_point lastTargetSeenTime{}; // 最近一次收到目标回调
    std::chrono::steady_clock::time_point lastDamageTime{};     // 最近一次检测到掉血
    struct DamageSample {
        std::chrono::steady_clock::time_point Time{};
        std::uint16_t Delta{0};
    };
    std::deque<DamageSample> postureRecentDamageSamples_{};
    std::chrono::steady_clock::time_point lastDamageBurstTime_{};
    bool postureHealthInitialized_{false};
    std::uint16_t postureLastHealth_{0};
    SentryPosture postureLastDesired_{SentryPosture::Unknown};
    std::string postureLastReason_{"init"};

    std::uint8_t naviCommandGoal{0}; // 导航目标
    Area::Point<std::uint16_t> naviGoalPosition{}; // 导航定位目标
    std::uint8_t lastNaviComnamdGoal{0}; // 上一次导航目标
    VelocityType naviVelocityInput{0, 0}; /// 外部导航输入速度（/ly/navi/vel）
    VelocityType naviVelocity{0, 0}; /// 定义回调，接收导航的速度控制数据
    bool naviRelativeTargetValid{false};
    float naviRelativeTargetX{0.0F};
    float naviRelativeTargetY{0.0F};
    float naviRelativeTargetZ{0.0F};
    float naviRelativeTargetDistance{0.0F};
    float naviRelativeTargetYawErrorDeg{0.0F};
    float naviRelativeTargetPitchErrorDeg{0.0F};
    std::uint8_t naviRelativeTargetArmorType{0U};
    std::uint8_t naviRelativeTargetAimMode{0U};
    TimerClock naviCommandIntervalClock{Seconds{10}}, recoveryClock{Seconds{90}}; // 控制间隔，回家时间 
    std::uint8_t speedLevel{1}; // 0 没电, 1 正常, 2 快速
    StrategyMode strategyMode_{StrategyMode::HitHero}; // 当前策略
    CompetitionProfile competitionProfile_{CompetitionProfile::Regional};
    std::string competitionProfileOverride_{};
    bool debugBypassGameStart_{false};
    bool publishNaviGoal_{true};
    int waitForGameStartTimeoutSec_{0};
    int leagueRefereeStaleTimeoutMs_{0};
    bool leagueRecoveryActive_{false};
    std::chrono::steady_clock::time_point leagueRecoveryStartTime_{};
    std::chrono::steady_clock::time_point leagueRecoveryReach350Time_{};
    std::chrono::steady_clock::time_point leagueRecoveryLastIncreaseTime_{};
    std::chrono::steady_clock::time_point leagueRecoveryCooldownUntil_{};
    std::uint16_t leagueRecoveryEntryHealth_{0};
    std::uint16_t leagueRecoveryPeakHealth_{0};
    std::size_t leaguePatrolGoalIndex_{0};
    bool leaguePatrolGoalInitialized_{false};
    std::size_t showcasePatrolGoalIndex_{0};
    bool showcasePatrolGoalInitialized_{false};
    std::size_t naviDebugGoalIndex_{0};
    bool naviDebugGoalInitialized_{false};
    bool leagueRouteCompatAfterGatePending_{false};
    bool leagueRouteCompatActive_{false};
    std::chrono::steady_clock::time_point leagueRouteCompatUntil_{};
    bool leagueRouteCompatHasPendingGoal_{false};
    std::uint8_t leagueRouteCompatPendingBaseGoal_{LangYa::Home.ID};
    int leagueRouteCompatPendingHoldSec_{1};
    std::chrono::steady_clock::time_point lastLeagueRecoveryGuardLogTime_{};
    std::chrono::steady_clock::time_point lastPositionDataGuardLogTime_{};

    // ==========================================
    // Runtime Guard (L1/L2)
    // ==========================================
    std::thread runtimeGuardThread_{};
    std::atomic<bool> runtimeGuardStop_{false};
    std::atomic<bool> runtimeRecoveryRequested_{false};
    std::atomic<RuntimeFaultCode> runtimeFaultCode_{RuntimeFaultCode::None};
    std::atomic<bool> runtimeTickInProgress_{false};
    std::atomic<std::int64_t> runtimeLastLoopBeatNs_{0};
    std::atomic<std::int64_t> runtimeTickStartNs_{0};
    std::atomic<std::int64_t> runtimeTickEndNs_{0};
    std::atomic<std::int64_t> runtimeLastSafePublishNs_{0};
    std::mutex runtimeRecoveryMutex_{};
    bool runtimeRecovering_{false};
    std::chrono::steady_clock::time_point runtimeRecoveryWindowStart_{};
    std::chrono::steady_clock::time_point runtimeLastSoftRecoverTime_{};
    int runtimeRecoveryCountInWindow_{0};
    static constexpr int kRuntimeLoopStallMs = 1500;
    static constexpr int kRuntimeTickStallMs = 800;
    static constexpr int kRuntimeGimbalStaleMs = 1200;
    static constexpr int kRuntimeRecoveryWindowSec = 60;
    static constexpr int kRuntimeRecoveryLimit = 3;
    static constexpr int kRuntimeRecoveryMinIntervalMs = 1500;
    static constexpr int kRuntimeSafePublishMinIntervalMs = 200;

    BT::Blackboard::Ptr GlobalBlackboard_ = BT::Blackboard::create(); // 跨 tick 持久黑板
    BT::Blackboard::Ptr TickBlackboard_ = BT::Blackboard::create();   // 每次 tick 中间黑板
    BT::BehaviorTreeFactory Factory{}; // 行为树工厂
    BT::Tree BTree{}; // 行为树
    std::unique_ptr<BT::FileLogger2> btFileLogger_; // bt_file_logger_v2
    std::unique_ptr<BT::Groot2Publisher> btGrootPublisher_; // Groot2
    // ==========================================

    std::shared_ptr<Logger> LoggerPtr; // 日志

    Config config{}; // 配置文件
    PostureManager postureManager_{};
    std::chrono::steady_clock::time_point lastUpdateBlackboardLogTime_{};
    std::chrono::steady_clock::time_point lastTreeTickLogTime_{};
    std::chrono::steady_clock::time_point lastTransportLogTime_{};

    RateClock fireRateClock{20}, treeTickRateClock{100}, naviCommandRateClock{2}; // 频率控制
    TimerClock rotateTimerClock{Seconds{2}}; // 旋转时间
    DescentDetector<std::uint16_t>  healthDecreaseDetector{400}; // 血量丢失检测器


    // ==========================================
    // [ROS 2] 通訊管理
    // ==========================================
    // 必須保存指針以防斷連
    std::vector<std::shared_ptr<void>> subscribers_;
    std::vector<std::shared_ptr<void>> publishers_;

    // [保留] 回调管理 (為了接口兼容性保留)
    MultiCallback<Application&> callbacks{*this};

    // [ROS 2] 訂閱生成器 — 接受兩參數 lambda: [](Application& app, MsgSharedPtr msg){}
    template<typename TTopic>
    void GenSub(std::function<void(Application&, typename TTopic::CallbackArg)> callback) {
        using MsgType = typename TTopic::Msg;
        std::string topic_name = TTopic::Name;

        auto sub = node_->create_subscription<MsgType>(
            topic_name,
            rclcpp::QoS(10),
            [this, callback](const typename MsgType::SharedPtr msg) {
                callback(*this, msg);
            }
        );
        subscribers_.push_back(sub);
    }
    
    // [新增] 發布生成器
    template<typename TTopic>
    auto GenPub() {
        using MsgType = typename TTopic::Type;
        auto pub = node_->create_publisher<MsgType>(TTopic::Name, rclcpp::QoS(10));
        publishers_.push_back(pub); 
        return pub;
    }

    // [ROS 2] 發布者指針 (明確類型)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_aa_enable_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ra_enable_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_outpost_enable_;

    rclcpp::Publisher<gimbal_driver::msg::GimbalAngles>::SharedPtr pub_gimbal_control_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gimbal_firecode_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gimbal_posture_;
    rclcpp::Publisher<gimbal_driver::msg::Vel>::SharedPtr pub_gimbal_vel_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gimbal_capV_;

    rclcpp::Publisher<auto_aim_common::msg::Target>::SharedPtr pub_predictor_target_;
    
    rclcpp::Publisher<gimbal_driver::msg::Vel>::SharedPtr pub_navi_vel_;
    rclcpp::Publisher<auto_aim_common::msg::RelativeTarget>::SharedPtr pub_navi_target_rel_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_navi_goal_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_navi_goal_pos_raw_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_navi_goal_pos_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_navi_speed_level_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_navi_lower_head_;
    
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_bt_target_;

    void RecordDamageSample(std::chrono::steady_clock::time_point now, std::uint16_t damage);


public:
    void SubscribeMessageAll();
    void PrintMessageAll();

    // 发布消息
    void PublishMessageAll();
    void PubAimModeEnableData();
    void PubGimbalControlData();
    void PubPostureControlData();
    void PubAimTargetData();
    void PubNaviControlData();
    void PubNaviRelativeTarget();
    void PubNaviGoal();
    void PubNaviGoalPos();


    // 等待比赛开始
    void WaitForGameStart();
    void WaitBeforeGame();

    //  比赛循环
    void GameLoop();

    /**
     * @brief 从黑板获取数据 \n
     */
    template<typename T>
    T GetInfoFromBlackBoard(const std::string &key) {
        T value{};
        bool ValueExist = GlobalBlackboard_->get<T>(key, value);
        if (!ValueExist) {
            if(LoggerPtr) LoggerPtr->Error("Blackboard key {} not found", key.c_str());
            else RCLCPP_ERROR(node_->get_logger(), "Blackboard key %s not found", key.c_str());
        }
        return value;
    }
    void UpdateBlackBoard();
    void TransportData();
    void PublishTogether();
    void TreeTick();
    void TreeTickGuarded();
    void ProcessData();
    bool CheckPositionRecovery();
    void SetPositionRepeat();
    void SetPositionProtect();
    void SetPositionNaviTest();
    void SetPositionHitSentry();
    void SetPositionHitHero();
    void SetPositionLeagueSimple();
    void SetPositionShowcasePatrol();
    void SetPositionNaviDebugPlan();
    bool IsLeagueRouteCompatEnabled() const noexcept;
    bool IsLeagueGoalSwitchBetween2And3(
        std::uint8_t from_goal_id,
        std::uint8_t to_goal_id,
        UnitTeam goal_team,
        bool apply_team_offset) const noexcept;
    bool TickLeagueRouteCompat(UnitTeam goal_team, bool apply_team_offset);
    void StartLeagueRouteCompat(
        std::uint8_t pending_base_goal,
        int pending_hold_sec,
        UnitTeam goal_team,
        bool apply_team_offset,
        const char* reason);
    void SetPositionByBaseGoal(std::uint8_t base_goal_id, UnitTeam team, bool apply_team_offset = true);
    std::uint8_t ResolveGoalId(std::uint8_t base_goal_id, UnitTeam team, bool apply_team_offset = true) const noexcept;
    void SetAimTarget();
    void SetAimTargetNormal();
    void SetAimMode();
    void CheckDebug();
    void UpdatePostureCommand(bool has_target);
    SentryPosture SelectDesiredPosture(bool has_target) const;
    bool HasRecentTarget() const;
    bool IsUnderFireRecent() const;
    bool IsUnderFireBurst() const;

    // 行为树初始化
    bool LoadBehaviorTree() noexcept;
    bool RegisterTreeNodes();

    // 初始化地图
    void InitMap();

    // 日志初始化
    bool InitLogger();

    // 获取配置文件
    bool ConfigurationInit();
    void StartRuntimeGuard();
    void StopRuntimeGuard();
    void RuntimeGuardLoop();
    void RequestSoftRecovery(RuntimeFaultCode code) noexcept;
    bool TryHandleSoftRecovery();
    bool TrySoftReloadBehaviorTree(RuntimeFaultCode code);
    bool IsCriticalInputStale() const;
    void PublishSafeControl(const char* reason, bool from_guard_thread = false) noexcept;
    void MarkLoopBeat() noexcept;
    void MarkTickStart() noexcept;
    void MarkTickEnd() noexcept;
    static std::int64_t NowSteadyNs() noexcept;
    static const char* RuntimeFaultCodeToString(RuntimeFaultCode code) noexcept;

    StrategyMode GetStrategyMode() const noexcept { return strategyMode_; }
    void SetStrategyMode(const StrategyMode mode) noexcept { strategyMode_ = mode; }
    CompetitionProfile GetCompetitionProfile() const noexcept { return competitionProfile_; }
    void SetCompetitionProfile(const CompetitionProfile profile) noexcept { competitionProfile_ = profile; }
    bool IsLeagueProfile() const noexcept { return competitionProfile_ == CompetitionProfile::League; }
    bool IsShowcasePatrolEnabled() const noexcept { return config.ShowcasePatrolSettings.Enable; }
    bool IsNaviDebugEnabled() const noexcept { return config.NaviDebugSettings.Enable; }
    AimMode GetAimMode() const noexcept { return aimMode; }
    BT::Blackboard::Ptr GetGlobalBlackboard() const noexcept { return GlobalBlackboard_; }
    BT::Blackboard::Ptr GetTickBlackboard() const noexcept { return TickBlackboard_; }
    void ResetTickBlackboard() {
        TickBlackboard_ = BT::Blackboard::create();
        if (GlobalBlackboard_) {
            GlobalBlackboard_->set("TickBlackboard", TickBlackboard_);
        }
    }
    int ElapsedSeconds() const {
        return static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - gameStartTime).count());
    }
    std::vector<UnitType> GetHitableTargetsCopy() const { return hitableTargets; }
    std::vector<UnitType> GetReliableEnemyPositionsCopy() const { return reliableEnemyPosuition; }
    ArmorData GetTargetArmorCopy() const { return targetArmor; }
    std::uint8_t GetPostureCommand() const noexcept { return postureCommand; }
    std::uint8_t GetPostureState() const noexcept { return postureState; }
    const PostureRuntime& GetPostureRuntime() const noexcept { return postureManager_.Runtime(); }

    Application(int argc, char **argv);
    ~Application();
    void Run();
};
}
