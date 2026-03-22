// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <chrono>
#include <cstdint>
#include <array>
#include <numbers>
#include <string>
#include <vector>

#pragma region Enums
namespace LangYa
{

#pragma region RobamasterTypes
    enum class UnitType : std::uint8_t
    {
        Unknown = 0,
        Hero = Unknown + 1,
        Engineer = Hero + 1,
        Infantry1 = Engineer + 1,
        Infantry2 = Infantry1 + 1,
        Infantry3 = Infantry2 + 1,
        Drone = Infantry3 + 1,
        Sentry = Drone + 1,
        Dart = Sentry + 1,
        Radar = Dart + 1
    };
    static std::vector<UnitType> RobotLists = {UnitType::Hero, UnitType::Engineer, UnitType::Infantry1, UnitType::Infantry2, UnitType::Sentry};

    /// @brief 团队类型
    enum class UnitTeam : std::uint8_t
    {
        Unknown = 0,
        Red = Unknown + 1,
        Blue = Red + 1,
        Other = Blue + 1
    };

    /// @brief 与串口协议中相同的单位类型数值
    enum class AllUnitType : std::uint8_t
    {
        Unknown = 0,
        RedOffset = 0,
        BlueOffset = 100,

        // sc_X macro for unit definition
#define sc_unit_list 	\
		sc_X(Hero) 		\
		sc_X(Engineer) 	\
		sc_X(Infantry1) \
		sc_X(Infantry2) \
		sc_X(Infantry3) \
		sc_X(Drone) 	\
		sc_X(Sentry) 	\
		sc_X(Radar)
#define sc_team_unit(team, type) team##type = team##Offset + static_cast<std::uint8_t>(UnitType::type),  // NOLINT(bugprone-macro-parentheses)
#define sc_X(type) sc_team_unit(Red, type)
        sc_unit_list
#undef sc_X
#define sc_X(type) sc_team_unit(Blue, type)
        sc_unit_list
#undef sc_X
#undef sc_team_unit
#undef sc_uint_list
    };

#pragma endregion RobomasterTypes

#pragma region Armor
    /// @brief 机器学习中的类型--地面作战单位
    enum class ArmorType : std::uint8_t
    {
        Base = 0,
        Hero = Base + 1,
        Engineer = Hero + 1,
        Infantry1 = Engineer + 1,
        Infantry2 = Infantry1 + 1,
        Infantry3 = Infantry2 + 1,
        Sentry = Infantry3 + 1,
        Outpost = Sentry + 1,
        UnKnown = Outpost + 1
    };

#pragma endregion Armor

#pragma region Gimbaldata
    using AngleType = float;
    using Angle100Type = std::int16_t;
    using RadianType = float;

    inline RadianType ToRadian(const AngleType angle) noexcept { return angle * std::numbers::pi_v<float> / 180.0f; }

    inline AngleType ToAngle(const RadianType radian) noexcept { return radian * 180.0f / std::numbers::pi_v<float>; }
#pragma pack(push, 1)
    struct GimbalAnglesType {
        AngleType Yaw;
        AngleType Pitch;
    };

    struct VelocityType {
        std::int8_t X;
        std::int8_t Y;
    };

    struct UWBPositionType {
        std::int16_t X{0}; // 自己的x位置，单位cm
        std::int16_t Y{0}; // 自己的y位置，单位cm
    };

    struct FireCodeType
    {
        /// @brief 开火状态，所有位反转表示开火 0b00 <-> 0b11
        std::uint8_t FireStatus : 2 = 0;

        /// @brief 电容状态， 00:不用 01:轻度使用 10:重度使用
        std::uint8_t CapState : 2 = 0;

        /// @brief 钻洞模式和辐瞄模式, 1表示启用，0表示禁用
        std::uint8_t HoleMode : 1 = 0;
        std::uint8_t AimMode : 1 = 0;

        /// @brief 小陀螺状态，共四档，0 表示无速，1 表示低速， 2 表示中速， 3 表示高速
        std::uint8_t Rotate : 2 = 0;

        /// @brief 翻转开火标志位
        void FlipFireStatus() noexcept { FireStatus = FireStatus == 0 ? 0b11 : 0b00; }
    };

    /// @note 这里类型要求使用 @c std::uint16_t ，是因为在非 g++ 编译器上，此结构体的大小可能不符合预期
    struct GameCodeType {
        std::uint16_t IsGameBegin        : 1 = 0; // 比赛是否开始，0表示未开始，1表示开始
        std::uint16_t HeroPrecaution     : 1 = 0; // 防御英雄，暂时没有使用
        std::uint16_t IsMyTeamRed        : 1 = 0; // 我方颜色，0表示蓝方，1表示红方
        std::uint16_t EnemyOutpostHealth : 6 = 60; // 我方前哨站血量
        std::uint16_t SelfOutpostHealth  : 6 = 60; // 敌方前哨站血量
        std::uint16_t IsReturnedHome     : 1 = 0; // 是否返回基地，暂时没有使用
    };

    /// 场地事件数据，共4个字节
    struct ExtEventDataType
    {
        std::uint32_t SelfSupplyStatus : 3 = 0;
        std::uint32_t SelfBuffStatus : 3 = 0;
        std::uint32_t SelfHighlandStatus : 6 = 0;
        std::uint32_t SelfBaseShield : 7 = 0;
        std::uint32_t LastDratTime : 9 = 0;
        std::uint32_t DartTarget : 2 = 0;
        std::uint32_t GainPointstatus : 2 = 0;
    };

    struct GimbalData {
        static constexpr auto TypeID = 0;

        GimbalAnglesType GimbalAngles;  // 8个字节
        VelocityType Velocity; // 2个字节
        FireCodeType FireCode; // 1个字节
        std::uint8_t CapV; // 1个字节
    };

    struct GameData {
        static constexpr auto TypeID = 1;

        GameCodeType GameCode{};  // 2个字节
        std::uint16_t AmmoLeft{}; // 2个字节 // 剩余弹药
        std::uint16_t TimeLeft{};  // 2个字节 // 比赛剩余时间
        std::uint16_t SelfHealth{};  // 2个字节 // 自己的血量
        std::uint32_t ExtEventData{};  // 4个字节 // 场地数据
    };

    // 不要再结构体里面调用构造函数
    struct HealthMyselfData {
        static constexpr auto TypeID = 2;

        std::uint16_t HeroMyself{200};
        std::uint16_t EngineerMyself{};
        std::uint16_t Infantry1Myself{200};
        std::uint16_t Infantry2Myself{};
        std::uint16_t BaseMyself{};
        std::uint16_t SentryMyself{};
    };

    struct HealthEnemyData {
        static constexpr auto TypeID = 3;

        std::uint16_t HeroEnemy{};
        std::uint16_t EngineerEnemy{};
        std::uint16_t Infantry1Enemy{};
        std::uint16_t Infantry2Enemy{};
        std::uint16_t BaseEnemy{}; // 基地血量，分度值100，0-50
        std::uint16_t SentryEnemy{};
    };

    struct BuffType{
        std::uint8_t reserve;
        std::uint8_t RecoveryBuff; // 回血增益，百分比
        std::uint8_t CoolingBuff; // 热量冷却倍率，直接值
        std::uint8_t DefenceBuff; // 防御增益，百分比
        std::uint8_t VulnerabilityBuff; // 负防御增益，百分比
        std::uint16_t AttackBuff; // 攻击增益，百分比
        std::uint8_t RemainingEnergy; // 剩余能量值反馈
                                      // 十六进制，小于50%反馈，默认反馈0x32
                                      // 50%以上反馈0x1F，0b11111
                                      // 30%以上反馈0x1E，0b11110
                                      // 25%以上反馈0x1C，0b11100
                                      // 5%以上反馈0x18，0b11000
                                      // 1%以上反馈0x10，0b10000
    };

    struct RFIDAndBuffData{
        static constexpr auto TypeID = 4;

        BuffType BuffStatus;
        std::uint32_t RFIDStatus; // bit19：己方补给区（RMUL补给区，与兑换区不重叠）
                                  // bit20：己方补给区（与兑换区不重叠）
    };

    struct PositionType{
        uint8_t CarId; /// 兵种id，x表示我方，100+x表示敌方
        UWBPositionType Position; // 4个字节
    };

    struct PositionData{
        static constexpr auto TypeID = 5;

        PositionType Friend; // 5字节
        PositionType Enemy; // 5字节
        uint16_t reserve;
    };

    struct GimbalControlData {
        std::uint8_t HeadFlag{'!'};
        GimbalAnglesType GimbalAngles;
        FireCodeType FireCode;
        std::uint8_t Tail{0};
    };
#pragma pack(pop)


#pragma endregion Gimbaldata

#pragma region Navigation
    using namespace std::chrono_literals;

    struct TeamedLocation
    {
        static constexpr auto LocationCount = 20;

        std::uint8_t ID;

        std::uint8_t operator()(const UnitTeam team) const
        {
            return team == UnitTeam::Blue ? ID + LocationCount : ID;
        }
    };
    static constexpr TeamedLocation Home{ 0 };
    static constexpr TeamedLocation Base{ 1 };
    static constexpr TeamedLocation Recovery{ 2 };
    static constexpr TeamedLocation BuffShoot{ 3 };
    static constexpr TeamedLocation LeftHighLand{ 4 };
    static constexpr TeamedLocation CastleLeft{ 5 };
    static constexpr TeamedLocation Castle{ 6 };
    static constexpr TeamedLocation CastleRight1{ 7 };
    static constexpr TeamedLocation CastleRight2{ 8 };
    static constexpr TeamedLocation FlyRoad{ 9 };
    static constexpr TeamedLocation OutpostArea{ 10 };
    static constexpr TeamedLocation MidShoot{ 11 };
    static constexpr TeamedLocation LeftShoot{ 12 };
    static constexpr TeamedLocation OutpostShoot{ 13 };
    static constexpr TeamedLocation BuffAround1{ 14 };
    static constexpr TeamedLocation BuffAround2{ 15 };
    static constexpr TeamedLocation RightShoot{ 16 };
    static constexpr TeamedLocation HoleRoad{ 17 };
    static constexpr TeamedLocation OccupyArea{ 18 };

    /// @brief 团队类型
    enum class NaviTeam : std::uint8_t
    {
        Unknown = 0,
        Myself = Unknown + 1,
        Enemy = Myself + 1,
        Other = Enemy + 1
    };

    // 行为树用于记录导航信息
    struct NaviPosition
    {
        TeamedLocation Location{ Home };
        NaviTeam Team{ NaviTeam::Unknown };
    };

#pragma pack(push, 1)
    struct LocatorMessage final {
        struct RotationType {
            float W;
            float X;
            float Y;
            float Z;
        };

        RotationType Rotation{};

        struct LocationType {
            float X;
            float Y;
            float Z;
        };

        LocationType Location{};
        std::int8_t Neighbor{};
    };

    struct NaviCommandMessage {
        std::uint8_t Head{'!'};
        std::uint8_t DestinationID{};
        std::uint8_t CRC{0};
    };

    struct NaviControlMessage {
        std::uint8_t Head{'!'};
        std::uint8_t LocationID{0};
        VelocityType Velocity{};
        LocatorMessage Locator{};
        std::uint8_t CRC{0};
    };
#pragma pack(pop)

#pragma endregion Navigation

#pragma region Aim

     struct ArmorData { // 接收来自辅瞄的装甲板序列
        ArmorType Type{ArmorType::UnKnown}; // 装甲板类型
        float Distance{30.0}; // 距离, 单位：米
    };
    /**
     * @brief 决策发送给辐瞄的数据
     */
    struct AimTargetData {
        std::uint8_t Head{'!'};
        ArmorType Target{ArmorType::Hero}; // 击打目标
        std::uint8_t CRC{0};
    };

    struct AimData { // 打符或辅瞄模式接收的数据
        bool FireStatus{false}; // true 表示开火
        bool BuffFollow{false};  // 用于打符的跟随
        bool Valid{false};       // 当前模式可直接使用的完整 yaw/pitch 对
        bool Fresh{false};       // 本循环内是否收到新的一帧
        bool HasLatchedAngles{false}; // 是否有上一帧可继续保持的有效锁角
        std::chrono::steady_clock::time_point LastValidTime{}; // 上一次收到有效锁角的时间
        GimbalAnglesType Angles; // 云台的控制角度
    };
    enum class AimMode : std::uint8_t { // 瞄准模式
        None = 0,
        AutoAim = 1,
        RotateScan = 2,
        Buff = 3,
        Outpost = 4
    };
#pragma endregion Aim


#pragma region Configuration

    // 辅瞄调试
    struct AimDebug {
        bool StopFire{false};
        bool StopRotate{false};
        bool StopScan{false};
        bool HitOutpost{false};
        bool HitBuff{false};
        bool HitCar{false};
        bool FireRequireTargetStatus{true};
    };

    // 频率相关
    struct Rate {
        int FireRate{20};
        int TreeTickRate{100};
        int NaviCommandRate{1};
    };
    struct GameStrategy {
        bool HitOutpost{false}; // 是否击打前哨站
        int HitBuff{false};  // 击打前哨站时
        bool TestNavi{false}; // 是否测试导航
        bool HitSentry{false}; // 攻击哨兵
        bool Protected{false}; // 保守模式
    };
    struct NaviSetting {
        bool UseXY{true};
    };

    struct LeagueStrategySetting {
        bool UseHealthRecovery{true};
        std::uint16_t HealthRecoveryThreshold{100};
        bool UseAmmoRecovery{true};
        std::uint16_t AmmoRecoveryThreshold{30};
        bool DamageScanBoostEnable{true};
        std::uint16_t HealthRecoveryExitMin{350};
        std::uint16_t HealthRecoveryExitPreferred{400};
        int HealthRecoveryPlateauSec{2};
        int HealthRecoveryExitStableSec{1};
        int HealthRecoveryMaxHoldSec{12};
        int HealthRecoveryCooldownSec{20};
        bool EnableDamageOpenGate{false};
        std::uint16_t DamageOpenGateThreshold{30};
        std::uint8_t MainGoal{OccupyArea.ID};
        std::vector<std::uint8_t> PatrolGoals{};
        int GoalHoldSec{15};
    };

    struct ShowcasePatrolSetting {
        bool Enable{false};
        std::vector<std::uint8_t> Goals{};
        int GoalHoldSec{5};
        bool Random{false};
        bool DisableTeamOffset{false};
        bool IgnoreRecovery{false};
    };

    struct NaviDebugSetting {
        bool Enable{false};
        std::string PlanFile{};
        std::string ActivePlan{};
        std::vector<std::uint8_t> Goals{};
        int GoalHoldSec{5};
        bool Random{false};
        bool DisableTeamOffset{false};
        bool IgnoreRecovery{true};
        std::uint8_t SpeedLevel{1};
    };

    // 姿态模块配置
    struct PostureSetting {
        bool Enable{true};
        int SwitchCooldownSec{5};     // 规则: 姿态切换冷却
        int MaxSinglePostureSec{180}; // 规则: 单姿态累计超过该值会降档
        int EarlyRotateSec{165};      // 接近降档前提前轮换
        int MinHoldSec{10};           // 防抖: 最短保持时间
        int PendingAckTimeoutMs{600}; // 等待回读超时
        int RetryIntervalMs{300};     // 重试间隔
        int MaxRetryCount{3};         // 最大重试次数
        bool OptimisticAck{true};     // 回读缺失时是否乐观确认
        int TargetKeepMs{800};        // 目标短时丢失容忍，防止姿态抖动
        int DamageKeepSec{4};         // 最近受击保持时间窗口
        int DamageBurstWindowMs{0};   // 短时间受击统计窗口，0=关闭
        int DamageBurstThreshold{0};  // 窗口内累计掉血阈值，0=关闭
        int DamageBurstDefenseHoldSec{0}; // 触发短时重受击后的防守保持时长
        int LowHealthThreshold{120};  // 低血阈值
        int VeryLowHealthThreshold{80}; // 极低血阈值
        int LowAmmoThreshold{30};     // 低弹阈值
        int ScoreHysteresis{2};       // 姿态切换分差迟滞
    };

    // Main
    struct Config {
        AimDebug AimDebugSettings{};
        Rate RateSettings{};
        GameStrategy GameStrategySettings{};
        NaviSetting NaviSettings{};
        LeagueStrategySetting LeagueStrategySettings{};
        ShowcasePatrolSetting ShowcasePatrolSettings{};
        NaviDebugSetting NaviDebugSettings{};
        PostureSetting PostureSettings{};
        int ScanCounter{1};  /// 扫描模式计数器，一定值后Yaw动一次
        std::string CompetitionProfile{"regional"};
    };

#pragma endregion Configuration
}
#pragma endregion

//數據框架定義
