// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <optional>

using namespace LangYa;

namespace BehaviorTree {


    float normalize_angle_0_360(float angle) {
        float normalized = fmod(angle, 360.0f);
        if (normalized < 0)
            normalized += 360.0f;
        return normalized;
    }

    float normalize_angle_near(float angle, float reference) {
        return reference + static_cast<float>(std::remainder(angle - reference, 360.0f));
    }

    namespace {
    constexpr std::uint8_t kMaxBaseGoalId = LangYa::OccupyArea.ID;
    constexpr std::uint8_t kLeagueRouteCompatViaGoalBaseId = LangYa::LeftHighLand.ID;  // base goal id=4
    constexpr int kLeagueRouteCompatViaHoldSec = 5;
    // 丢 1~2 帧时保留锁角，避免抖动；时间过长会让云台“粘住旧目标”。
    constexpr auto kLostTargetHold = std::chrono::milliseconds(200);

    bool IsValidBaseGoalId(const std::uint8_t goal_id) {
        return goal_id <= kMaxBaseGoalId;
    }

    std::optional<ArmorType> ArmorTypeFromPriorityId(const int armor_type_id) {
        switch (static_cast<ArmorType>(armor_type_id)) {
            case ArmorType::Hero:
            case ArmorType::Engineer:
            case ArmorType::Infantry1:
            case ArmorType::Infantry2:
            case ArmorType::Sentry:
                return static_cast<ArmorType>(armor_type_id);
            default:
                return std::nullopt;
        }
    }

    std::optional<UnitType> UnitTypeFromArmorType(const ArmorType armor_type) {
        switch (armor_type) {
            case ArmorType::Hero:
                return UnitType::Hero;
            case ArmorType::Engineer:
                return UnitType::Engineer;
            case ArmorType::Infantry1:
                return UnitType::Infantry1;
            case ArmorType::Infantry2:
                return UnitType::Infantry2;
            case ArmorType::Sentry:
                return UnitType::Sentry;
            default:
                return std::nullopt;
        }
    }

    std::optional<ArmorType> ArmorTypeFromUnitType(const UnitType unit_type) {
        switch (unit_type) {
            case UnitType::Hero:
                return ArmorType::Hero;
            case UnitType::Engineer:
                return ArmorType::Engineer;
            case UnitType::Infantry1:
                return ArmorType::Infantry1;
            case UnitType::Infantry2:
                return ArmorType::Infantry2;
            case UnitType::Sentry:
                return ArmorType::Sentry;
            default:
                return std::nullopt;
        }
    }

    bool IsIgnoredArmorType(const std::vector<int>& ignore_list, const ArmorType armor_type) {
        return std::find(ignore_list.begin(),
                         ignore_list.end(),
                         static_cast<int>(armor_type)) != ignore_list.end();
    }

    bool IsIgnoredUnitType(const std::vector<int>& ignore_list, const UnitType unit_type) {
        const auto armor_type = ArmorTypeFromUnitType(unit_type);
        return armor_type.has_value() && IsIgnoredArmorType(ignore_list, *armor_type);
    }

    int ClampToInt8(const int value) {
        return std::clamp(value, -128, 127);
    }

    std::string NormalizeDecisionModule(std::string_view module) {
        std::string normalized(module);
        std::transform(normalized.begin(), normalized.end(), normalized.begin(),
            [](unsigned char c) {
                if (c == '-' || c == ' ') {
                    return '_';
                }
                return static_cast<char>(std::tolower(c));
            });
        return normalized;
    }

    bool HasAutonomyToken(const std::vector<std::string>& tokens, std::string_view expected) {
        const auto normalized_expected = NormalizeDecisionModule(expected);
        return std::find(tokens.begin(), tokens.end(), normalized_expected) != tokens.end();
    }

    std::optional<StrategyMode> StrategyModeFromAutonomyToken(const std::string& token) {
        if (token == "hithero" || token == "hit_hero" || token == "hero") {
            return StrategyMode::HitHero;
        }
        if (token == "hitsentry" || token == "hit_sentry" || token == "sentry") {
            return StrategyMode::HitSentry;
        }
        if (token == "protected" || token == "protect") {
            return StrategyMode::Protected;
        }
        if (token == "navitest" || token == "navi_test") {
            return StrategyMode::NaviTest;
        }
        if (token == "leaguesimple" || token == "league_simple") {
            return StrategyMode::LeagueSimple;
        }
        return std::nullopt;
    }

    std::optional<Area::MainAreaKind> MainAreaKindFromToken(std::string_view token) {
        const auto normalized = NormalizeDecisionModule(token);
        if (normalized == "base") {
            return Area::MainAreaKind::Base;
        }
        if (normalized == "highland" || normalized == "high_land" || normalized == "high") {
            return Area::MainAreaKind::Highland;
        }
        if (normalized == "roadland" || normalized == "road_land" || normalized == "road") {
            return Area::MainAreaKind::Roadland;
        }
        if (normalized == "central" || normalized == "center" ||
            normalized == "centre" || normalized == "middle") {
            return Area::MainAreaKind::Central;
        }
        return std::nullopt;
    }

    Area::Point<std::uint16_t> GoalPointByBaseId(const std::uint8_t base_goal_id, const UnitTeam goal_team) {
        switch (base_goal_id) {
            case LangYa::Home.ID: return BehaviorTree::Area::Home(goal_team);
            case LangYa::Base.ID: return BehaviorTree::Area::Base(goal_team);
            case LangYa::Recovery.ID: return BehaviorTree::Area::Recovery(goal_team);
            case LangYa::BuffShoot.ID: return BehaviorTree::Area::BuffShoot(goal_team);
            case LangYa::LeftHighLand.ID: return BehaviorTree::Area::LeftHighLand(goal_team);
            case LangYa::CastleLeft.ID: return BehaviorTree::Area::CastleLeft(goal_team);
            case LangYa::Castle.ID: return BehaviorTree::Area::Castle(goal_team);
            case LangYa::CastleRight1.ID: return BehaviorTree::Area::CastleRight1(goal_team);
            case LangYa::CastleRight2.ID: return BehaviorTree::Area::CastleRight2(goal_team);
            case LangYa::FlyRoad.ID: return BehaviorTree::Area::FlyRoad(goal_team);
            case LangYa::OutpostArea.ID: return BehaviorTree::Area::OutpostArea(goal_team);
            case LangYa::MidShoot.ID: return BehaviorTree::Area::MidShoot(goal_team);
            case LangYa::LeftShoot.ID: return BehaviorTree::Area::LeftShoot(goal_team);
            case LangYa::OutpostShoot.ID: return BehaviorTree::Area::OutpostShoot(goal_team);
            case LangYa::BuffAround1.ID: return BehaviorTree::Area::BuffAround1(goal_team);
            case LangYa::BuffAround2.ID: return BehaviorTree::Area::BuffAround2(goal_team);
            case LangYa::RightShoot.ID: return BehaviorTree::Area::RightShoot(goal_team);
            case LangYa::HoleRoad.ID: return BehaviorTree::Area::HoleRoad(goal_team);
            case LangYa::OccupyArea.ID: return BehaviorTree::Area::OccupyArea(goal_team);
            default: return BehaviorTree::Area::Home(goal_team);
        }
    }

    struct RuntimeNaviGoalCandidate {
        std::uint8_t BaseGoalId{LangYa::Home.ID};
        UnitTeam Team{UnitTeam::Red};
        double Bias{0.0};
    };
    }  // namespace

     /**
     * @brief 更新黑板数据 \n
     * @brief  更新数据从上到下依次是：我方颜色，敌方哨站血量，我方哨站血量，剩余弹药，比赛剩余时间 \n
     * @brief  自身血量，己方英雄血量，己方3号步兵血量，视野中的装甲板序列，是否找到目标 \n
     */
    void Application::UpdateBlackBoard() {

        std::uint16_t SelfHealth = myselfHealth;
        // 三路目标源统一折叠成一个 IsFindTarget，供 BT 和姿态模块复用。
        // 注意这里是“本拍是否有新鲜目标”，不是长期跟踪状态。
        const bool has_auto_target = autoAimData.Fresh && autoAimData.Valid;
        const bool has_buff_target = buffAimData.Fresh && buffAimData.Valid && buffAimData.BuffFollow;
        const bool has_outpost_target = outpostAimData.Fresh && outpostAimData.Valid;
        const bool IsFindTarget = has_auto_target || has_buff_target || has_outpost_target;

        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }

        // 将数据写入黑板
        GlobalBlackboard_->set<UnitTeam>("MyTeam", team);
        GlobalBlackboard_->set<std::uint16_t>("TimeLeft", timeLeft);
        GlobalBlackboard_->set<std::uint16_t>("SelfHealth", SelfHealth);
        GlobalBlackboard_->set<std::uint16_t>("AmmoLeft", ammoLeft);
        GlobalBlackboard_->set<Robots>("FriendRobots", friendRobots);
        GlobalBlackboard_->set<Robots>("EnemyRobots", enemyRobots);
        GlobalBlackboard_->set<std::uint16_t>("EnemyOutpostHealth", enemyOutpostHealth);
        GlobalBlackboard_->set<std::uint16_t>("SelfOutpostHealth", selfOutpostHealth);
        GlobalBlackboard_->set<std::uint16_t>("SelfBaseHealth", selfBaseHealth);
        GlobalBlackboard_->set<std::uint16_t>("EnemyBaseHealth", enemyBaseHealth);
        GlobalBlackboard_->set<std::uint32_t>("RfidStatus", rfidStatus);
        GlobalBlackboard_->set<std::uint32_t>("ExtEventData", extEventData);
        GlobalBlackboard_->set("ArmorList", armorList);
        GlobalBlackboard_->set("TeamBuff", teamBuff);
        GlobalBlackboard_->set<std::uint8_t>("AimMode", static_cast<std::uint8_t>(aimMode));
        GlobalBlackboard_->set<std::uint8_t>("NaviGoal", naviCommandGoal);
        GlobalBlackboard_->set<int>("BuffShootCount", buff_shoot_count);
        GlobalBlackboard_->set<std::uint8_t>("StrategyMode", static_cast<std::uint8_t>(strategyMode_));
        GlobalBlackboard_->set<std::chrono::steady_clock::time_point>("GameStartTime", gameStartTime);
        GlobalBlackboard_->set<bool>("IsFindTarget", IsFindTarget);
        GlobalBlackboard_->set<std::uint8_t>("PostureState", postureState);
        GlobalBlackboard_->set<std::uint8_t>("PostureCommand", postureCommand);
        GlobalBlackboard_->set<bool>("PostureUnderFireRecent", IsUnderFireRecent());
        GlobalBlackboard_->set<bool>("PostureUnderFireBurst", IsUnderFireBurst());
        GlobalBlackboard_->set<std::int16_t>("GimbalYawVelRaw", gimbalYawVelRaw);
        GlobalBlackboard_->set<float>("GimbalYawVelDegPerSec", gimbalYawVelDegPerSec);
        GlobalBlackboard_->set<std::int16_t>("GimbalYawAngleRaw", gimbalYawAngleRaw);
        GlobalBlackboard_->set<float>("GimbalYawAngleDeg", gimbalYawAngleDeg);

        const auto now = std::chrono::steady_clock::now();
        if (now - lastUpdateBlackboardLogTime_ > std::chrono::seconds(2)) {
            LoggerPtr->Debug("Blackboard updated: TimeLeft={}, SelfHealth={}, AmmoLeft={}, EnemyOutpostHealth={}, SelfOutpostHealth={}",
                timeLeft, SelfHealth, ammoLeft, enemyOutpostHealth, selfOutpostHealth);
            lastUpdateBlackboardLogTime_ = now;
        }
    }

    /**
     * @brief 从黑板获取数据,处理ros队列的消息并发布 \n
     * @brief 从黑板获取的有辐瞄击打目标，导航目的地
     */
    void Application::TransportData() {
        // targetArmor.Type = GetInfoFromBlackBoard<ArmorType>("AimTarget");
        // naviCommandGoal = GetInfoFromBlackBoard<std::uint8_t>("naviCommandGoal");
        // LoggerPtr->Info("AimTarget: {}, NaviGoal: {}", static_cast<int>(targetArmor.Type), static_cast<int>(naviCommandGoal));

        
        // targetArmor = ArmorType::Infantry1;
        // 设置数据内容
        PublishTogether();
        const auto now = std::chrono::steady_clock::now();
        if (now - lastTransportLogTime_ > std::chrono::seconds(1)) {
            PrintMessageAll();
            LoggerPtr->Debug("TransportData: published control data.");
            lastTransportLogTime_ = now;
        }
    }

     /**
     * @brief 实现PublishTogether \n
     * @brief 判断是否找到目标， 找到目标就发送目标数据 \n
     * @brief 否则经过一定时间之后， 将gimbalControlData的GimbalAngles均匀变化
     */
    void Application::PublishTogether() {

        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();
        static constexpr auto delta_yaw = 1.0f; //bt每tick單位
        static constexpr auto buff_yaw = -50.0f + 360.0f;
        static constexpr auto kPatrolScanYawStep = 9.0f * delta_yaw; //單向巡航每tick度數
        static constexpr auto kPatrolScanYawBoostStep = 10.0f * delta_yaw; //受擊加速
        static constexpr auto kPatrolScanPitchCenterDeg = 0.0f; //巡航pitch中心
        static constexpr auto kPatrolScanPitchHalfRangeDeg = 12.0f; //巡航pitch上下半幅
        static constexpr auto kPatrolScanPitchPeriodMs = 500.0f; //巡航pitch完整波形週期

        static constexpr auto kPatrolSwingYawStep = 1.0f * delta_yaw; //雙向巡航每tick度數
        static constexpr auto kPatrolSwingYawBoostStep = 1.1f * delta_yaw; //受擊加速
        static constexpr auto kPatrolSwingHalfRangeDeg = 30.0f; // mode2: 左右擺頭半幅
        static constexpr auto kPatrolSwingCenterDriftPerCycleDeg = -70.0f; // mode2: 每完整左右掃一圈，中心點右偏角度
        static constexpr auto kTwoPi = 6.2831853071795864769f;
        static constexpr int kDamageScanBoostWindowMs = 1300;
        static constexpr int kDamageScanYawPhaseMs = 160;

        
        // 小陀螺策略（老设计）：
        // 1) 受击后按 1 -> 2 -> 3 递进换档；
        // 2) 到 3 档后持续保持；
        // 3) 仅在一段时间未受击后，回落到 1 档。
        const auto rotate_now = std::chrono::steady_clock::now();
        static auto last_damage_rotate_time = std::chrono::steady_clock::time_point{};
        static auto rotate_ramp_start_time = std::chrono::steady_clock::time_point{};
        static bool rotate_under_fire = false;

        constexpr int kRotateNoHitTimeoutMs = 1800;
        constexpr int kRotateGear1HoldMs = 220;
        constexpr int kRotateGear2HoldMs = 220;

        if (healthDecreaseDetector.trigger(myselfHealth)) { // 血量减少
            last_damage_rotate_time = rotate_now;
            if (!rotate_under_fire) {
                rotate_under_fire = true;
                rotate_ramp_start_time = rotate_now;
            }
            rotateTimerClock.tick();
        }

        bool in_damage_rotate_window = false;
        int damage_rotate_elapsed_ms = -1;
        std::uint8_t rotate_gear = 1;

        if (last_damage_rotate_time.time_since_epoch().count() != 0) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                rotate_now - last_damage_rotate_time).count();
            damage_rotate_elapsed_ms = static_cast<int>(elapsed_ms);
            in_damage_rotate_window = elapsed_ms <= kDamageScanBoostWindowMs;
        }

        if (rotate_under_fire) {
            const auto no_hit_ms = (last_damage_rotate_time.time_since_epoch().count() == 0)
                ? kRotateNoHitTimeoutMs + 1
                : static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    rotate_now - last_damage_rotate_time).count());

            if (no_hit_ms > kRotateNoHitTimeoutMs) {
                rotate_under_fire = false;
                rotate_gear = 1;
            } else {
                const auto ramp_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    rotate_now - rotate_ramp_start_time).count());
                if (ramp_ms < kRotateGear1HoldMs) {
                    rotate_gear = 1;
                } else if (ramp_ms < (kRotateGear1HoldMs + kRotateGear2HoldMs)) {
                    rotate_gear = 2;
                } else {
                    rotate_gear = 3;
                }
            }
        }

        gimbalControlData.FireCode.Rotate = rotate_gear;
        if (config.AimDebugSettings.StopRotate) {
            // StopRotate=true means disable chassis spin output.
            gimbalControlData.FireCode.Rotate = 0;
        }

        static auto last_rotate_log = std::chrono::steady_clock::time_point{};
        if (now_time >= 0) {
            const auto log_now = std::chrono::steady_clock::now();
            if (log_now - last_rotate_log > std::chrono::seconds(2)) {
                LoggerPtr->Debug(
                    "Rotate Gear: {} (under_fire={} damage_elapsed_ms={} no_hit_timeout_ms={})",
                    gimbalControlData.FireCode.Rotate,
                    rotate_under_fire ? 1 : 0,
                    damage_rotate_elapsed_ms,
                    kRotateNoHitTimeoutMs);
                last_rotate_log = log_now;
            }
        }

        /*----------云台----------*/
        auto now = std::chrono::steady_clock::now();
        const AimData* activeAimData = &autoAimData;
        if (aimMode == AimMode::Buff) {
            activeAimData = &buffAimData;
        } else if (aimMode == AimMode::Outpost) {
            activeAimData = &outpostAimData;
        }
        GimbalAnglesType nextAngles = gimbalAngles;
        VelocityType nextVelocity = naviVelocityInput;
        const bool find_target = isFindTargetAtomic.load(std::memory_order_relaxed);
        const auto chase_mode_enabled = [&]() -> bool {
            if (!config.ChaseSettings.Enable || !config.ChaseSettings.FollowAimTarget) {
                return false;
            }
            switch (aimMode) {
                case AimMode::AutoAim:
                    return config.ChaseSettings.EnableInAutoAim;
                case AimMode::RotateScan:
                    return config.ChaseSettings.EnableInRotateScan;
                case AimMode::Outpost:
                    return config.ChaseSettings.EnableInOutpostMode;
                case AimMode::Buff:
                    return config.ChaseSettings.EnableInBuffMode;
                default:
                    return false;
            }
        }();
        naviRelativeTargetValid = false;
        naviRelativeTargetX = 0.0F;
        naviRelativeTargetY = 0.0F;
        naviRelativeTargetZ = 0.0F;
        naviRelativeTargetDistance = 0.0F;
        naviRelativeTargetYawErrorDeg = 0.0F;
        naviRelativeTargetPitchErrorDeg = 0.0F;
        naviRelativeTargetArmorType = 0U;
        naviRelativeTargetAimMode = static_cast<std::uint8_t>(aimMode);
        auto reset_patrol_scan_state = [this]() {
            patrolScanDirection_ = 1;
            patrolScanCenterYaw_ = 0.0f;
            patrolScanOffsetYaw_ = 0.0f;
            patrolScanPhaseRad_ = 0.0f;
            patrolScanCenterInitialized_ = false;
            patrolScanActiveMode_ = 0;
        };
        if (find_target) {
            reset_patrol_scan_state();
            LoggerPtr->Debug("Find Target, AimMode={}", static_cast<int>(aimMode));
            if (!config.AimDebugSettings.StopFire){
                if(aimMode == AimMode::Buff) { // 打符模式
                    if(activeAimData->FireStatus){
                        /// 立刻响应不需要tick
                        RecFireCode.FlipFireStatus();
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                        buffAimData.FireStatus = false;
                        buff_shoot_count++;
                    }
                } else { // 非打符模式（打前哨和打车），沿用老代码：收到回调就按频率开火
                    if(fireRateClock.trigger()){
                        fireRateClock.tick();
                        RecFireCode.FlipFireStatus();
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                    }
                }
            }
            gimbalControlData.FireCode.AimMode = 1;
            lastFoundEnemyTime = now;
            
            nextAngles = activeAimData->Angles;
            if (aimMode != AimMode::Buff && aimMode != AimMode::Outpost) {
                LoggerPtr->Debug("AutoAim Angles -> Pitch: {}, Yaw: {}", autoAimData.Angles.Pitch, autoAimData.Angles.Yaw);
            }
        }
        else { // 未识别到目标
            
            if(aimMode != AimMode::Buff) {
                if (!config.AimDebugSettings.StopScan && now - lastFoundEnemyTime > std::chrono::milliseconds(2000)) {
                    static auto last_searching_log = std::chrono::steady_clock::time_point{};
                    const int patrol_mode = config.PatrolScanSettings.Mode;
                    const bool boost_patrol_scan =
                        aimMode == AimMode::RotateScan &&
                        damage_rotate_elapsed_ms >= 0 &&
                        damage_rotate_elapsed_ms <= kDamageScanBoostWindowMs;
                    float yaw_scan_step = boost_patrol_scan
                        ? kPatrolScanYawBoostStep
                        : kPatrolScanYawStep;
                    int yaw_scan_direction = 1;

                    if (patrol_mode == 2) {
                        yaw_scan_step = boost_patrol_scan ? kPatrolSwingYawBoostStep : kPatrolSwingYawStep;

                        if (!patrolScanCenterInitialized_ || patrolScanActiveMode_ != patrol_mode) {
                            patrolScanCenterInitialized_ = true;
                            patrolScanActiveMode_ = patrol_mode;
                            patrolScanCenterYaw_ = gimbalAngles.Yaw;
                            patrolScanOffsetYaw_ = 0.0f;
                            patrolScanPhaseRad_ = 0.0f;
                            patrolScanDirection_ = 1; // 新一轮巡逻默认先向右
                        }

                        const float half_range = kPatrolSwingHalfRangeDeg;
                        const float phase_step = yaw_scan_step / std::max(half_range, 1.0f);
                        const float center_drift_step =
                            kPatrolSwingCenterDriftPerCycleDeg * phase_step / kTwoPi;

                        // mode2: 讓中心點每完成一個正弦掃描週期固定右偏同樣角度。
                        patrolScanCenterYaw_ = normalize_angle_near(
                            patrolScanCenterYaw_ + center_drift_step,
                            gimbalAngles.Yaw);

                        patrolScanPhaseRad_ = std::fmod(patrolScanPhaseRad_ + phase_step, kTwoPi);
                        patrolScanOffsetYaw_ = half_range * std::sin(patrolScanPhaseRad_);
                        patrolScanDirection_ = (std::cos(patrolScanPhaseRad_) >= 0.0f) ? 1 : -1;
                        if (patrolScanPhaseRad_ < 0.0f) {
                            patrolScanPhaseRad_ += kTwoPi;
                        }
                        yaw_scan_direction = patrolScanDirection_;
                    } else {
                        if (patrolScanActiveMode_ != patrol_mode || patrolScanCenterInitialized_) {
                            reset_patrol_scan_state();
                            patrolScanActiveMode_ = patrol_mode;
                        }
                        yaw_scan_direction = boost_patrol_scan
                            ? (((damage_rotate_elapsed_ms / kDamageScanYawPhaseMs) % 2 == 0) ? 1 : -1)
                            : 1;
                    }
                    if (now - last_searching_log > std::chrono::seconds(2)) {
                        LoggerPtr->Debug(
                            "Searching Target... patrol_mode={} yaw_step={} dir={} (damage_boost={} elapsed_ms={})",
                            patrol_mode,
                            yaw_scan_step,
                            yaw_scan_direction,
                            boost_patrol_scan ? 1 : 0,
                            damage_rotate_elapsed_ms);
                        last_searching_log = now;
                        gimbalControlData.FireCode.AimMode = 0;
                    }
                    const auto current_time = std::chrono::steady_clock::now();
                    const float next_scan_yaw = (patrol_mode == 2 && patrolScanCenterInitialized_)
                        ? normalize_angle_near(patrolScanCenterYaw_ + patrolScanOffsetYaw_, gimbalAngles.Yaw)
                        : static_cast<float>(gimbalAngles.Yaw + yaw_scan_direction * yaw_scan_step);
                    const auto pitch_elapsed_ms = static_cast<float>(
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            current_time - gameStartTime).count());
                    const float next_scan_pitch = kPatrolScanPitchCenterDeg +
                        kPatrolScanPitchHalfRangeDeg *
                            std::sin(pitch_elapsed_ms * kTwoPi / std::max(kPatrolScanPitchPeriodMs, 1.0f));
                    nextAngles = GimbalAnglesType{
                        static_cast<AngleType>(next_scan_yaw),
                        static_cast<AngleType>(next_scan_pitch)
                    };

                    if (aimMode == AimMode::Outpost) {
                        nextAngles.Pitch += 15.0f;
                    }
                } else if (config.AimDebugSettings.ReuseLatchedAnglesOnNoTarget &&
                           activeAimData->HasLatchedAngles) {
                    reset_patrol_scan_state();
                    nextAngles = activeAimData->Angles;
                    LoggerPtr->Debug(
                        "Reuse latched aim angles -> Pitch: {}, Yaw: {}",
                        nextAngles.Pitch,
                        nextAngles.Yaw);
                } else {
                    reset_patrol_scan_state();
                    nextAngles = gimbalAngles;
                    LoggerPtr->Debug(
                        "Hold current gimbal angles (latched reuse disabled or unavailable) -> Pitch: {}, Yaw: {}",
                        gimbalAngles.Pitch,
                        gimbalAngles.Yaw);
                }
            }else { // 打符模式
                reset_patrol_scan_state();
                if (now_time < 5) {
                    LoggerPtr->Info("Set Angles, Buff Mode, 10 min!");
                    // Yaw
                    float current_yaw = normalize_angle_0_360(gimbalAngles.Yaw);
                    float delta = buff_yaw - current_yaw;
                    if (delta > 180.0f) delta -= 360.0f; // 角度差大于180，反向旋转
                    if (delta < -180.0f) delta += 360.0f;
                    int opt = delta > 0 ? 1 : -1;
                    int target_yaw = gimbalAngles.Yaw + delta;
                    nextAngles = gimbalAngles;

                    if (std::abs(delta) > 10 * delta_yaw) {
                        nextAngles.Yaw = static_cast<AngleType>(gimbalAngles.Yaw + delta_yaw * opt);
                    } else {
                        nextAngles.Yaw = static_cast<AngleType>(target_yaw);
                    }
                    nextAngles.Pitch = 19.0f;
                }else {
                    nextAngles = (buffAimData.Fresh && buffAimData.Valid && buffAimData.BuffFollow)
                        ? buffAimData.Angles
                        : gimbalAngles;
                }
            }
            gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
        }

        if (chase_mode_enabled) {
            const bool use_relative_target_topic = config.ChaseSettings.UseRelativeTargetTopic;
            const bool use_tf_goal_bridge =
                config.NaviSettings.UseXY && config.NaviSettings.UseTfGoalBridge;
            bool has_chase_target = find_target;
            if (!has_chase_target &&
                config.ChaseSettings.LostTargetHoldMs > 0 &&
                activeAimData->HasLatchedAngles &&
                lastTargetSeenTime.time_since_epoch().count() != 0) {
                has_chase_target = (now - lastTargetSeenTime) <=
                    std::chrono::milliseconds(config.ChaseSettings.LostTargetHoldMs);
            }

            bool chase_distance_valid = false;
            float distance_cm = 0.0f;
            if (std::isfinite(targetArmor.Distance) && targetArmor.Distance > 0.0f) {
                distance_cm = targetArmor.Distance * 100.0f;
                chase_distance_valid =
                    distance_cm >= static_cast<float>(config.ChaseSettings.MinValidDistanceCm) &&
                    distance_cm <= static_cast<float>(config.ChaseSettings.MaxValidDistanceCm);
            }

            if (has_chase_target &&
                chase_distance_valid &&
                targetArmor.Type != ArmorType::UnKnown) {
                const auto yaw_error_deg = static_cast<double>(
                    std::remainder(nextAngles.Yaw - gimbalAngles.Yaw, 360.0f));
                const auto pitch_error_deg = static_cast<double>(
                    std::remainder(nextAngles.Pitch - gimbalAngles.Pitch, 360.0f));
                const double distance_m = static_cast<double>(distance_cm) * 0.01;
                constexpr double kDegToRad = 0.017453292519943295;
                const double yaw_rad = yaw_error_deg * kDegToRad;
                const double pitch_rad = pitch_error_deg * kDegToRad;
                const double cos_pitch = std::cos(pitch_rad);

                naviRelativeTargetValid = true;
                naviRelativeTargetX = static_cast<float>(distance_m * cos_pitch * std::cos(yaw_rad));
                naviRelativeTargetY = static_cast<float>(distance_m * cos_pitch * std::sin(yaw_rad));
                naviRelativeTargetZ = static_cast<float>(distance_m * std::sin(pitch_rad));
                naviRelativeTargetDistance = static_cast<float>(distance_m);
                naviRelativeTargetYawErrorDeg = static_cast<float>(yaw_error_deg);
                naviRelativeTargetPitchErrorDeg = static_cast<float>(pitch_error_deg);
                naviRelativeTargetArmorType = static_cast<std::uint8_t>(targetArmor.Type);

                if (use_relative_target_topic) {
                    // 直发地图坐标模式：UseXY=true 且关闭 tf bridge。
                    if (config.NaviSettings.UseXY && !use_tf_goal_bridge) {
                        const auto maybe_target_unit = UnitTypeFromArmorType(targetArmor.Type);
                        if (maybe_target_unit.has_value()) {
                            const int enemy_x = static_cast<int>(enemyRobots[*maybe_target_unit].position_.X);
                            const int enemy_y = static_cast<int>(enemyRobots[*maybe_target_unit].position_.Y);
                            if (enemy_x >= 0 && enemy_y >= 0) {
                                naviGoalPosition.x = static_cast<std::uint16_t>(std::clamp(enemy_x, 0, 65535));
                                naviGoalPosition.y = static_cast<std::uint16_t>(std::clamp(enemy_y, 0, 65535));
                            }
                        }
                    }
                } else {
                    const double distance_error_cm =
                        static_cast<double>(distance_cm) -
                        static_cast<double>(config.ChaseSettings.PreferredDistanceCm);

                    int chase_vx = 0;
                    if (std::abs(distance_error_cm) >
                        static_cast<double>(config.ChaseSettings.DistanceDeadbandCm)) {
                        chase_vx = static_cast<int>(std::lround(config.ChaseSettings.DistanceKp * distance_error_cm));
                        chase_vx = std::clamp(
                            chase_vx,
                            -config.ChaseSettings.MaxBackwardSpeed,
                            config.ChaseSettings.MaxForwardSpeed);
                    }

                    int chase_vy = 0;
                    if (config.ChaseSettings.UseYawStrafe) {
                        if (std::abs(yaw_error_deg) > static_cast<double>(config.ChaseSettings.YawDeadbandDeg)) {
                            chase_vy = static_cast<int>(std::lround(config.ChaseSettings.YawKp * yaw_error_deg));
                            if (config.ChaseSettings.InvertStrafeDirection) {
                                chase_vy = -chase_vy;
                            }
                            chase_vy = std::clamp(
                                chase_vy,
                                -config.ChaseSettings.MaxStrafeSpeed,
                                config.ChaseSettings.MaxStrafeSpeed);
                        }
                    }

                    nextVelocity.X = static_cast<std::int8_t>(ClampToInt8(chase_vx));
                    nextVelocity.Y = static_cast<std::int8_t>(ClampToInt8(chase_vy));
                }
            } else if (!use_relative_target_topic &&
                       config.ChaseSettings.StopWhenNoTarget) {
                nextVelocity = VelocityType{0, 0};
            } else if (use_relative_target_topic &&
                       config.NaviSettings.UseXY &&
                       !use_tf_goal_bridge &&
                       config.ChaseSettings.StopWhenNoTarget) {
                const int self_x = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
                const int self_y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
                if (self_x >= 0 && self_y >= 0) {
                    naviGoalPosition.x = static_cast<std::uint16_t>(std::clamp(self_x, 0, 65535));
                    naviGoalPosition.y = static_cast<std::uint16_t>(std::clamp(self_y, 0, 65535));
                }
            }
        }

        // lower_head 只在未锁目标时生效，并且整对角一起切换，避免混用旧 yaw/new pitch。
        if(naviLowerHead && !find_target) {
            nextAngles = GimbalAnglesType{gimbalAngles.Yaw, -15.0f}; //-22.5 - 26.0
        }
        gimbalControlData.GimbalAngles = nextAngles;
        naviVelocity = nextVelocity;

        PublishMessageAll();
        autoAimData.Fresh = false;
        buffAimData.Fresh = false;
        outpostAimData.Fresh = false;
        isFindTargetAtomic = false;
    }

    /**
     * @brief 决策进程主循环 \n
     * @brief 1. 设置黑板数据 \n
     * @brief 2. 休眠 \n
     * @brief 3. 更新黑板数据 \n
     * @brief 4. 处理行为树 \n
     */
    void Application::GameLoop() {

        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }
        ResetTickBlackboard();

        GlobalBlackboard_->set<UnitTeam>("MyTeam", team); // 队伍颜色
        GlobalBlackboard_->set<std::chrono::steady_clock::time_point>(
            "LastCommandTime", std::chrono::steady_clock::now()); // 上次发送命令的时间
        GlobalBlackboard_->set<std::chrono::seconds>("CommandInterval", std::chrono::seconds{0}); // 命令间隔
        GlobalBlackboard_->set<ArmorType>("AimTarget", ArmorType::Hero); // 辅瞄击打目标
        GlobalBlackboard_->set<std::uint8_t>("naviCommandGoal", Home(team)); // 导航目的地
        GlobalBlackboard_->set<std::shared_ptr<Logger>>("LoggerPtr", LoggerPtr);
        GlobalBlackboard_->set("TickBlackboard", TickBlackboard_);
        UpdateBlackBoard();

        while (rclcpp::ok()) {
            MarkLoopBeat();
            rclcpp::spin_some(node_); // 处理回调函数

            if (TryHandleSoftRecovery()) {
                treeTickRateClock.sleep();
                continue;
            }

            if (IsCriticalInputStale()) {
                static auto last_stale_log = std::chrono::steady_clock::time_point{};
                const auto now = std::chrono::steady_clock::now();
                if (LoggerPtr && (now - last_stale_log > std::chrono::seconds(2))) {
                    LoggerPtr->Warning("Critical input stale: gimbal angles > {} ms, publish safe-control.",
                                       kRuntimeGimbalStaleMs);
                    last_stale_log = now;
                }
                PublishSafeControl("gimbal_stale");
                treeTickRateClock.sleep();
                continue;
            }

            if (runtimeRearmStartGate_ && !is_game_begin) {
                const auto now = std::chrono::steady_clock::now();
                if (!runtimeStartGateActive_) {
                    runtimeStartGateActive_ = true;
                    runtimeStartGateLastLogTime_ = now;
                    LoggerPtr->Info("Runtime start gate armed: waiting /ly/game/is_start=true.");
                } else if (LoggerPtr && (now - runtimeStartGateLastLogTime_ > std::chrono::seconds(2))) {
                    LoggerPtr->Debug("Runtime start gate waiting for /ly/game/is_start=true...");
                    runtimeStartGateLastLogTime_ = now;
                }

                SET_POSITION(Home, team);
                if (publishNaviGoal_ && naviCommandRateClock.trigger()) {
                    naviCommandRateClock.tick();
                    const bool use_tf_goal_bridge =
                        config.NaviSettings.UseXY &&
                        config.NaviSettings.UseTfGoalBridge &&
                        config.ChaseSettings.Enable &&
                        config.ChaseSettings.UseRelativeTargetTopic;
                    if (config.NaviSettings.UseXY && !use_tf_goal_bridge) {
                        PubNaviGoalPos();
                    } else {
                        PubNaviGoal();
                    }
                }

                naviVelocityInput = VelocityType{0, 0};
                naviVelocity = VelocityType{0, 0};
                postureCommand = 0;
                PublishSafeControl("runtime_start_gate");
                UpdateBlackBoard();
                if (decisionTraceEnabled_) {
                    WriteDecisionTrace("tick");
                }
                treeTickRateClock.sleep();
                continue;
            }
            if (runtimeRearmStartGate_ && runtimeStartGateActive_ && is_game_begin) {
                runtimeStartGateActive_ = false;
                gameStartTime = std::chrono::steady_clock::now();
                if (LoggerPtr) {
                    LoggerPtr->Info("Runtime start gate opened: resume decision loop.");
                }
                if (decisionTraceEnabled_) {
                    WriteDecisionTrace("game_start");
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastTreeTickLogTime_ > std::chrono::seconds(2)) {
                LoggerPtr->Debug("BehaviorTree Root Tick...");
                lastTreeTickLogTime_ = now;
            }
            TreeTickGuarded();
            if (decisionTraceEnabled_) {
                WriteDecisionTrace("tick");
            }
            treeTickRateClock.sleep();
        }
    }

    void Application::TreeTickGuarded() {
        MarkTickStart();
        try {
            TreeTick();
        } catch (const std::exception& ex) {
            if (LoggerPtr) {
                LoggerPtr->Error("BehaviorTree tick exception: {}", ex.what());
            }
            RequestSoftRecovery(RuntimeFaultCode::TreeException);
        } catch (...) {
            if (LoggerPtr) {
                LoggerPtr->Error("BehaviorTree tick exception: <unknown>");
            }
            RequestSoftRecovery(RuntimeFaultCode::TreeException);
        }
        MarkTickEnd();
    }

    void Application::TreeTick() {
        if (BTree.subtrees.empty()) {
            LoggerPtr->Error("BehaviorTree is empty, skip tick.");
            RequestSoftRecovery(RuntimeFaultCode::TreeEmpty);
            return;
        }

        const auto status = BTree.tickWhileRunning(std::chrono::milliseconds(1));
        if (status == BT::NodeStatus::FAILURE) {
            LoggerPtr->Warning("BehaviorTree tick returned FAILURE.");
        }
    }

    bool Application::IsDecisionAutonomyModuleEnabled(std::string_view module) const {
        const auto& autonomy = config.DecisionAutonomySettings;
        if (!autonomy.Enable) {
            return false;
        }
        const auto normalized_module = NormalizeDecisionModule(module);
        if (HasAutonomyToken(autonomy.HardRuleModules, normalized_module)) {
            return false;
        }
        if (HasAutonomyToken(autonomy.EnabledModules, "all")) {
            return true;
        }
        return HasAutonomyToken(autonomy.EnabledModules, normalized_module);
    }

    void Application::SelectStrategyMode() {
        if (GetCompetitionProfile() == CompetitionProfile::League) {
            SetStrategyMode(StrategyMode::LeagueSimple);
            return;
        }
        if (IsNaviDebugEnabled()) {
            SetStrategyMode(StrategyMode::NaviTest);
            return;
        }

        const int now_time = ElapsedSeconds();
        std::uint16_t self_outpost_health = selfOutpostHealth;
        std::uint16_t enemy_outpost_health = enemyOutpostHealth;
        std::uint16_t self_health = myselfHealth;
        std::uint16_t time_left = timeLeft;
        BuffType team_buff = teamBuff;
        if (GlobalBlackboard_) {
            (void)GlobalBlackboard_->get("SelfOutpostHealth", self_outpost_health);
            (void)GlobalBlackboard_->get("EnemyOutpostHealth", enemy_outpost_health);
            (void)GlobalBlackboard_->get("SelfHealth", self_health);
            (void)GlobalBlackboard_->get("TimeLeft", time_left);
            (void)GlobalBlackboard_->get("TeamBuff", team_buff);
        }

        const StrategyMode current_strategy = GetStrategyMode();
        StrategyMode next_strategy = current_strategy;
        const bool low_resource = (self_health < 100 || time_left <= 120 ||
                                   team_buff.RemainingEnergy == 0b10000 ||
                                   team_buff.RemainingEnergy == 0b00000);
        const bool sentry_window =
            (enemy_outpost_health > 0 && self_outpost_health > 100 && now_time < 55);

        // 开局保持初始策略，避免频繁抖动。
        if (now_time < 10) {
            SetStrategyMode(next_strategy);
            return;
        }
        // NaviTest 保持与旧逻辑一致，直到脚本窗口结束。
        if (current_strategy == StrategyMode::NaviTest && now_time < 340) {
            SetStrategyMode(next_strategy);
            return;
        }

        if (IsDecisionAutonomyModuleEnabled("strategy_mode")) {
            const auto& strategy_autonomy = config.DecisionAutonomySettings.Strategy;
            std::vector<StrategyMode> candidates;
            candidates.reserve(strategy_autonomy.Candidates.size());
            for (const auto& candidate : strategy_autonomy.Candidates) {
                const auto mode = StrategyModeFromAutonomyToken(candidate);
                if (!mode.has_value()) {
                    continue;
                }
                if (std::find(candidates.begin(), candidates.end(), *mode) == candidates.end()) {
                    candidates.push_back(*mode);
                }
            }
            if (candidates.empty()) {
                candidates = {StrategyMode::HitHero, StrategyMode::HitSentry, StrategyMode::Protected};
            }

            double best_score = -std::numeric_limits<double>::infinity();
            std::optional<StrategyMode> best_mode;
            for (const auto candidate : candidates) {
                double score = 0.0;
                switch (candidate) {
                    case StrategyMode::HitHero:
                        score += strategy_autonomy.HitHeroBias;
                        break;
                    case StrategyMode::HitSentry:
                        score += strategy_autonomy.HitSentryBias;
                        if (sentry_window) {
                            score += strategy_autonomy.SentryWindowBonus;
                        }
                        if (enemy_outpost_health <= 0 || !sentry_window) {
                            score -= strategy_autonomy.NoOutpostSentryPenalty;
                        }
                        break;
                    case StrategyMode::Protected:
                        score += strategy_autonomy.ProtectedBias;
                        if (low_resource) {
                            score += strategy_autonomy.LowResourceProtectedBonus;
                        }
                        if (time_left <= 120) {
                            score += strategy_autonomy.TimePressureProtectedBonus;
                        }
                        break;
                    default:
                        break;
                }
                if (low_resource && candidate != StrategyMode::Protected) {
                    score -= strategy_autonomy.LowResourceOffensePenalty;
                }
                if (candidate == current_strategy) {
                    score += strategy_autonomy.CurrentStrategyBonus;
                }
                if (score > best_score) {
                    best_score = score;
                    best_mode = candidate;
                }
            }
            if (best_mode.has_value()) {
                next_strategy = *best_mode;
            }
        } else {
            if (low_resource) {
                next_strategy = StrategyMode::Protected;
            } else if (sentry_window) {
                next_strategy = StrategyMode::HitSentry;
            } else {
                next_strategy = StrategyMode::HitHero;
            }
        }

        SetStrategyMode(next_strategy);
        if (next_strategy != current_strategy) {
            LoggerPtr->Info(
                "StrategyMode switch: {} -> {} (autonomy={})",
                StrategyModeToString(current_strategy),
                StrategyModeToString(next_strategy),
                IsDecisionAutonomyModuleEnabled("strategy_mode") ? 1 : 0);
        }
    }

    void Application::SetAimMode() {
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();
        LoggerPtr->Info("SetAimMode - now_time: {}", now_time);
        if(config.GameStrategySettings.HitBuff) { // 打符
            if(now_time < 25 && buff_shoot_count <= 15){
                if(now_time > 7) {
                    if(teamBuff.DefenceBuff > 20 || teamBuff.VulnerabilityBuff > 20){
                        LoggerPtr->Info("Buff has been activated!");
                        LoggerPtr->Info("Defense Buff: {}, Vulnerability Buff: {}", teamBuff.DefenceBuff, teamBuff.VulnerabilityBuff);
                        aimMode = AimMode::RotateScan;
                    }
                    else{
                        LoggerPtr->Info("Buff has not been activated!");
                        aimMode = AimMode::Buff;
                    }
                }else aimMode = AimMode::Buff;
                
            }else {
                LoggerPtr->Info("Time out 25 seconds, stop hit buff!");
                aimMode = AimMode::RotateScan;
            }
        }else if(config.GameStrategySettings.HitOutpost) { // 打前哨站
            if(enemyOutpostHealth > 0) {
                LoggerPtr->Info("Enemy Outpost Health: {}", enemyOutpostHealth);;
                if(now_time < 90) {
                    aimMode = AimMode::Outpost;
                }else {
                    LoggerPtr->Info("Time out 1.5 min, stop hit outpost!");
                    aimMode = AimMode::RotateScan;
                }
            }else {
                LoggerPtr->Info("Enemy Outpost has been destroyed!");
                aimMode = AimMode::RotateScan;
            }
        }else { // 普通模式
            LoggerPtr->Info("AimMode: RotateScan!");
            aimMode = AimMode::RotateScan;
        }

    }
    // 提前处理坐标等数据
    void Application::ProcessData() {
        int now_time = 420 - timeLeft;
        // 处理坐标数据
        reliableEnemyPosuition.clear();
        for(auto robot : RobotLists) {
           if(enemyRobots[robot].position_.X > 100 && enemyRobots[robot].position_.Y > 100) {
                reliableEnemyPosuition.push_back(robot);
            }
        }
        LoggerPtr->Info("> reliableEnemyPosuition <");
        for(auto robot : reliableEnemyPosuition) {
            LoggerPtr->Info("ID: {}, X: {}, Y:{}", static_cast<int>(robot), enemyRobots[robot].position_.X, enemyRobots[robot].position_.Y);
        }

        // 处理距离和无敌状态的数据
        hitableTargets.clear();
        for (auto Armor : armorList) {
            if (Armor.Type == ArmorType::UnKnown) continue;
            if(Armor.Type == ArmorType::Hero) {
                enemyRobots[UnitType::Hero].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Hero].isInvulnerable()) hitableTargets.push_back(UnitType::Hero);
            }else if(Armor.Type == ArmorType::Engineer) {
                enemyRobots[UnitType::Engineer].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Engineer].isInvulnerable() && now_time > 60) hitableTargets.push_back(UnitType::Engineer);
            }else if(Armor.Type == ArmorType::Infantry1) {
                enemyRobots[UnitType::Infantry1].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Infantry1].isInvulnerable()) hitableTargets.push_back(UnitType::Infantry1);
            }else if(Armor.Type == ArmorType::Infantry2) {
                enemyRobots[UnitType::Infantry2].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Infantry2].isInvulnerable()) hitableTargets.push_back(UnitType::Infantry2);
            }else if(Armor.Type == ArmorType::Sentry) {
                enemyRobots[UnitType::Sentry].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Sentry].isInvulnerable()) hitableTargets.push_back(UnitType::Sentry);
            }
        }
        for(auto robot : hitableTargets) {
            LoggerPtr->Info("ID{}", static_cast<int>(robot));
        }
    }

    void Application::SetAimTarget() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        std::uint16_t nowx = friendRobots[UnitType::Sentry].position_.X, nowy = friendRobots[UnitType::Sentry].position_.Y;
        const auto is_ignored_armor = [this](const ArmorType armor_type) -> bool {
            return IsIgnoredArmorType(config.AimTargetIgnore, armor_type);
        };
        if(aimMode == AimMode::Buff) { // 打符，修改为默认值
            if(BehaviorTree::Area::BuffShoot.near(nowx, nowy, 100, MyTeam) &&
               !is_ignored_armor(ArmorType::Hero)) {
                targetArmor.Type = ArmorType::Hero;
            } else {
                SetAimTargetNormal();
            }
        }else if(aimMode == AimMode::Outpost) { // 打前哨站
            if(BehaviorTree::Area::OutpostShoot.near(nowx, nowy, 100, MyTeam) &&
               !is_ignored_armor(ArmorType::Outpost)) {
                targetArmor.Type = ArmorType::Outpost;
            } else {
                SetAimTargetNormal();
            }
        }else { // 普通模式
            if(naviCommandGoal == LangYa::HoleRoad(EnemyTeam)) { // 英雄点位1
                if(BehaviorTree::Area::HoleRoad.near(nowx, nowy, 100, MyTeam) &&
                   !is_ignored_armor(ArmorType::Hero)) {
                    targetArmor.Type = ArmorType::Hero;
                    targetArmor.Distance = enemyRobots[UnitType::Hero].distance_;
                } else {
                    SetAimTargetNormal();
                }
            } else {
                SetAimTargetNormal();
            }
            LoggerPtr->Info("Target: {}", static_cast<int>(targetArmor.Type));
        }

    }

    bool Application::TrySetAimTargetByAutonomy() {
        if (!IsDecisionAutonomyModuleEnabled("aim_target")) {
            return false;
        }

        struct AimCandidate {
            ArmorType Armor{ArmorType::UnKnown};
            UnitType Unit{UnitType::Unknown};
            float Distance{0.0f};
            std::uint16_t Health{0U};
        };

        std::vector<AimCandidate> candidates;
        candidates.reserve(hitableTargets.size());
        std::uint16_t max_health = 1U;
        for (const auto unit_type : hitableTargets) {
            if (IsIgnoredUnitType(config.AimTargetIgnore, unit_type)) {
                continue;
            }
            const auto armor_type = ArmorTypeFromUnitType(unit_type);
            if (!armor_type.has_value()) {
                continue;
            }
            const auto& robot = enemyRobots[unit_type];
            candidates.push_back(AimCandidate{
                .Armor = *armor_type,
                .Unit = unit_type,
                .Distance = robot.distance_,
                .Health = robot.currentHealth_
            });
            max_health = std::max(max_health, robot.currentHealth_);
        }
        if (candidates.empty()) {
            return false;
        }

        const auto& autonomy = config.DecisionAutonomySettings.AimTarget;
        const auto get_priority_rank = [this](const ArmorType armor_type) {
            const int armor_id = static_cast<int>(armor_type);
            const auto it = std::find(config.AimTargetPriority.begin(), config.AimTargetPriority.end(), armor_id);
            if (it == config.AimTargetPriority.end()) {
                return static_cast<int>(config.AimTargetPriority.size());
            }
            return static_cast<int>(std::distance(config.AimTargetPriority.begin(), it));
        };

        double best_score = -std::numeric_limits<double>::infinity();
        std::optional<AimCandidate> best_candidate;
        for (const auto& candidate : candidates) {
            const int rank = get_priority_rank(candidate.Armor);
            const double priority_score = config.AimTargetPriority.empty()
                ? 0.0
                : static_cast<double>(config.AimTargetPriority.size() - rank) /
                    static_cast<double>(config.AimTargetPriority.size());
            const double distance_score = (std::isfinite(candidate.Distance) && candidate.Distance > 0.0f)
                ? 1.0 / (0.1 + static_cast<double>(candidate.Distance))
                : 0.0;
            const double health_score = 1.0 -
                static_cast<double>(candidate.Health) / static_cast<double>(std::max<std::uint16_t>(1U, max_health));

            double score = 0.0;
            score += autonomy.PriorityWeight * priority_score;
            score += autonomy.DistanceWeight * distance_score;
            score += autonomy.LowHealthWeight * health_score;
            if (candidate.Armor == targetArmor.Type) {
                score += autonomy.CurrentTargetBonus;
            }
            if (candidate.Armor == ArmorType::Hero) {
                score += autonomy.HeroBonus;
            } else if (candidate.Armor == ArmorType::Sentry) {
                score += autonomy.SentryBonus;
            }

            if (score > best_score) {
                best_score = score;
                best_candidate = candidate;
            }
        }

        if (!best_candidate.has_value()) {
            return false;
        }
        targetArmor.Type = best_candidate->Armor;
        targetArmor.Distance = best_candidate->Distance;
        return true;
    }

    void Application::SetAimTargetNormal() {
        auto set_target = [&](const ArmorType armor_type, const UnitType unit_type) {
            targetArmor.Type = armor_type;
            targetArmor.Distance = enemyRobots[unit_type].distance_;
        };
        const auto is_armor_ignored = [this](const ArmorType armor_type) -> bool {
            return IsIgnoredArmorType(config.AimTargetIgnore, armor_type);
        };
        const auto has_target = [&](const UnitType unit_type) -> bool {
            return !IsIgnoredUnitType(config.AimTargetIgnore, unit_type) &&
                   std::find(hitableTargets.begin(), hitableTargets.end(), unit_type) != hitableTargets.end();
        };
        const bool infantry1_find = has_target(UnitType::Infantry1);
        const bool infantry2_find = has_target(UnitType::Infantry2);

        if (TrySetAimTargetByAutonomy()) {
            return;
        }

        if (!hitableTargets.empty()) {
            for (const auto armor_id : config.AimTargetPriority) {
                const auto armor_type = ArmorTypeFromPriorityId(armor_id);
                if (!armor_type.has_value()) {
                    continue;
                }
                if (is_armor_ignored(*armor_type)) {
                    continue;
                }
                if ((*armor_type == ArmorType::Infantry1 || *armor_type == ArmorType::Infantry2) &&
                    infantry1_find && infantry2_find) {
                    // 兼容旧逻辑：步兵1/2同时可打时，优先近距离；距离接近时看血量更低者。
                    const auto distance1 = enemyRobots[UnitType::Infantry1].distance_;
                    const auto distance2 = enemyRobots[UnitType::Infantry2].distance_;
                    const auto delta_distance = distance1 - distance2;
                    if (std::fabs(delta_distance) > 1.0f) {
                        if (distance1 < distance2) {
                            set_target(ArmorType::Infantry1, UnitType::Infantry1);
                        } else {
                            set_target(ArmorType::Infantry2, UnitType::Infantry2);
                        }
                    } else {
                        const auto health1 = enemyRobots[UnitType::Infantry1].currentHealth_;
                        const auto health2 = enemyRobots[UnitType::Infantry2].currentHealth_;
                        if (health1 < health2) {
                            set_target(ArmorType::Infantry1, UnitType::Infantry1);
                        } else {
                            set_target(ArmorType::Infantry2, UnitType::Infantry2);
                        }
                    }
                    return;
                }
                const auto unit_type = UnitTypeFromArmorType(*armor_type);
                if (!unit_type.has_value()) {
                    continue;
                }
                if (!has_target(*unit_type)) {
                    continue;
                }
                set_target(*armor_type, *unit_type);
                return;
            }

            // 配置优先级没有命中时，回退到最近目标
            std::optional<UnitType> nearest_unit;
            for (const auto unit_type : hitableTargets) {
                if (IsIgnoredUnitType(config.AimTargetIgnore, unit_type)) {
                    continue;
                }
                if (!nearest_unit.has_value() ||
                    enemyRobots[unit_type].distance_ < enemyRobots[*nearest_unit].distance_) {
                    nearest_unit = unit_type;
                }
            }
            if (nearest_unit.has_value()) {
                const auto armor_type = ArmorTypeFromUnitType(*nearest_unit);
                if (armor_type.has_value()) {
                    set_target(*armor_type, *nearest_unit);
                    return;
                }
            }
        }

        for (const auto armor_id : config.AimTargetPriority) {
            const auto armor_type = ArmorTypeFromPriorityId(armor_id);
            if (!armor_type.has_value() || is_armor_ignored(*armor_type)) {
                continue;
            }
            targetArmor.Type = *armor_type;
            targetArmor.Distance = 30;
            return;
        }
        targetArmor.Type = ArmorType::UnKnown;
        targetArmor.Distance = 30;
    }

    void Application::CheckDebug() {
        if (config.AimDebugSettings.HitBuff) aimMode = AimMode::Buff;
        else if(config.AimDebugSettings.HitOutpost) aimMode = AimMode::Outpost; 
        /*------------打印日志---------*/
        if(aimMode == AimMode::AutoAim) LoggerPtr->Info("AimMode: AutoAim");
        else if(aimMode == AimMode::Buff) LoggerPtr->Info("AimMode: Buff");
        else if(aimMode == AimMode::Outpost) LoggerPtr->Info("AimMode: Outpost");
        else if(aimMode == AimMode::RotateScan) LoggerPtr->Info("AimMode: RotateScan");
    }
}


namespace BehaviorTree {

    bool Application::IsLeagueRouteCompatEnabled() const noexcept {
        return IsLeagueProfile() &&
            !config.NaviSettings.UseXY &&
            config.LeagueStrategySettings.EnableRouteCompat;
    }

    bool Application::IsLeagueGoalSwitchBetween2And3(
        const std::uint8_t from_goal_id,
        const std::uint8_t to_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) const noexcept {
        const auto goal_2 = ResolveGoalId(LangYa::Recovery.ID, goal_team, apply_team_offset);
        const auto goal_3 = ResolveGoalId(LangYa::BuffShoot.ID, goal_team, apply_team_offset);
        return (from_goal_id == goal_2 && to_goal_id == goal_3) ||
            (from_goal_id == goal_3 && to_goal_id == goal_2);
    }

    bool Application::TickLeagueRouteCompat(
        const UnitTeam goal_team,
        const bool apply_team_offset) {
        if (!IsLeagueRouteCompatEnabled() || !leagueRouteCompatActive_) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now < leagueRouteCompatUntil_) {
            SetPositionByBaseGoal(kLeagueRouteCompatViaGoalBaseId, goal_team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            return true;
        }

        leagueRouteCompatActive_ = false;
        leagueRouteCompatUntil_ = std::chrono::steady_clock::time_point{};
        if (!leagueRouteCompatHasPendingGoal_) {
            return false;
        }

        const auto pending_base_goal = leagueRouteCompatPendingBaseGoal_;
        const int pending_hold_sec = std::max(1, leagueRouteCompatPendingHoldSec_);
        leagueRouteCompatHasPendingGoal_ = false;

        SetPositionByBaseGoal(pending_base_goal, goal_team, apply_team_offset);
        naviCommandIntervalClock.reset(Seconds{pending_hold_sec});
        speedLevel = 1;
        LoggerPtr->Info(
            "League route compat finished: via goal done, continue goal={} hold={}s.",
            static_cast<int>(naviCommandGoal),
            pending_hold_sec);
        return true;
    }

    void Application::StartLeagueRouteCompat(
        const std::uint8_t pending_base_goal,
        const int pending_hold_sec,
        const UnitTeam goal_team,
        const bool apply_team_offset,
        const char* reason) {
        if (!IsLeagueRouteCompatEnabled()) {
            return;
        }
        const int safe_pending_hold_sec = std::max(1, pending_hold_sec);
        leagueRouteCompatActive_ = true;
        leagueRouteCompatUntil_ = std::chrono::steady_clock::now() + std::chrono::seconds(kLeagueRouteCompatViaHoldSec);
        leagueRouteCompatHasPendingGoal_ = true;
        leagueRouteCompatPendingBaseGoal_ = pending_base_goal;
        leagueRouteCompatPendingHoldSec_ = safe_pending_hold_sec;

        SetPositionByBaseGoal(kLeagueRouteCompatViaGoalBaseId, goal_team, apply_team_offset);
        naviCommandIntervalClock.reset(Seconds{1});
        speedLevel = 1;
        LoggerPtr->Info(
            "League route compat {}: via goal={} for {}s, then goal={} hold={}s.",
            reason ? reason : "start",
            static_cast<int>(naviCommandGoal),
            kLeagueRouteCompatViaHoldSec,
            static_cast<int>(ResolveGoalId(pending_base_goal, goal_team, apply_team_offset)),
            safe_pending_hold_sec);
    }

    std::uint8_t Application::ResolveGoalId(
        const std::uint8_t base_goal_id,
        const UnitTeam team,
        const bool apply_team_offset) const noexcept {
        if (!apply_team_offset) {
            return base_goal_id;
        }
        return team == UnitTeam::Blue
            ? static_cast<std::uint8_t>(base_goal_id + LangYa::TeamedLocation::LocationCount)
            : base_goal_id;
    }

    bool Application::IsNaviGoalAreaScopeEnabled() const noexcept {
        return config.DecisionAutonomySettings.Enable &&
               config.DecisionAutonomySettings.NaviGoal.UseAreaScope;
    }

    bool Application::IsNaviGoalAllowedByAreaScope(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const UnitTeam enemy_team) const {
        if (!IsNaviGoalAreaScopeEnabled()) {
            return true;
        }
        if (!IsValidBaseGoalId(base_goal_id)) {
            return false;
        }
        if (goal_team != UnitTeam::Red && goal_team != UnitTeam::Blue) {
            return false;
        }

        const std::vector<std::string>* allowed_areas = nullptr;
        if (goal_team == my_team) {
            allowed_areas = &config.DecisionAutonomySettings.NaviGoal.MyArea;
        } else if (goal_team == enemy_team) {
            allowed_areas = &config.DecisionAutonomySettings.NaviGoal.EnemyArea;
        } else {
            return false;
        }
        if (allowed_areas == nullptr || allowed_areas->empty()) {
            return false;
        }

        const auto goal_point = GoalPointByBaseId(base_goal_id, goal_team);
        for (const auto& area_token : *allowed_areas) {
            const auto area_kind = MainAreaKindFromToken(area_token);
            if (!area_kind.has_value()) {
                continue;
            }
            if (Area::IsPointInsideMainArea(
                    goal_team,
                    *area_kind,
                    static_cast<int>(goal_point.x),
                    static_cast<int>(goal_point.y))) {
                return true;
            }
        }
        return false;
    }

    bool Application::TrySetScopedPositionByBaseGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const bool apply_team_offset,
        const char* reason) {
        if (!IsNaviGoalAllowedByAreaScope(base_goal_id, goal_team, my_team, enemy_team)) {
            naviGoalPublishAllowed_ = false;
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "DecisionAutonomy[navi_goal_area]: block goal={} team={} reason={}",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    goal_team == my_team ? "my" : "enemy",
                    reason ? reason : "area_scope");
            }
            return false;
        }

        SetPositionByBaseGoal(base_goal_id, goal_team, apply_team_offset);
        return true;
    }

    bool Application::TrySetRandomScopedPositionByBaseGoal(
        const std::vector<std::pair<std::uint8_t, UnitTeam>>& goals,
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const char* reason) {
        std::vector<std::pair<std::uint8_t, UnitTeam>> allowed_goals;
        allowed_goals.reserve(goals.size());
        for (const auto& goal : goals) {
            if (!IsValidBaseGoalId(goal.first)) {
                continue;
            }
            if (IsNaviGoalAllowedByAreaScope(goal.first, goal.second, my_team, enemy_team)) {
                allowed_goals.push_back(goal);
            }
        }
        if (allowed_goals.empty()) {
            if (IsNaviGoalAreaScopeEnabled()) {
                naviGoalPublishAllowed_ = false;
                if (LoggerPtr) {
                    LoggerPtr->Info(
                        "DecisionAutonomy[navi_goal_area]: block all random goals reason={}",
                        reason ? reason : "area_scope");
                }
            }
            return false;
        }

        Random random;
        const auto index = random.Get(0, static_cast<int>(allowed_goals.size()) - 1);
        const auto& selected = allowed_goals[static_cast<std::size_t>(index)];
        return TrySetScopedPositionByBaseGoal(
            selected.first,
            selected.second,
            my_team,
            enemy_team,
            true,
            reason);
    }

    void Application::SetPositionByBaseGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) {
        auto assign_position = [&](const auto& goal_location, const auto& area_location) {
            naviCommandGoal = apply_team_offset ? goal_location(goal_team) : goal_location.ID;
            naviGoalPosition = area_location(goal_team);
            naviGoalPublishAllowed_ = true;
        };

        switch (base_goal_id) {
            case LangYa::Home.ID: assign_position(LangYa::Home, BehaviorTree::Area::Home); break;
            case LangYa::Base.ID: assign_position(LangYa::Base, BehaviorTree::Area::Base); break;
            case LangYa::Recovery.ID: assign_position(LangYa::Recovery, BehaviorTree::Area::Recovery); break;
            case LangYa::BuffShoot.ID: assign_position(LangYa::BuffShoot, BehaviorTree::Area::BuffShoot); break;
            case LangYa::LeftHighLand.ID: assign_position(LangYa::LeftHighLand, BehaviorTree::Area::LeftHighLand); break;
            case LangYa::CastleLeft.ID: assign_position(LangYa::CastleLeft, BehaviorTree::Area::CastleLeft); break;
            case LangYa::Castle.ID: assign_position(LangYa::Castle, BehaviorTree::Area::Castle); break;
            case LangYa::CastleRight1.ID: assign_position(LangYa::CastleRight1, BehaviorTree::Area::CastleRight1); break;
            case LangYa::CastleRight2.ID: assign_position(LangYa::CastleRight2, BehaviorTree::Area::CastleRight2); break;
            case LangYa::FlyRoad.ID: assign_position(LangYa::FlyRoad, BehaviorTree::Area::FlyRoad); break;
            case LangYa::OutpostArea.ID: assign_position(LangYa::OutpostArea, BehaviorTree::Area::OutpostArea); break;
            case LangYa::MidShoot.ID: assign_position(LangYa::MidShoot, BehaviorTree::Area::MidShoot); break;
            case LangYa::LeftShoot.ID: assign_position(LangYa::LeftShoot, BehaviorTree::Area::LeftShoot); break;
            case LangYa::OutpostShoot.ID: assign_position(LangYa::OutpostShoot, BehaviorTree::Area::OutpostShoot); break;
            case LangYa::BuffAround1.ID: assign_position(LangYa::BuffAround1, BehaviorTree::Area::BuffAround1); break;
            case LangYa::BuffAround2.ID: assign_position(LangYa::BuffAround2, BehaviorTree::Area::BuffAround2); break;
            case LangYa::RightShoot.ID: assign_position(LangYa::RightShoot, BehaviorTree::Area::RightShoot); break;
            case LangYa::HoleRoad.ID: assign_position(LangYa::HoleRoad, BehaviorTree::Area::HoleRoad); break;
            case LangYa::OccupyArea.ID: assign_position(LangYa::OccupyArea, BehaviorTree::Area::OccupyArea); break;
            default:
                LoggerPtr->Warning("Unknown base goal id={}, fallback to Home.", static_cast<int>(base_goal_id));
                assign_position(LangYa::Home, BehaviorTree::Area::Home);
                break;
        }
    }

    void Application::SetPositionRepeat() {
        if(IsLeagueProfile()) SetPositionLeagueSimple();
        else if(IsShowcasePatrolEnabled()) SetPositionShowcasePatrol();
        else if(config.GameStrategySettings.HitSentry) SetPositionHitSentry();
        else if(config.GameStrategySettings.TestNavi) SetPositionNaviTest();
        else if(config.GameStrategySettings.Protected) SetPositionProtect();
        else SetPositionHitHero();
    }

    void Application::SetPositionLeagueSimple() {
        const auto& league = config.LeagueStrategySettings;
        const int hold_sec = std::max(1, league.GoalHoldSec);
        constexpr bool apply_team_offset = true;
        if (IsLeagueRouteCompatEnabled() && leagueRouteCompatAfterGatePending_) {
            leagueRouteCompatAfterGatePending_ = false;
            leagueRouteCompatActive_ = true;
            leagueRouteCompatUntil_ =
                std::chrono::steady_clock::now() + std::chrono::seconds(kLeagueRouteCompatViaHoldSec);
            leagueRouteCompatHasPendingGoal_ = false;
            leagueRouteCompatPendingBaseGoal_ = LangYa::Home.ID;
            leagueRouteCompatPendingHoldSec_ = 1;
            SetPositionByBaseGoal(kLeagueRouteCompatViaGoalBaseId, team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            LoggerPtr->Info(
                "League route compat after_gate: via goal={} for {}s.",
                static_cast<int>(naviCommandGoal),
                kLeagueRouteCompatViaHoldSec);
            return;
        }

        if (TickLeagueRouteCompat(team, apply_team_offset)) {
            return;
        }

        // 联赛模式优先做回补判定；命中后直接返回，不再切换巡航点。
        if (CheckPositionRecovery()) {
            LoggerPtr->Info("League profile recovery: health={} ammo={}", myselfHealth, ammoLeft);
            leaguePatrolGoalIndex_ = 0;
            leaguePatrolGoalInitialized_ = false;
            return;
        }

        std::vector<std::uint8_t> plan;
        plan.reserve(1 + league.PatrolGoals.size());
        // 计划路径由 MainGoal + PatrolGoals 去重组成。
        // 这里保证非法点位不会进入运行态。
        auto append_goal = [&](const std::uint8_t goal_id) {
            if (!IsValidBaseGoalId(goal_id)) {
                LoggerPtr->Warning("Skip invalid league goal id={}.", static_cast<int>(goal_id));
                return;
            }
            if (std::find(plan.begin(), plan.end(), goal_id) == plan.end()) {
                plan.push_back(goal_id);
            }
        };
        append_goal(league.MainGoal);
        for (const auto goal_id : league.PatrolGoals) {
            append_goal(goal_id);
        }
        if (plan.empty()) {
            append_goal(LangYa::OccupyArea.ID);
        }

        if (!leaguePatrolGoalInitialized_) {
            leaguePatrolGoalIndex_ = 0;
            const auto init_goal_id = ResolveGoalId(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            if (IsLeagueRouteCompatEnabled()) {
                if (IsLeagueGoalSwitchBetween2And3(
                        naviCommandGoal,
                        init_goal_id,
                        team,
                        apply_team_offset)) {
                    leaguePatrolGoalInitialized_ = true;
                    StartLeagueRouteCompat(
                        plan[leaguePatrolGoalIndex_],
                        hold_sec,
                        team,
                        apply_team_offset,
                        "init_2_3_switch");
                    return;
                }
            }
            SetPositionByBaseGoal(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            leaguePatrolGoalInitialized_ = true;
            LoggerPtr->Info("League profile init goal={}", static_cast<int>(naviCommandGoal));
            return;
        }

        const auto current_it = std::find_if(plan.begin(), plan.end(),
            [this](const std::uint8_t goal_id) {
                return naviCommandGoal == ResolveGoalId(goal_id, team);
            });
        if (current_it == plan.end()) {
            leaguePatrolGoalIndex_ = 0;
            const auto reset_goal_id = ResolveGoalId(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            if (IsLeagueRouteCompatEnabled() &&
                IsLeagueGoalSwitchBetween2And3(
                    naviCommandGoal,
                    reset_goal_id,
                    team,
                    apply_team_offset)) {
                StartLeagueRouteCompat(
                    plan[leaguePatrolGoalIndex_],
                    hold_sec,
                    team,
                    apply_team_offset,
                    "reset_2_3_switch");
                return;
            }
            SetPositionByBaseGoal(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            LoggerPtr->Info("League profile reset goal={}", static_cast<int>(naviCommandGoal));
            return;
        }
        leaguePatrolGoalIndex_ = static_cast<std::size_t>(std::distance(plan.begin(), current_it));

        if (plan.size() == 1U) {
            speedLevel = 1;
            return;
        }

        if (!naviCommandIntervalClock.trigger()) {
            speedLevel = 1;
            return;
        }

        if (plan.size() > 1U) {
            leaguePatrolGoalIndex_ = (leaguePatrolGoalIndex_ + 1U) % plan.size();
        }
        const auto target_base_goal = plan[leaguePatrolGoalIndex_];
        const auto target_goal_id = ResolveGoalId(target_base_goal, team, apply_team_offset);
        if (IsLeagueRouteCompatEnabled() &&
            IsLeagueGoalSwitchBetween2And3(
                naviCommandGoal,
                target_goal_id,
                team,
                apply_team_offset)) {
            StartLeagueRouteCompat(
                target_base_goal,
                hold_sec,
                team,
                apply_team_offset,
                "switch_2_3");
            return;
        }
        SetPositionByBaseGoal(target_base_goal, team, apply_team_offset);
        naviCommandIntervalClock.reset(Seconds{hold_sec});
        speedLevel = 1;
        LoggerPtr->Info("League profile switch goal={}", static_cast<int>(naviCommandGoal));
    }

    void Application::SetPositionShowcasePatrol() {
        const auto& showcase = config.ShowcasePatrolSettings;
        const bool apply_team_offset = !showcase.DisableTeamOffset;
        const int hold_sec = std::max(1, showcase.GoalHoldSec);
        const auto& plan = showcase.Goals;

        auto choose_index = [&](const bool initialize) -> std::size_t {
            if (plan.empty()) {
                return 0U;
            }
            if (!showcase.Random || plan.size() == 1U) {
                return initialize ? 0U : (showcasePatrolGoalIndex_ + 1U) % plan.size();
            }
            Random random;
            const auto upper_bound = static_cast<int>(plan.size()) - 1;
            std::size_t next_index = initialize
                ? static_cast<std::size_t>(random.Get(0, upper_bound))
                : showcasePatrolGoalIndex_;
            while (!initialize && next_index == showcasePatrolGoalIndex_) {
                next_index = static_cast<std::size_t>(random.Get(0, upper_bound));
            }
            return next_index;
        };

        auto apply_goal = [&](const std::size_t goal_index, const char* reason) {
            showcasePatrolGoalIndex_ = goal_index;
            SetPositionByBaseGoal(plan[goal_index], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            LoggerPtr->Info(
                "Showcase patrol {} goal_id={} raw_base_goal={}",
                reason,
                static_cast<int>(naviCommandGoal),
                static_cast<int>(plan[goal_index]));
        };

        if (!showcase.IgnoreRecovery && CheckPositionRecovery()) {
            LoggerPtr->Info("Showcase patrol recovery: health={} ammo={}", myselfHealth, ammoLeft);
            showcasePatrolGoalIndex_ = 0;
            showcasePatrolGoalInitialized_ = false;
            return;
        }

        if (plan.empty()) {
            SetPositionByBaseGoal(LangYa::OccupyArea.ID, team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            LoggerPtr->Warning("Showcase patrol has empty goal plan, fallback to OccupyArea.");
            return;
        }

        if (!showcasePatrolGoalInitialized_) {
            apply_goal(choose_index(true), "init");
            showcasePatrolGoalInitialized_ = true;
            return;
        }

        const auto current_it = std::find_if(plan.begin(), plan.end(),
            [&](const std::uint8_t goal_id) {
                return naviCommandGoal == ResolveGoalId(goal_id, team, apply_team_offset);
            });
        if (current_it == plan.end()) {
            apply_goal(choose_index(true), "reset");
            showcasePatrolGoalInitialized_ = true;
            return;
        }
        showcasePatrolGoalIndex_ = static_cast<std::size_t>(std::distance(plan.begin(), current_it));

        if (plan.size() == 1U) {
            speedLevel = 1;
            return;
        }

        if (!naviCommandIntervalClock.trigger()) {
            speedLevel = 1;
            return;
        }

        apply_goal(choose_index(false), "switch");
    }

    void Application::SetPositionNaviDebugPlan() {
        const auto& navi_debug = config.NaviDebugSettings;
        const bool apply_team_offset = !navi_debug.DisableTeamOffset;
        const int hold_sec = std::max(1, navi_debug.GoalHoldSec);
        const auto& plan = navi_debug.Goals;

        auto choose_index = [&](const bool initialize) -> std::size_t {
            if (plan.empty()) {
                return 0U;
            }
            if (!navi_debug.Random || plan.size() == 1U) {
                return initialize ? 0U : (naviDebugGoalIndex_ + 1U) % plan.size();
            }
            Random random;
            const auto upper_bound = static_cast<int>(plan.size()) - 1;
            std::size_t next_index = initialize
                ? static_cast<std::size_t>(random.Get(0, upper_bound))
                : naviDebugGoalIndex_;
            while (!initialize && next_index == naviDebugGoalIndex_) {
                next_index = static_cast<std::size_t>(random.Get(0, upper_bound));
            }
            return next_index;
        };

        auto apply_goal = [&](const std::size_t goal_index, const char* reason) {
            naviDebugGoalIndex_ = goal_index;
            SetPositionByBaseGoal(plan[goal_index], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = navi_debug.SpeedLevel;
            LoggerPtr->Info(
                "NaviDebug {} goal_id={} raw_base_goal={} speed_level={}",
                reason,
                static_cast<int>(naviCommandGoal),
                static_cast<int>(plan[goal_index]),
                static_cast<int>(speedLevel));
        };

        if (!navi_debug.IgnoreRecovery && CheckPositionRecovery()) {
            LoggerPtr->Info("NaviDebug recovery: health={} ammo={}", myselfHealth, ammoLeft);
            naviDebugGoalIndex_ = 0;
            naviDebugGoalInitialized_ = false;
            return;
        }

        if (plan.empty()) {
            SetPositionByBaseGoal(LangYa::OccupyArea.ID, team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = navi_debug.SpeedLevel;
            LoggerPtr->Warning("NaviDebug has empty goal plan, fallback to OccupyArea.");
            return;
        }

        if (!naviDebugGoalInitialized_) {
            apply_goal(choose_index(true), "init");
            naviDebugGoalInitialized_ = true;
            return;
        }

        const auto current_it = std::find_if(plan.begin(), plan.end(),
            [&](const std::uint8_t goal_id) {
                return naviCommandGoal == ResolveGoalId(goal_id, team, apply_team_offset);
            });
        if (current_it == plan.end()) {
            apply_goal(choose_index(true), "reset");
            naviDebugGoalInitialized_ = true;
            return;
        }
        naviDebugGoalIndex_ = static_cast<std::size_t>(std::distance(plan.begin(), current_it));

        if (plan.size() == 1U) {
            speedLevel = navi_debug.SpeedLevel;
            return;
        }

        if (!naviCommandIntervalClock.trigger()) {
            speedLevel = navi_debug.SpeedLevel;
            return;
        }

        apply_goal(choose_index(false), "switch");
    }

    bool Application::CheckPositionRecovery() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        int now_time = 420 - timeLeft;
        const bool disable_team_offset_for_debug =
            (config.ShowcasePatrolSettings.Enable && config.ShowcasePatrolSettings.DisableTeamOffset) ||
            (config.NaviDebugSettings.Enable && config.NaviDebugSettings.DisableTeamOffset);
        const bool apply_team_offset = IsLeagueProfile()
            ? true
            : !disable_team_offset_for_debug;
        const auto recovery_goal_id = ResolveGoalId(LangYa::Recovery.ID, MyTeam, apply_team_offset);
        if (IsLeagueProfile()) {
            // 联赛回补策略核心：
            // - 以裁判输入（自身血量/弹药）作为唯一触发源
            // - 带 stale 检查，避免旧数据误触发回补
            // - 回补失败后带 cooldown，防止频繁抖动
            const auto& league = config.LeagueStrategySettings;
            const auto now = std::chrono::steady_clock::now();
            if (TickLeagueRouteCompat(MyTeam, apply_team_offset)) {
                return true;
            }
            const std::uint16_t recovery_exit_min = league.HealthRecoveryExitMin;
            const std::uint16_t recovery_exit_preferred = league.HealthRecoveryExitPreferred;
            const auto recovery_plateau = std::chrono::seconds(league.HealthRecoveryPlateauSec);
            const auto recovery_max_hold = std::chrono::seconds(league.HealthRecoveryMaxHoldSec);
            const auto recovery_cooldown = std::chrono::seconds(league.HealthRecoveryCooldownSec);
            auto reset_league_recovery_state = [&]() {
                leagueRecoveryActive_ = false;
                leagueRecoveryStartTime_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryLastIncreaseTime_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryEntryHealth_ = 0;
                leagueRecoveryPeakHealth_ = 0;
            };
            auto is_referee_value_ready = [&](const bool has_received,
                                              const std::chrono::steady_clock::time_point& last_rx_time) {
                if (!has_received) {
                    return false;
                }
                if (leagueRefereeStaleTimeoutMs_ <= 0) {
                    return true;
                }
                if (last_rx_time.time_since_epoch().count() == 0) {
                    return false;
                }
                const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_rx_time).count();
                return age_ms <= leagueRefereeStaleTimeoutMs_;
            };

            const bool health_ready = is_referee_value_ready(hasReceivedMyselfHealth_, lastMyselfHealthRxTime);
            const bool health_low = league.UseHealthRecovery && health_ready &&
                myselfHealth < league.HealthRecoveryThreshold;
            const bool missing_health_input = league.UseHealthRecovery && !health_ready;
            const bool health_recovery_cooldown_active =
                leagueRecoveryCooldownUntil_.time_since_epoch().count() != 0 &&
                now < leagueRecoveryCooldownUntil_;
            const bool effective_health_low = health_low && !health_recovery_cooldown_active;

            if (missing_health_input &&
                (now - lastLeagueRecoveryGuardLogTime_ > std::chrono::seconds(2))) {
                LoggerPtr->Warning(
                    "League recovery guard: skip invalid referee inputs (health_ready={} stale_timeout_ms={})",
                    health_ready ? 1 : 0,
                    leagueRefereeStaleTimeoutMs_);
                lastLeagueRecoveryGuardLogTime_ = now;
            }
            if (health_recovery_cooldown_active &&
                (now - lastLeagueRecoveryGuardLogTime_ > std::chrono::seconds(2))) {
                const auto cooldown_left_sec = std::chrono::duration_cast<std::chrono::seconds>(
                    leagueRecoveryCooldownUntil_ - now).count();
                LoggerPtr->Warning(
                    "League recovery cooldown active: skip health-triggered Recovery for {}s.",
                    cooldown_left_sec > 0 ? cooldown_left_sec : 0);
                lastLeagueRecoveryGuardLogTime_ = now;
            }

            if (effective_health_low && !leagueRecoveryActive_) {
                // 进入“血量回补状态机”
                leagueRecoveryActive_ = true;
                leagueRecoveryStartTime_ = now;
                leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryLastIncreaseTime_ = now;
                leagueRecoveryEntryHealth_ = myselfHealth;
                leagueRecoveryPeakHealth_ = myselfHealth;
                LoggerPtr->Info(
                    "League health recovery activated: hp={} < threshold={}",
                    myselfHealth,
                    league.HealthRecoveryThreshold);
            }

            if (leagueRecoveryActive_) {
                if (health_ready && myselfHealth > leagueRecoveryPeakHealth_) {
                    leagueRecoveryPeakHealth_ = myselfHealth;
                    leagueRecoveryLastIncreaseTime_ = now;
                }

                const bool reach_preferred =
                    health_ready && myselfHealth >= recovery_exit_preferred;
                const bool reach_min_plateau =
                    health_ready &&
                    myselfHealth >= recovery_exit_min &&
                    leagueRecoveryLastIncreaseTime_.time_since_epoch().count() != 0 &&
                    (now - leagueRecoveryLastIncreaseTime_) >= recovery_plateau;
                const bool recovery_timeout =
                    leagueRecoveryStartTime_.time_since_epoch().count() != 0 &&
                    (now - leagueRecoveryStartTime_) >= recovery_max_hold;

                if (reach_preferred || reach_min_plateau) {
                    LoggerPtr->Info(
                        "League health recovery completed: hp={} (peak={}), return to normal strategy.",
                        myselfHealth,
                        leagueRecoveryPeakHealth_);
                    reset_league_recovery_state();
                    return false;
                }

                if (recovery_timeout) {
                    LoggerPtr->Warning(
                        "League health recovery timeout: entry_hp={} peak_hp={} current_hp={}, fallback to normal strategy.",
                        leagueRecoveryEntryHealth_,
                        leagueRecoveryPeakHealth_,
                        myselfHealth);
                    reset_league_recovery_state();
                    leagueRecoveryCooldownUntil_ = now + recovery_cooldown;
                    return false;
                }

                // 回补进行中：持续锁定 Recovery 点位，缩短导航重发间隔。
                if (IsLeagueRouteCompatEnabled() &&
                    IsLeagueGoalSwitchBetween2And3(
                        naviCommandGoal,
                        recovery_goal_id,
                        MyTeam,
                        apply_team_offset)) {
                    StartLeagueRouteCompat(
                        LangYa::Recovery.ID,
                        1,
                        MyTeam,
                        apply_team_offset,
                        "recovery_2_3_switch");
                    return true;
                }
                SetPositionByBaseGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            if (naviCommandGoal == recovery_goal_id &&
                (effective_health_low || missing_health_input)) {
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            if (effective_health_low) {
                if (IsLeagueRouteCompatEnabled() &&
                    IsLeagueGoalSwitchBetween2And3(
                        naviCommandGoal,
                        recovery_goal_id,
                        MyTeam,
                        apply_team_offset)) {
                    StartLeagueRouteCompat(
                        LangYa::Recovery.ID,
                        1,
                        MyTeam,
                        apply_team_offset,
                        "recovery_2_3_switch");
                    return true;
                }
                SetPositionByBaseGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }
            return false;
        }
        // 复活
        if(naviCommandGoal == recovery_goal_id) {
            if(myselfHealth < 380) {
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }
        }
        // 回家
        // 条件为：血量低于150 或者 弹药为0且距离上一次回家已经过去90秒
        if(myselfHealth < 150 || (ammoLeft <= 30 && recoveryClock.trigger())) {
            SetPositionByBaseGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
            recoveryClock.tick();
            naviCommandIntervalClock.reset(Seconds{1});
            return true;
        }
        return false;
    }

    bool Application::TrySetNaviGoalByAutonomy(
        const StrategyMode strategy_mode,
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        const auto& autonomy = config.DecisionAutonomySettings;
        const auto& enabled_modules = autonomy.EnabledModules;
        const bool has_specific_filter =
            HasAutonomyToken(enabled_modules, "navi_goal_hit_hero") ||
            HasAutonomyToken(enabled_modules, "navi_goal_hit_sentry") ||
            HasAutonomyToken(enabled_modules, "navi_goal_protect");
        const char* specific_module = nullptr;
        switch (strategy_mode) {
            case StrategyMode::HitHero:
                specific_module = "navi_goal_hit_hero";
                break;
            case StrategyMode::HitSentry:
                specific_module = "navi_goal_hit_sentry";
                break;
            case StrategyMode::Protected:
                specific_module = "navi_goal_protect";
                break;
            default:
                return false;
        }
        const bool base_enabled = IsDecisionAutonomyModuleEnabled("navi_goal");
        const bool specific_enabled = IsDecisionAutonomyModuleEnabled(specific_module);
        if (!base_enabled && !specific_enabled) {
            return false;
        }
        if (has_specific_filter && !specific_enabled && !HasAutonomyToken(enabled_modules, "all")) {
            return false;
        }

        const auto& navi_autonomy = autonomy.NaviGoal;
        std::vector<NaviGoalOption> options;
        if (navi_autonomy.UseCustomCandidates) {
            switch (strategy_mode) {
                case StrategyMode::HitHero:
                    options = navi_autonomy.HitHeroCandidates;
                    break;
                case StrategyMode::HitSentry:
                    options = navi_autonomy.HitSentryCandidates;
                    break;
                case StrategyMode::Protected:
                    options = navi_autonomy.ProtectCandidates;
                    break;
                default:
                    break;
            }
        }
        if (options.empty()) {
            if (strategy_mode == StrategyMode::HitHero || strategy_mode == StrategyMode::HitSentry) {
                options = {
                    NaviGoalOption{LangYa::MidShoot.ID, "my", 0.0, true},
                    NaviGoalOption{LangYa::BuffAround1.ID, "my", 0.0, true},
                    NaviGoalOption{LangYa::BuffAround2.ID, "my", 0.0, true},
                    NaviGoalOption{LangYa::RightShoot.ID, "my", 0.0, true},
                    NaviGoalOption{LangYa::MidShoot.ID, "enemy", 0.0, true},
                    NaviGoalOption{LangYa::BuffAround1.ID, "enemy", 0.0, true},
                    NaviGoalOption{LangYa::BuffAround2.ID, "enemy", 0.0, true},
                    NaviGoalOption{LangYa::RightShoot.ID, "enemy", 0.0, true},
                    NaviGoalOption{LangYa::LeftShoot.ID, "enemy", 0.0, true}
                };
            } else if (strategy_mode == StrategyMode::Protected) {
                options = {
                    NaviGoalOption{LangYa::CastleLeft.ID, "my", 0.0, true},
                    NaviGoalOption{LangYa::CastleRight1.ID, "my", 0.0, true},
                    NaviGoalOption{LangYa::CastleRight2.ID, "my", 0.0, true},
                    NaviGoalOption{LangYa::BuffShoot.ID, "my", 0.2, true}
                };
            }
        }
        if (options.empty()) {
            return false;
        }

        std::vector<RuntimeNaviGoalCandidate> candidates;
        candidates.reserve(options.size());
        int area_filtered_count = 0;
        for (const auto& option : options) {
            if (!option.Enable) {
                continue;
            }
            if (!IsValidBaseGoalId(option.GoalId)) {
                continue;
            }
            const UnitTeam candidate_team =
                NormalizeDecisionModule(option.Team) == "enemy" ? enemy_team : my_team;
            if (!IsNaviGoalAllowedByAreaScope(option.GoalId, candidate_team, my_team, enemy_team)) {
                ++area_filtered_count;
                continue;
            }
            candidates.push_back(RuntimeNaviGoalCandidate{
                .BaseGoalId = option.GoalId,
                .Team = candidate_team,
                .Bias = option.Bias
            });
        }
        if (candidates.empty()) {
            if (IsNaviGoalAreaScopeEnabled() && area_filtered_count > 0) {
                naviGoalPublishAllowed_ = false;
                LoggerPtr->Info(
                    "DecisionAutonomy[navi_goal]: all {} candidates blocked by MyArea/EnemyArea scope.",
                    area_filtered_count);
                return true;
            }
            return false;
        }

        const int self_x = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
        const int self_y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
        const bool self_pos_valid = self_x >= 0 && self_y >= 0;
        const int hero_x = static_cast<int>(enemyRobots[UnitType::Hero].position_.X);
        const int hero_y = static_cast<int>(enemyRobots[UnitType::Hero].position_.Y);
        const bool hero_pos_valid = hero_x >= 0 && hero_y >= 0;
        const bool low_energy =
            (teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000);
        const bool low_outpost = selfOutpostHealth <= 200;

        double best_score = -std::numeric_limits<double>::infinity();
        std::optional<RuntimeNaviGoalCandidate> best_candidate;
        for (const auto& candidate : candidates) {
            const auto goal_point = GoalPointByBaseId(candidate.BaseGoalId, candidate.Team);
            double score = navi_autonomy.GoalBiasWeight * candidate.Bias;
            if (self_pos_valid) {
                const double dx = static_cast<double>(goal_point.x) - static_cast<double>(self_x);
                const double dy = static_cast<double>(goal_point.y) - static_cast<double>(self_y);
                const double distance = std::sqrt(dx * dx + dy * dy);
                score -= navi_autonomy.DistanceWeight * (distance * 0.001);
            }
            if (candidate.Team == enemy_team) {
                score += navi_autonomy.EnemyTeamBonus;
            }
            if (naviCommandGoal == ResolveGoalId(candidate.BaseGoalId, candidate.Team, true)) {
                score += navi_autonomy.CurrentGoalBonus;
            }
            if (low_energy) {
                if (candidate.Team == my_team) {
                    score += navi_autonomy.LowEnergyOwnSideBonus;
                } else {
                    score -= navi_autonomy.LowEnergyEnemyPenalty;
                }
            }
            if (low_outpost && candidate.Team == my_team) {
                score += navi_autonomy.LowOutpostOwnSideBonus;
            }
            if (hero_pos_valid) {
                const double hdx = static_cast<double>(goal_point.x) - static_cast<double>(hero_x);
                const double hdy = static_cast<double>(goal_point.y) - static_cast<double>(hero_y);
                const double hero_distance = std::sqrt(hdx * hdx + hdy * hdy);
                score += navi_autonomy.HeroProximityWeight / (1.0 + hero_distance * 0.01);
            }

            if (score > best_score) {
                best_score = score;
                best_candidate = candidate;
            }
        }
        if (!best_candidate.has_value()) {
            return false;
        }

        if (!TrySetScopedPositionByBaseGoal(
                best_candidate->BaseGoalId,
                best_candidate->Team,
                my_team,
                enemy_team,
                true,
                "utility_pick")) {
            return true;
        }

        int hold_sec = 10;
        if (strategy_mode == StrategyMode::Protected) {
            hold_sec = (best_candidate->BaseGoalId == LangYa::BuffShoot.ID &&
                        best_candidate->Team == my_team)
                ? 30
                : 10;
        } else if (best_candidate->Team == enemy_team &&
                   (best_candidate->BaseGoalId == LangYa::MidShoot.ID ||
                    best_candidate->BaseGoalId == LangYa::LeftShoot.ID)) {
            hold_sec = 8;
        }
        naviCommandIntervalClock.reset(Seconds{hold_sec});
        LoggerPtr->Info(
            "DecisionAutonomy[navi_goal]: strategy={} pick_goal={} team={} score={:.3f} hold={}s",
            StrategyModeToString(strategy_mode),
            static_cast<int>(naviCommandGoal),
            best_candidate->Team == my_team ? "my" : "enemy",
            best_score,
            hold_sec);
        return true;
    }

    void Application::SetPositionProtect() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        int now_time = 420 - timeLeft;
        
        // 检测是否需要回家
        if(CheckPositionRecovery()) {
            // if(teamBuff.RemainingEnergy) speedLevel = 1;
            // else speedLevel = 2;
            LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
            LoggerPtr->Info("> Go Recovery");
            return;
        }

        // 防止高速切换指令
        if(!naviCommandIntervalClock.trigger()) {
            return;
        }        
        if(aimMode == AimMode::Buff) {
            TrySetScopedPositionByBaseGoal(
                LangYa::BuffShoot.ID, MyTeam, MyTeam, EnemyTeam, true, "protect_buff");
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 2; //加速
        }else if (aimMode == AimMode::Outpost) {
            TrySetScopedPositionByBaseGoal(
                LangYa::OutpostShoot.ID, MyTeam, MyTeam, EnemyTeam, true, "protect_outpost");
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 1; // 正常
        }else { //普通模式
            if (!TrySetNaviGoalByAutonomy(StrategyMode::Protected, MyTeam, EnemyTeam)) {
                const bool selected = TrySetRandomScopedPositionByBaseGoal(
                    {
                        {LangYa::CastleLeft.ID, MyTeam},
                        {LangYa::CastleRight1.ID, MyTeam},
                        {LangYa::CastleRight2.ID, MyTeam},
                        {LangYa::BuffShoot.ID, MyTeam}
                    },
                    MyTeam,
                    EnemyTeam,
                    "protect_fallback");
                if(selected && naviCommandGoal == BuffShoot(MyTeam)) naviCommandIntervalClock.reset(Seconds{30});
                else naviCommandIntervalClock.reset(Seconds(10));
            }
            // 时间超过5分钟 或 底盘能量低于5%
            if(now_time > 300 || 
                 
                teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
                LoggerPtr->Info("!!! Low Energy !!!");
                // naviCommandGoal = Castle(MyTeam);
                speedLevel = 0;
            } //else speedLevel = 1;
            LoggerPtr->Info("> NaviCommandGoal: {}", static_cast<int>(naviCommandGoal));
        }
    }

    void Application::SetPositionNaviTest() {
        if (config.NaviDebugSettings.Enable) {
            SetPositionNaviDebugPlan();
            return;
        }

        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();

        if(CheckPositionRecovery()) {
            // if(teamBuff.RemainingEnergy) speedLevel = 1;
            // else speedLevel = 2;
            LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
            LoggerPtr->Info("> Go Recovery");
            return;
        }
        // 防止高速切换指令
        if(!naviCommandIntervalClock.trigger()) {
            return;
        }
        
        if(now_time < 20) SET_POSITION(BuffShoot, MyTeam);
        else if(now_time < 40) SET_POSITION(LeftHighLand, MyTeam);
        else if(now_time < 60) SET_POSITION(CastleLeft, MyTeam);
        else if(now_time < 80) SET_POSITION(CastleRight1, MyTeam);
        else if(now_time < 100) SET_POSITION(CastleRight2, MyTeam);
        else if(now_time < 120) SET_POSITION(FlyRoad, MyTeam);
        else if(now_time < 140) SET_POSITION(OutpostArea, MyTeam);
        else if(now_time < 160) SET_POSITION(MidShoot, MyTeam);
        else if(now_time < 180) SET_POSITION(LeftShoot, MyTeam);
        else if(now_time < 200) SET_POSITION(OutpostShoot, MyTeam);
        else if(now_time < 220) SET_POSITION(FlyRoad, EnemyTeam);
        else if(now_time < 240) SET_POSITION(CastleRight1, EnemyTeam);
        else if(now_time < 260) SET_POSITION(CastleRight2, EnemyTeam);
        else if(now_time < 280) SET_POSITION(CastleLeft, EnemyTeam);
        else if(now_time < 300) SET_POSITION(LeftHighLand, EnemyTeam);
        else if(now_time < 320) SET_POSITION(BuffShoot, EnemyTeam);
        else if(now_time < 340) SET_POSITION(OutpostArea, EnemyTeam);
        else SET_POSITION(Castle, MyTeam);
    }

    void Application::SetPositionHitSentry() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();

        if(CheckPositionRecovery()) {
            // if(teamBuff.RemainingEnergy) speedLevel = 1;
            // else speedLevel = 2;
            LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
            LoggerPtr->Info("> Go Recovery");
            return;
        }
        // 防止高速切换指令
        if(!naviCommandIntervalClock.trigger()) {
            return;
        }
        if(aimMode == AimMode::Buff) {
            TrySetScopedPositionByBaseGoal(
                LangYa::BuffShoot.ID, MyTeam, MyTeam, EnemyTeam, true, "hit_sentry_buff");
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 2;
        }else if (aimMode == AimMode::Outpost) {
            TrySetScopedPositionByBaseGoal(
                LangYa::OutpostShoot.ID, MyTeam, MyTeam, EnemyTeam, true, "hit_sentry_outpost");
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 1;
        }else { //普通模式
            if (!TrySetNaviGoalByAutonomy(StrategyMode::HitSentry, MyTeam, EnemyTeam)) {
                // if(!hitableTargets.empty()) { // 视野里存在目标
                //     bool low_health_enemy = false;
                //     // 检测低血量敌人
                //     for(auto robot : hitableTargets) {
                //         if(enemyRobots[robot].currentHealth_ < 50) {
                //             low_health_enemy = true;
                //         }
                //     }
                bool infantry1_in_central = 
                    Area::CentralHighLandBlue.isPointInside(enemyRobots[UnitType::Infantry1].position_.X, enemyRobots[UnitType::Infantry1].position_.Y)
                    || Area::CentralHighLandBlue.isPointInside(enemyRobots[UnitType::Infantry1].position_.X, enemyRobots[UnitType::Infantry1].position_.Y);
                bool infantry2_in_central = 
                    Area::CentralHighLandBlue.isPointInside(enemyRobots[UnitType::Infantry2].position_.X, enemyRobots[UnitType::Infantry2].position_.Y)
                    || Area::CentralHighLandBlue.isPointInside(enemyRobots[UnitType::Infantry2].position_.X, enemyRobots[UnitType::Infantry2].position_.Y);

                if(infantry1_in_central) {
                    LoggerPtr->Debug("Infantry1 in Highland");
                }
                if(infantry2_in_central) {
                    LoggerPtr->Debug("Infantry2 in Highland");
                }
                
                if(selfOutpostHealth > 100 && now_time < 55 ) {
                    const bool selected = TrySetRandomScopedPositionByBaseGoal(
                        {
                            {LangYa::MidShoot.ID, MyTeam},
                            {LangYa::BuffAround1.ID, MyTeam},
                            {LangYa::BuffAround2.ID, MyTeam},
                            {LangYa::RightShoot.ID, MyTeam},
                            {LangYa::MidShoot.ID, EnemyTeam},
                            {LangYa::BuffAround1.ID, EnemyTeam},
                            {LangYa::BuffAround2.ID, EnemyTeam},
                            {LangYa::RightShoot.ID, EnemyTeam},
                            {LangYa::LeftShoot.ID, EnemyTeam}
                        },
                        MyTeam,
                        EnemyTeam,
                        "hit_sentry_fallback");
                    if(selected &&
                       (naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam)))
                        naviCommandIntervalClock.reset(Seconds(8));
                    else naviCommandIntervalClock.reset(Seconds(10));
                }else {
                    TrySetScopedPositionByBaseGoal(
                        LangYa::FlyRoad.ID, EnemyTeam, MyTeam, EnemyTeam, true, "hit_sentry_late");
                    naviCommandIntervalClock.reset(Seconds(10));
                }
            }
                    
             // 底盘能量低于5%
            if(teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
                LoggerPtr->Info("!!! Low Energy !!!");
                TrySetScopedPositionByBaseGoal(
                    LangYa::HoleRoad.ID, MyTeam, MyTeam, EnemyTeam, true, "hit_sentry_low_energy");
                naviCommandIntervalClock.reset(Seconds(10));
                speedLevel = 0;
            } // else speedLevel = 1;
            speedLevel = 1;
            LoggerPtr->Info("> NaviCommandGoal: {}", static_cast<int>(naviCommandGoal));
        }
    }

    void Application::SetPositionHitHero() {
        if (IsLeagueProfile()) {
            SetPositionLeagueSimple();
            return;
        }
        if (IsShowcasePatrolEnabled()) {
            SetPositionShowcasePatrol();
            return;
        }

        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();

        if(CheckPositionRecovery()) {
            // if(teamBuff.RemainingEnergy) speedLevel = 1;
            // else speedLevel = 2;
            LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
            LoggerPtr->Info("> Go Recovery");
            return;
        }
        // 防止高速切换指令
        if(!naviCommandIntervalClock.trigger()) {
            return;
        }
        if(aimMode == AimMode::Buff) {
            TrySetScopedPositionByBaseGoal(
                LangYa::BuffShoot.ID, MyTeam, MyTeam, EnemyTeam, true, "hit_hero_buff");
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 2;
        }else if (aimMode == AimMode::Outpost) {
            TrySetScopedPositionByBaseGoal(
                LangYa::OutpostShoot.ID, MyTeam, MyTeam, EnemyTeam, true, "hit_hero_outpost");
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 1;
        }else { //普通模式
            const bool selected_by_autonomy =
                TrySetNaviGoalByAutonomy(StrategyMode::HitHero, MyTeam, EnemyTeam);
            if (!selected_by_autonomy) {
                // 判断英雄是否处于高地
                bool hero_in_central = false;
                std::int16_t hero_x = enemyRobots[UnitType::Hero].position_.X, hero_y = enemyRobots[UnitType::Hero].position_.Y;
                hero_in_central = Area::CentralHighLandRed.isPointInside(hero_x, hero_y) || Area::CentralHighLandBlue.isPointInside(hero_x, hero_y);

                if(hero_in_central) {
                    LoggerPtr->Debug("!!!Hero in highland!!!");
                    TrySetScopedPositionByBaseGoal(
                        LangYa::BuffAround1.ID, MyTeam, MyTeam, EnemyTeam, true, "hit_hero_highland");
                }else {
                    if(selfOutpostHealth > 200) {
                        int redpx = 982, redpy = 1124;
                        int dx = redpx - enemyRobots[UnitType::Hero].position_.X, dy = redpy - enemyRobots[UnitType::Hero].position_.Y;
                        int len = std::sqrt(dx * dx + dy * dy);
                        if(len < 100) {
                            TrySetScopedPositionByBaseGoal(
                                LangYa::HoleRoad.ID, EnemyTeam, MyTeam, EnemyTeam, true, "hit_hero_close");
                            naviCommandIntervalClock.reset(Seconds(2));
                        }else {
                            const bool selected = TrySetRandomScopedPositionByBaseGoal(
                                {
                                    {LangYa::MidShoot.ID, MyTeam},
                                    {LangYa::BuffAround1.ID, MyTeam},
                                    {LangYa::BuffAround2.ID, MyTeam},
                                    {LangYa::RightShoot.ID, MyTeam},
                                    {LangYa::MidShoot.ID, EnemyTeam},
                                    {LangYa::BuffAround1.ID, EnemyTeam},
                                    {LangYa::BuffAround2.ID, EnemyTeam},
                                    {LangYa::RightShoot.ID, EnemyTeam},
                                    {LangYa::LeftShoot.ID, EnemyTeam}
                                },
                                MyTeam,
                                EnemyTeam,
                                "hit_hero_fallback");
                            if(selected &&
                               (naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam)))
                                naviCommandIntervalClock.reset(Seconds(8));
                            else naviCommandIntervalClock.reset(Seconds(10));
                        }
                    }else {
                        if(selfBaseHealth > 2000){
                            TrySetScopedPositionByBaseGoal(
                                LangYa::HoleRoad.ID, EnemyTeam, MyTeam, EnemyTeam, true, "hit_hero_base_high");
                            naviCommandIntervalClock.reset(Seconds(2));
                        }
                        else {
                            const bool selected = TrySetRandomScopedPositionByBaseGoal(
                                {
                                    {LangYa::MidShoot.ID, MyTeam},
                                    {LangYa::BuffAround1.ID, MyTeam},
                                    {LangYa::BuffAround2.ID, MyTeam},
                                    {LangYa::RightShoot.ID, MyTeam},
                                    {LangYa::MidShoot.ID, EnemyTeam},
                                    {LangYa::BuffAround1.ID, EnemyTeam},
                                    {LangYa::BuffAround2.ID, EnemyTeam},
                                    {LangYa::RightShoot.ID, EnemyTeam},
                                    {LangYa::LeftShoot.ID, EnemyTeam}
                                },
                                MyTeam,
                                EnemyTeam,
                                "hit_hero_base_low");
                            if(selected &&
                               (naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam)))
                                naviCommandIntervalClock.reset(Seconds(8));
                            else naviCommandIntervalClock.reset(Seconds(10));
                        }
                    }
                }
            }
            
            // 底盘能量低于5%
            if(teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
                LoggerPtr->Info("!!! Low Energy !!!");
                TrySetRandomScopedPositionByBaseGoal(
                    {
                        {LangYa::BuffAround1.ID, MyTeam},
                        {LangYa::BuffAround2.ID, MyTeam},
                        {LangYa::RightShoot.ID, MyTeam}
                    },
                    MyTeam,
                    EnemyTeam,
                    "hit_hero_low_energy");
                naviCommandIntervalClock.reset(Seconds(10));
                speedLevel = 0;
            } // else speedLevel = 1;
            speedLevel = 1;
            LoggerPtr->Info("> NaviCommandGoal: {}", static_cast<int>(naviCommandGoal));
        }
    }



}
