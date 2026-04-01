// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace {

std::string NormalizeProfile(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

BehaviorTree::CompetitionProfile ParseCompetitionProfile(const std::string& value) {
    const auto normalized = NormalizeProfile(value);
    if (normalized == "league") {
        return BehaviorTree::CompetitionProfile::League;
    }
    return BehaviorTree::CompetitionProfile::Regional;
}

bool IsValidBaseGoal(const std::uint8_t goal_id) {
    return goal_id <= LangYa::OccupyArea.ID;
}

std::string ResolveBehaviorTreeConfigPath(const std::string& configured_path) {
    if (configured_path.empty()) {
        return {};
    }

    const std::filesystem::path path(configured_path);
    if (path.is_absolute()) {
        return path.lexically_normal().string();
    }

    try {
        const auto pkg_path = ament_index_cpp::get_package_share_directory("behavior_tree");
        return (std::filesystem::path(pkg_path) / path).lexically_normal().string();
    } catch (...) {
        return path.lexically_normal().string();
    }
}

bool LoadNaviDebugPlanFile(LangYa::NaviDebugSetting& nd, const std::shared_ptr<Logger>& logger) {
    const std::string default_plan_file = "Scripts/ConfigJson/navi_debug_points.json";
    nd.PlanFile = ResolveBehaviorTreeConfigPath(nd.PlanFile.empty() ? default_plan_file : nd.PlanFile);

    std::ifstream ifs(nd.PlanFile);
    if (!ifs.is_open()) {
        if (logger) {
            logger->Warning("Failed to open NaviDebug plan file: {}", nd.PlanFile);
        }
        return false;
    }

    nlohmann::json root;
    ifs >> root;

    const nlohmann::json* selected_plan = nullptr;
    std::string selected_plan_name = nd.ActivePlan.empty()
        ? root.value("ActivePlan", std::string{})
        : nd.ActivePlan;

    const auto plans_it = root.find("Plans");
    if (plans_it != root.end() && plans_it->is_object()) {
        if (selected_plan_name.empty()) {
            if (plans_it->size() == 1U) {
                selected_plan_name = plans_it->begin().key();
            } else if (plans_it->contains("default")) {
                selected_plan_name = "default";
            }
        }
        if (!selected_plan_name.empty()) {
            const auto selected_it = plans_it->find(selected_plan_name);
            if (selected_it != plans_it->end() && selected_it->is_object()) {
                selected_plan = &(*selected_it);
            }
        }
        if (selected_plan == nullptr && !plans_it->empty()) {
            selected_plan_name = plans_it->begin().key();
            selected_plan = &plans_it->begin().value();
            if (logger) {
                logger->Warning("NaviDebug plan '{}' not found, fallback to first plan '{}'.",
                    nd.ActivePlan, selected_plan_name);
            }
        }
    } else {
        selected_plan = &root;
        if (selected_plan_name.empty()) {
            selected_plan_name = "<root>";
        }
    }

    if (selected_plan == nullptr || !selected_plan->is_object()) {
        if (logger) {
            logger->Warning("NaviDebug plan file '{}' has no valid plan object.", nd.PlanFile);
        }
        return false;
    }

    nd.ActivePlan = selected_plan_name;
    nd.GoalHoldSec = selected_plan->value("GoalHoldSec", nd.GoalHoldSec);
    nd.DisableTeamOffset = selected_plan->value("DisableTeamOffset", nd.DisableTeamOffset);
    nd.IgnoreRecovery = selected_plan->value("IgnoreRecovery", nd.IgnoreRecovery);
    nd.SpeedLevel = static_cast<std::uint8_t>(
        std::clamp(selected_plan->value("SpeedLevel", static_cast<int>(nd.SpeedLevel)), 0, 2));

    const std::string plan_mode = NormalizeProfile(selected_plan->value(
        "Mode", nd.Random ? std::string("random") : std::string("sequence")));
    if (plan_mode == "random") {
        nd.Random = true;
    } else if (plan_mode == "sequence" || plan_mode == "sequential" || plan_mode == "ordered") {
        nd.Random = false;
    } else {
        nd.Random = selected_plan->value("Random", nd.Random);
    }

    if (selected_plan->contains("Goals")) {
        selected_plan->at("Goals").get_to(nd.Goals);
    } else {
        nd.Goals.clear();
    }

    if (logger) {
        logger->Info(
            "Loaded NaviDebug plan file='{}' active_plan='{}' mode={} goals={} hold_sec={} speed={} disable_team_offset={} ignore_recovery={}",
            nd.PlanFile,
            nd.ActivePlan,
            nd.Random ? "random" : "sequence",
            nd.Goals.size(),
            nd.GoalHoldSec,
            static_cast<int>(nd.SpeedLevel),
            nd.DisableTeamOffset ? 1 : 0,
            nd.IgnoreRecovery ? 1 : 0);
    }
    return true;
}

}  // namespace


namespace LangYa {

    // 自定义 from_json 函数，用于自动转换 JSON 到结构体
    void from_json(const json& j, AimDebug& ad) {
        ad.StopFire = j.value("StopFire", ad.StopFire);
        ad.StopRotate = j.value("StopRotate", ad.StopRotate);
        ad.StopScan = j.value("StopScan", ad.StopScan);
        ad.HitOutpost = j.value("HitOutpost", ad.HitOutpost);
        ad.HitBuff = j.value("HitBuff", ad.HitBuff);
        ad.HitCar = j.value("HitCar", ad.HitCar);
        ad.FireRequireTargetStatus = j.value("FireRequireTargetStatus", ad.FireRequireTargetStatus);
    }

    void from_json(const json& j, Rate& r) {
        r.FireRate = j.value("FireRate", r.FireRate);
        r.TreeTickRate = j.value("TreeTickRate", r.TreeTickRate);
        r.NaviCommandRate = j.value("NaviCommandRate", r.NaviCommandRate);
    }
    void from_json(const json& j, GameStrategy& gs) {
        gs.HitBuff = j.value("HitBuff", gs.HitBuff);
        gs.HitOutpost = j.value("HitOutpost", gs.HitOutpost);
        gs.TestNavi = j.value("TestNavi", gs.TestNavi);
        gs.HitSentry = j.value("HitSentry", gs.HitSentry);
        gs.Protected = j.value("Protected", gs.Protected);
    }
    void from_json(const json& j, NaviSetting& ns) {
        ns.UseXY = j.value("UseXY", ns.UseXY);
    }

    void from_json(const json& j, LeagueStrategySetting& ls) {
        ls.EnableRouteCompat = j.value("EnableRouteCompat", ls.EnableRouteCompat);
        ls.UseHealthRecovery = j.value("UseHealthRecovery", ls.UseHealthRecovery);
        ls.HealthRecoveryThreshold = j.value("HealthRecoveryThreshold", ls.HealthRecoveryThreshold);
        ls.UseAmmoRecovery = j.value("UseAmmoRecovery", ls.UseAmmoRecovery);
        ls.AmmoRecoveryThreshold = j.value("AmmoRecoveryThreshold", ls.AmmoRecoveryThreshold);
        ls.DamageScanBoostEnable = j.value("DamageScanBoostEnable", ls.DamageScanBoostEnable);
        ls.HealthRecoveryExitMin = j.value("HealthRecoveryExitMin", ls.HealthRecoveryExitMin);
        ls.HealthRecoveryExitPreferred = j.value("HealthRecoveryExitPreferred", ls.HealthRecoveryExitPreferred);
        ls.HealthRecoveryPlateauSec = j.value("HealthRecoveryPlateauSec", ls.HealthRecoveryPlateauSec);
        ls.HealthRecoveryExitStableSec = j.value("HealthRecoveryExitStableSec", ls.HealthRecoveryExitStableSec);
        ls.HealthRecoveryMaxHoldSec = j.value("HealthRecoveryMaxHoldSec", ls.HealthRecoveryMaxHoldSec);
        ls.HealthRecoveryCooldownSec = j.value("HealthRecoveryCooldownSec", ls.HealthRecoveryCooldownSec);
        ls.EnableDamageOpenGate = j.value("EnableDamageOpenGate", ls.EnableDamageOpenGate);
        ls.DamageOpenGateThreshold = j.value("DamageOpenGateThreshold", ls.DamageOpenGateThreshold);
        ls.MainGoal = j.value("MainGoal", ls.MainGoal);
        ls.GoalHoldSec = j.value("GoalHoldSec", ls.GoalHoldSec);
        if (j.contains("PatrolGoals")) {
            j.at("PatrolGoals").get_to(ls.PatrolGoals);
        }
    }

    void from_json(const json& j, ShowcasePatrolSetting& sp) {
        sp.Enable = j.value("Enable", sp.Enable);
        sp.GoalHoldSec = j.value("GoalHoldSec", sp.GoalHoldSec);
        sp.Random = j.value("Random", sp.Random);
        sp.DisableTeamOffset = j.value("DisableTeamOffset", sp.DisableTeamOffset);
        sp.IgnoreRecovery = j.value("IgnoreRecovery", sp.IgnoreRecovery);
        if (j.contains("Goals")) {
            j.at("Goals").get_to(sp.Goals);
        }
    }

    void from_json(const json& j, NaviDebugSetting& nd) {
        nd.Enable = j.value("Enable", nd.Enable);
        nd.PlanFile = j.value("PlanFile", nd.PlanFile);
        nd.ActivePlan = j.value("ActivePlan", nd.ActivePlan);
        nd.GoalHoldSec = j.value("GoalHoldSec", nd.GoalHoldSec);
        nd.Random = j.value("Random", nd.Random);
        nd.DisableTeamOffset = j.value("DisableTeamOffset", nd.DisableTeamOffset);
        nd.IgnoreRecovery = j.value("IgnoreRecovery", nd.IgnoreRecovery);
        nd.SpeedLevel = static_cast<std::uint8_t>(
            std::clamp(j.value("SpeedLevel", static_cast<int>(nd.SpeedLevel)), 0, 2));
        if (j.contains("Goals")) {
            j.at("Goals").get_to(nd.Goals);
        }
    }

    void from_json(const json& j, PostureSetting& ps) {
        ps.Enable = j.value("Enable", ps.Enable);
        ps.SwitchCooldownSec = j.value("SwitchCooldownSec", ps.SwitchCooldownSec);
        ps.MaxSinglePostureSec = j.value("MaxSinglePostureSec", ps.MaxSinglePostureSec);
        ps.EarlyRotateSec = j.value("EarlyRotateSec", ps.EarlyRotateSec);
        ps.MinHoldSec = j.value("MinHoldSec", ps.MinHoldSec);
        ps.PendingAckTimeoutMs = j.value("PendingAckTimeoutMs", ps.PendingAckTimeoutMs);
        ps.RetryIntervalMs = j.value("RetryIntervalMs", ps.RetryIntervalMs);
        ps.MaxRetryCount = j.value("MaxRetryCount", ps.MaxRetryCount);
        ps.OptimisticAck = j.value("OptimisticAck", ps.OptimisticAck);
        ps.TargetKeepMs = j.value("TargetKeepMs", ps.TargetKeepMs);
        ps.DamageKeepSec = j.value("DamageKeepSec", ps.DamageKeepSec);
        ps.DamageBurstWindowMs = j.value("DamageBurstWindowMs", ps.DamageBurstWindowMs);
        ps.DamageBurstThreshold = j.value("DamageBurstThreshold", ps.DamageBurstThreshold);
        ps.DamageBurstDefenseHoldSec = j.value("DamageBurstDefenseHoldSec", ps.DamageBurstDefenseHoldSec);
        ps.LowHealthThreshold = j.value("LowHealthThreshold", ps.LowHealthThreshold);
        ps.VeryLowHealthThreshold = j.value("VeryLowHealthThreshold", ps.VeryLowHealthThreshold);
        ps.LowAmmoThreshold = j.value("LowAmmoThreshold", ps.LowAmmoThreshold);
        ps.ScoreHysteresis = j.value("ScoreHysteresis", ps.ScoreHysteresis);
    }

    void from_json(const json& j, Config& c) {
        if (j.contains("AimDebug")) {
            j.at("AimDebug").get_to(c.AimDebugSettings);
        }
        if (j.contains("Rate")) {
            j.at("Rate").get_to(c.RateSettings);
        }
        if (j.contains("GameStrategy")) {
            j.at("GameStrategy").get_to(c.GameStrategySettings);
        }
        c.ScanCounter = j.value("ScanCounter", c.ScanCounter);
        if (j.contains("NaviSetting")) {
            j.at("NaviSetting").get_to(c.NaviSettings);
        }
        if (j.contains("LeagueStrategy")) {
            j.at("LeagueStrategy").get_to(c.LeagueStrategySettings);
        }
        if (j.contains("ShowcasePatrol")) {
            j.at("ShowcasePatrol").get_to(c.ShowcasePatrolSettings);
        }
        if (j.contains("NaviDebug")) {
            j.at("NaviDebug").get_to(c.NaviDebugSettings);
        }
        if (j.contains("Posture")) {
            j.at("Posture").get_to(c.PostureSettings);
        }
        c.CompetitionProfile = j.value("CompetitionProfile", c.CompetitionProfile);
    }
}

namespace BehaviorTree {
    using namespace LangYa;
    using json = nlohmann::json;

    bool Application::ConfigurationInit() {
        std::ifstream ifs(config_file_);
        if (!ifs.is_open()) {
            LoggerPtr->Error("Failed to open config.json, file location: {}", config_file_);
            return false;
        }
        LoggerPtr->Info("Open config.json, file location: {}", config_file_);

        // 解析 JSON 文件
        json j;
        ifs >> j;
        // 一次性反序列化到 Config，后续再做范围校验与默认回退。
        config = j.get<Config>();
        LoggerPtr->Debug("------ AimDebug ------");
        LoggerPtr->Debug("StopFire: {}", config.AimDebugSettings.StopFire);
        LoggerPtr->Debug("StopRotate: {}", config.AimDebugSettings.StopRotate);
        LoggerPtr->Debug("StopScan: {}", config.AimDebugSettings.StopScan);
        LoggerPtr->Debug("HitOutpost: {}", config.AimDebugSettings.HitOutpost);
        LoggerPtr->Debug("HitBuff: {}", config.AimDebugSettings.HitBuff);
        LoggerPtr->Debug("HitCar: {}", config.AimDebugSettings.HitCar);
        LoggerPtr->Debug("FireRequireTargetStatus: {}", config.AimDebugSettings.FireRequireTargetStatus);
        LoggerPtr->Debug("------ Rate ------");
        LoggerPtr->Debug("FireRate: {}", config.RateSettings.FireRate);
        LoggerPtr->Debug("TickRate: {}", config.RateSettings.TreeTickRate);
        LoggerPtr->Debug("NaviCommandRate: {}", config.RateSettings.NaviCommandRate);
        LoggerPtr->Debug("------ GameStrategy ------");
        LoggerPtr->Debug("ScanCounter: {}", config.ScanCounter);
        LoggerPtr->Debug("HitOutpost: {}", config.GameStrategySettings.HitOutpost);
        LoggerPtr->Debug("HitBuff: {}", config.GameStrategySettings.HitBuff);
        LoggerPtr->Debug("HitSentry: {}", config.GameStrategySettings.HitSentry);
        LoggerPtr->Debug("TestNavi: {}", config.GameStrategySettings.TestNavi);
        LoggerPtr->Debug("Protected: {}", config.GameStrategySettings.Protected);
        LoggerPtr->Debug("------ NaviSetting ------");
        LoggerPtr->Debug("UseXY: {}", config.NaviSettings.UseXY);
        LoggerPtr->Debug("------ LeagueStrategy ------");
        LoggerPtr->Debug("EnableRouteCompat: {}", config.LeagueStrategySettings.EnableRouteCompat);
        LoggerPtr->Debug("UseHealthRecovery: {}", config.LeagueStrategySettings.UseHealthRecovery);
        LoggerPtr->Debug("HealthRecoveryThreshold: {}", config.LeagueStrategySettings.HealthRecoveryThreshold);
        LoggerPtr->Debug("UseAmmoRecovery: {}", config.LeagueStrategySettings.UseAmmoRecovery);
        LoggerPtr->Debug("AmmoRecoveryThreshold: {}", config.LeagueStrategySettings.AmmoRecoveryThreshold);
        LoggerPtr->Debug("DamageScanBoostEnable: {}", config.LeagueStrategySettings.DamageScanBoostEnable);
        LoggerPtr->Debug("HealthRecoveryExitMin: {}", config.LeagueStrategySettings.HealthRecoveryExitMin);
        LoggerPtr->Debug("HealthRecoveryExitPreferred: {}", config.LeagueStrategySettings.HealthRecoveryExitPreferred);
        LoggerPtr->Debug("HealthRecoveryPlateauSec: {}", config.LeagueStrategySettings.HealthRecoveryPlateauSec);
        LoggerPtr->Debug("HealthRecoveryExitStableSec: {}", config.LeagueStrategySettings.HealthRecoveryExitStableSec);
        LoggerPtr->Debug("HealthRecoveryMaxHoldSec: {}", config.LeagueStrategySettings.HealthRecoveryMaxHoldSec);
        LoggerPtr->Debug("HealthRecoveryCooldownSec: {}", config.LeagueStrategySettings.HealthRecoveryCooldownSec);
        LoggerPtr->Debug("EnableDamageOpenGate: {}", config.LeagueStrategySettings.EnableDamageOpenGate);
        LoggerPtr->Debug("DamageOpenGateThreshold: {}", config.LeagueStrategySettings.DamageOpenGateThreshold);
        LoggerPtr->Debug("MainGoal: {}", static_cast<int>(config.LeagueStrategySettings.MainGoal));
        LoggerPtr->Debug("GoalHoldSec: {}", config.LeagueStrategySettings.GoalHoldSec);
        LoggerPtr->Debug("------ ShowcasePatrol ------");
        LoggerPtr->Debug("Enable: {}", config.ShowcasePatrolSettings.Enable);
        LoggerPtr->Debug("GoalHoldSec: {}", config.ShowcasePatrolSettings.GoalHoldSec);
        LoggerPtr->Debug("Random: {}", config.ShowcasePatrolSettings.Random);
        LoggerPtr->Debug("DisableTeamOffset: {}", config.ShowcasePatrolSettings.DisableTeamOffset);
        LoggerPtr->Debug("IgnoreRecovery: {}", config.ShowcasePatrolSettings.IgnoreRecovery);
        LoggerPtr->Debug("------ NaviDebug ------");
        LoggerPtr->Debug("Enable: {}", config.NaviDebugSettings.Enable);
        LoggerPtr->Debug("PlanFile: {}", config.NaviDebugSettings.PlanFile);
        LoggerPtr->Debug("ActivePlan: {}", config.NaviDebugSettings.ActivePlan);
        LoggerPtr->Debug("------ PostureSetting ------");
        LoggerPtr->Debug("Enable: {}", config.PostureSettings.Enable);
        LoggerPtr->Debug("SwitchCooldownSec: {}", config.PostureSettings.SwitchCooldownSec);
        LoggerPtr->Debug("MaxSinglePostureSec: {}", config.PostureSettings.MaxSinglePostureSec);
        LoggerPtr->Debug("EarlyRotateSec: {}", config.PostureSettings.EarlyRotateSec);
        LoggerPtr->Debug("MinHoldSec: {}", config.PostureSettings.MinHoldSec);
        LoggerPtr->Debug("PendingAckTimeoutMs: {}", config.PostureSettings.PendingAckTimeoutMs);
        LoggerPtr->Debug("RetryIntervalMs: {}", config.PostureSettings.RetryIntervalMs);
        LoggerPtr->Debug("MaxRetryCount: {}", config.PostureSettings.MaxRetryCount);
        LoggerPtr->Debug("OptimisticAck: {}", config.PostureSettings.OptimisticAck);
        LoggerPtr->Debug("TargetKeepMs: {}", config.PostureSettings.TargetKeepMs);
        LoggerPtr->Debug("DamageKeepSec: {}", config.PostureSettings.DamageKeepSec);
        LoggerPtr->Debug("DamageBurstWindowMs: {}", config.PostureSettings.DamageBurstWindowMs);
        LoggerPtr->Debug("DamageBurstThreshold: {}", config.PostureSettings.DamageBurstThreshold);
        LoggerPtr->Debug("DamageBurstDefenseHoldSec: {}", config.PostureSettings.DamageBurstDefenseHoldSec);
        LoggerPtr->Debug("LowHealthThreshold: {}", config.PostureSettings.LowHealthThreshold);
        LoggerPtr->Debug("VeryLowHealthThreshold: {}", config.PostureSettings.VeryLowHealthThreshold);
        LoggerPtr->Debug("LowAmmoThreshold: {}", config.PostureSettings.LowAmmoThreshold);
        LoggerPtr->Debug("ScoreHysteresis: {}", config.PostureSettings.ScoreHysteresis);
        LoggerPtr->Debug("------ End ------");
        LoggerPtr->Debug("Configuration completed.");
        fireRateClock.reset(config.RateSettings.FireRate);
        treeTickRateClock.reset(config.RateSettings.TreeTickRate);
        naviCommandRateClock.reset(config.RateSettings.NaviCommandRate);

        // 关键参数防御式校验：避免配置错误把系统带入不可控状态。
        if (config.LeagueStrategySettings.GoalHoldSec <= 0) {
            LoggerPtr->Warning("Invalid LeagueStrategy.GoalHoldSec={}, fallback to 15.", config.LeagueStrategySettings.GoalHoldSec);
            config.LeagueStrategySettings.GoalHoldSec = 15;
        }
        if (config.LeagueStrategySettings.HealthRecoveryExitMin > 400) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryExitMin={}, clamp to 400.",
                config.LeagueStrategySettings.HealthRecoveryExitMin);
            config.LeagueStrategySettings.HealthRecoveryExitMin = 400;
        }
        if (config.LeagueStrategySettings.HealthRecoveryExitPreferred > 400) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryExitPreferred={}, clamp to 400.",
                config.LeagueStrategySettings.HealthRecoveryExitPreferred);
            config.LeagueStrategySettings.HealthRecoveryExitPreferred = 400;
        }
        if (config.LeagueStrategySettings.HealthRecoveryExitPreferred <
            config.LeagueStrategySettings.HealthRecoveryExitMin) {
            LoggerPtr->Warning(
                "LeagueStrategy.HealthRecoveryExitPreferred({}) < ExitMin({}), align preferred to min.",
                config.LeagueStrategySettings.HealthRecoveryExitPreferred,
                config.LeagueStrategySettings.HealthRecoveryExitMin);
            config.LeagueStrategySettings.HealthRecoveryExitPreferred =
                config.LeagueStrategySettings.HealthRecoveryExitMin;
        }
        if (config.LeagueStrategySettings.HealthRecoveryPlateauSec <= 0) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryPlateauSec={}, fallback to 2.",
                config.LeagueStrategySettings.HealthRecoveryPlateauSec);
            config.LeagueStrategySettings.HealthRecoveryPlateauSec = 2;
        }
        if (config.LeagueStrategySettings.HealthRecoveryExitStableSec <= 0) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryExitStableSec={}, fallback to 1.",
                config.LeagueStrategySettings.HealthRecoveryExitStableSec);
            config.LeagueStrategySettings.HealthRecoveryExitStableSec = 1;
        }
        if (config.LeagueStrategySettings.HealthRecoveryMaxHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryMaxHoldSec={}, fallback to 12.",
                config.LeagueStrategySettings.HealthRecoveryMaxHoldSec);
            config.LeagueStrategySettings.HealthRecoveryMaxHoldSec = 12;
        }
        if (config.LeagueStrategySettings.HealthRecoveryCooldownSec < 0) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.HealthRecoveryCooldownSec={}, fallback to 0.",
                config.LeagueStrategySettings.HealthRecoveryCooldownSec);
            config.LeagueStrategySettings.HealthRecoveryCooldownSec = 0;
        }
        if (config.LeagueStrategySettings.DamageOpenGateThreshold == 0) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.DamageOpenGateThreshold=0, fallback to 30.");
            config.LeagueStrategySettings.DamageOpenGateThreshold = 30;
        }
        if (config.LeagueStrategySettings.DamageOpenGateThreshold > 400) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.DamageOpenGateThreshold={}, clamp to 400.",
                config.LeagueStrategySettings.DamageOpenGateThreshold);
            config.LeagueStrategySettings.DamageOpenGateThreshold = 400;
        }
        if (!IsValidBaseGoal(config.LeagueStrategySettings.MainGoal)) {
            LoggerPtr->Warning(
                "Invalid LeagueStrategy.MainGoal={}, fallback to OccupyArea.",
                static_cast<int>(config.LeagueStrategySettings.MainGoal));
            config.LeagueStrategySettings.MainGoal = LangYa::OccupyArea.ID;
        }
        std::vector<std::uint8_t> sanitized_patrol_goals;
        sanitized_patrol_goals.reserve(config.LeagueStrategySettings.PatrolGoals.size());
        for (const auto goal_id : config.LeagueStrategySettings.PatrolGoals) {
            if (!IsValidBaseGoal(goal_id)) {
                LoggerPtr->Warning("Ignore invalid LeagueStrategy.PatrolGoals item={}.", static_cast<int>(goal_id));
                continue;
            }
            if (std::find(sanitized_patrol_goals.begin(), sanitized_patrol_goals.end(), goal_id) == sanitized_patrol_goals.end()) {
                sanitized_patrol_goals.push_back(goal_id);
            }
        }
        config.LeagueStrategySettings.PatrolGoals = std::move(sanitized_patrol_goals);

        if (config.ShowcasePatrolSettings.GoalHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid ShowcasePatrol.GoalHoldSec={}, fallback to 5.",
                config.ShowcasePatrolSettings.GoalHoldSec);
            config.ShowcasePatrolSettings.GoalHoldSec = 5;
        }
        std::vector<std::uint8_t> sanitized_showcase_goals;
        sanitized_showcase_goals.reserve(config.ShowcasePatrolSettings.Goals.size());
        for (const auto goal_id : config.ShowcasePatrolSettings.Goals) {
            if (!IsValidBaseGoal(goal_id)) {
                LoggerPtr->Warning("Ignore invalid ShowcasePatrol.Goals item={}.", static_cast<int>(goal_id));
                continue;
            }
            if (std::find(sanitized_showcase_goals.begin(), sanitized_showcase_goals.end(), goal_id) ==
                sanitized_showcase_goals.end()) {
                sanitized_showcase_goals.push_back(goal_id);
            }
        }
        config.ShowcasePatrolSettings.Goals = std::move(sanitized_showcase_goals);
        if (config.ShowcasePatrolSettings.Enable && config.ShowcasePatrolSettings.Goals.empty()) {
            LoggerPtr->Warning("ShowcasePatrol enabled but no valid goals found, fallback to OccupyArea.");
            config.ShowcasePatrolSettings.Goals.push_back(LangYa::OccupyArea.ID);
        }

        if (config.NaviDebugSettings.Enable) {
            if (!LoadNaviDebugPlanFile(config.NaviDebugSettings, LoggerPtr)) {
                LoggerPtr->Warning("Disable NaviDebug and fallback to legacy TestNavi route.");
                config.NaviDebugSettings.Enable = false;
            }
        }
        if (config.NaviDebugSettings.GoalHoldSec <= 0) {
            LoggerPtr->Warning(
                "Invalid NaviDebug.GoalHoldSec={}, fallback to 5.",
                config.NaviDebugSettings.GoalHoldSec);
            config.NaviDebugSettings.GoalHoldSec = 5;
        }
        std::vector<std::uint8_t> sanitized_navi_debug_goals;
        sanitized_navi_debug_goals.reserve(config.NaviDebugSettings.Goals.size());
        for (const auto goal_id : config.NaviDebugSettings.Goals) {
            if (!IsValidBaseGoal(goal_id)) {
                LoggerPtr->Warning("Ignore invalid NaviDebug.Goals item={}.", static_cast<int>(goal_id));
                continue;
            }
            if (std::find(sanitized_navi_debug_goals.begin(), sanitized_navi_debug_goals.end(), goal_id) ==
                sanitized_navi_debug_goals.end()) {
                sanitized_navi_debug_goals.push_back(goal_id);
            }
        }
        config.NaviDebugSettings.Goals = std::move(sanitized_navi_debug_goals);
        if (config.NaviDebugSettings.Enable && config.NaviDebugSettings.Goals.empty()) {
            LoggerPtr->Warning("NaviDebug enabled but no valid goals found, fallback to OccupyArea.");
            config.NaviDebugSettings.Goals.push_back(LangYa::OccupyArea.ID);
        }

        const std::string config_profile = NormalizeProfile(config.CompetitionProfile);
        const std::string effective_profile = competitionProfileOverride_.empty()
            ? config_profile
            : NormalizeProfile(competitionProfileOverride_);
        if (!competitionProfileOverride_.empty() && !config_profile.empty() &&
            effective_profile != config_profile) {
            LoggerPtr->Warning(
                "competition_profile override '{}' replaces config profile '{}'.",
                competitionProfileOverride_, config.CompetitionProfile);
        }
        // 最终生效 profile = launch override 优先，其次 JSON，最后 regional 默认值。
        const auto selected_profile = effective_profile.empty() ? std::string("regional") : effective_profile;
        config.CompetitionProfile = selected_profile;
        competitionProfile_ = ParseCompetitionProfile(config.CompetitionProfile);
        LoggerPtr->Info("Effective CompetitionProfile: {}", CompetitionProfileToString(competitionProfile_));
        if (competitionProfile_ == CompetitionProfile::League && config.NaviSettings.UseXY) {
            LoggerPtr->Warning("League profile is using UseXY=true. Goal-ID mode is recommended for league.");
        }
        if (config.ShowcasePatrolSettings.Enable && config.NaviSettings.UseXY) {
            LoggerPtr->Warning("ShowcasePatrol is enabled with UseXY=true. DisableTeamOffset only affects /ly/navi/goal.");
        }
        if (config.NaviDebugSettings.Enable && config.NaviSettings.UseXY) {
            LoggerPtr->Warning("NaviDebug is enabled with UseXY=true. Dedicated point-plan mode recommends /ly/navi/goal.");
        }

        // 与旧逻辑保持一致：SetPositionRepeat 的初始优先级
        // 联赛模式会直接进入 LeagueSimple。
        if (competitionProfile_ == CompetitionProfile::League) {
            strategyMode_ = StrategyMode::LeagueSimple;
        } else if (config.GameStrategySettings.HitSentry) {
            strategyMode_ = StrategyMode::HitSentry;
        } else if (config.GameStrategySettings.TestNavi) {
            strategyMode_ = StrategyMode::NaviTest;
        } else if (config.GameStrategySettings.Protected) {
            strategyMode_ = StrategyMode::Protected;
        } else {
            strategyMode_ = StrategyMode::HitHero;
        }
        LoggerPtr->Info("Initial StrategyMode: {}", StrategyModeToString(strategyMode_));

        postureManager_.Configure(config.PostureSettings);
        postureManager_.Reset(std::chrono::steady_clock::now(), SentryPosture::Move);
        leaguePatrolGoalIndex_ = 0;
        leaguePatrolGoalInitialized_ = false;
        showcasePatrolGoalIndex_ = 0;
        showcasePatrolGoalInitialized_ = false;
        naviDebugGoalIndex_ = 0;
        naviDebugGoalInitialized_ = false;
        leagueRecoveryActive_ = false;
        leagueRecoveryStartTime_ = std::chrono::steady_clock::time_point{};
        leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
        leagueRecoveryCooldownUntil_ = std::chrono::steady_clock::time_point{};
        leagueRecoveryEntryHealth_ = 0;
        leagueRecoveryPeakHealth_ = 0;

        return true;
    }
}
