// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"

#include <algorithm>

namespace BehaviorTree {

namespace {
struct PostureScore {
    int Attack{0};
    int Defense{0};
    int Move{0};
};

int GetScore(const PostureScore& score, const SentryPosture posture) {
    switch (posture) {
        case SentryPosture::Attack: return score.Attack;
        case SentryPosture::Defense: return score.Defense;
        case SentryPosture::Move: return score.Move;
        default: return -1000000;
    }
}

void AddScore(PostureScore& score, const SentryPosture posture, const int delta) {
    switch (posture) {
        case SentryPosture::Attack: score.Attack += delta; break;
        case SentryPosture::Defense: score.Defense += delta; break;
        case SentryPosture::Move: score.Move += delta; break;
        default: break;
    }
}
}  // namespace

bool Application::HasRecentTarget() const {
    if (lastTargetSeenTime.time_since_epoch().count() == 0) return false;
    const int keep_ms = std::max(0, config.PostureSettings.TargetKeepMs);
    return (std::chrono::steady_clock::now() - lastTargetSeenTime) <= std::chrono::milliseconds(keep_ms);
}

bool Application::IsUnderFireRecent() const {
    if (lastDamageTime.time_since_epoch().count() == 0) return false;
    const int keep_sec = std::max(0, config.PostureSettings.DamageKeepSec);
    return (std::chrono::steady_clock::now() - lastDamageTime) <= std::chrono::seconds(keep_sec);
}

bool Application::IsUnderFireBurst() const {
    const auto now = std::chrono::steady_clock::now();
    const int hold_sec = std::max(0, config.PostureSettings.DamageBurstDefenseHoldSec);
    if (hold_sec > 0 && lastDamageBurstTime_.time_since_epoch().count() != 0 &&
        (now - lastDamageBurstTime_) <= std::chrono::seconds(hold_sec)) {
        return true;
    }

    const int window_ms = std::max(0, config.PostureSettings.DamageBurstWindowMs);
    const int threshold = std::max(0, config.PostureSettings.DamageBurstThreshold);
    if (window_ms <= 0 || threshold <= 0) {
        return false;
    }

    std::uint32_t total_damage = 0;
    for (auto it = postureRecentDamageSamples_.rbegin(); it != postureRecentDamageSamples_.rend(); ++it) {
        if ((now - it->Time) > std::chrono::milliseconds(window_ms)) {
            break;
        }
        total_damage += it->Delta;
        if (total_damage >= static_cast<std::uint32_t>(threshold)) {
            return true;
        }
    }
    return false;
}

void Application::RecordDamageSample(const std::chrono::steady_clock::time_point now, const std::uint16_t damage) {
    if (damage == 0U) {
        return;
    }

    postureRecentDamageSamples_.push_back({now, damage});

    const int window_ms = std::max(0, config.PostureSettings.DamageBurstWindowMs);
    if (window_ms > 0) {
        const auto cutoff = now - std::chrono::milliseconds(window_ms);
        while (!postureRecentDamageSamples_.empty() && postureRecentDamageSamples_.front().Time < cutoff) {
            postureRecentDamageSamples_.pop_front();
        }
    } else if (postureRecentDamageSamples_.size() > 32U) {
        postureRecentDamageSamples_.pop_front();
    }

    const int threshold = std::max(0, config.PostureSettings.DamageBurstThreshold);
    if (window_ms <= 0 || threshold <= 0) {
        return;
    }

    std::uint32_t total_damage = 0;
    for (auto it = postureRecentDamageSamples_.rbegin(); it != postureRecentDamageSamples_.rend(); ++it) {
        if ((now - it->Time) > std::chrono::milliseconds(window_ms)) {
            break;
        }
        total_damage += it->Delta;
        if (total_damage >= static_cast<std::uint32_t>(threshold)) {
            lastDamageBurstTime_ = now;
            return;
        }
    }
}

SentryPosture Application::SelectDesiredPosture(const bool has_target) const {
    if (!config.PostureSettings.Enable) {
        return SentryPosture::Unknown;
    }

    const auto& runtime = postureManager_.Runtime();
    PostureScore score{};

    // 1) 基础策略加权
    switch (strategyMode_) {
        case StrategyMode::Protected:
            score.Defense += 6;
            score.Move += 1;
            break;
        case StrategyMode::NaviTest:
            score.Move += 8;
            break;
        case StrategyMode::HitSentry:
            score.Attack += 3;
            score.Move += 1;
            break;
        case StrategyMode::HitHero:
            score.Attack += 2;
            score.Move += 1;
            break;
        case StrategyMode::LeagueSimple:
            score.Attack += 1;
            score.Move += 3;
            break;
    }

    // 2) 瞄准模式加权
    if (aimMode == AimMode::RotateScan) {
        score.Move += 4;
    } else if (aimMode == AimMode::Buff || aimMode == AimMode::Outpost) {
        score.Attack += 3;
        score.Move += 1;
    } else {
        score.Attack += 2;
    }

    // 3) 目标可见性（带时间窗）
    if (has_target) {
        score.Attack += 6;
        score.Move -= 1;
    } else {
        score.Move += 1;
    }

    // 4) 资源与风险态势
    const bool low_energy =
        teamBuff.RemainingEnergy == 0b10000 ||
        teamBuff.RemainingEnergy == 0b00000;
    const bool very_low_health = myselfHealth <= static_cast<std::uint16_t>(config.PostureSettings.VeryLowHealthThreshold);
    const bool low_health = myselfHealth <= static_cast<std::uint16_t>(config.PostureSettings.LowHealthThreshold);
    const bool low_ammo = ammoLeft <= static_cast<std::uint16_t>(config.PostureSettings.LowAmmoThreshold);
    const bool under_fire = IsUnderFireRecent();
    const bool under_fire_burst = IsUnderFireBurst();

    if (under_fire_burst) {
        return SentryPosture::Defense;
    }

    if (low_energy) {
        score.Defense += 6;
        score.Move += 3;
        score.Attack -= 4;
    }
    if (very_low_health) {
        score.Defense += 7;
        score.Attack -= 4;
    } else if (low_health) {
        score.Defense += 4;
        score.Attack -= 2;
    }
    if (low_ammo) {
        score.Defense += 2;
        score.Move += 2;
        score.Attack -= 2;
    }
    if (under_fire) {
        score.Defense += 5;
        score.Move += 1;
        score.Attack -= 2;
    }

    // 5) 回读过期时降低激进性，优先保守/机动
    if (runtime.FeedbackStale) {
        score.Defense += 3;
        score.Move += 3;
        score.Attack -= 3;
    }

    // 6) 单姿态累计过久时，惩罚当前姿态，降低 3 分钟惩罚风险
    if (IsValidPosture(runtime.Current)) {
        const auto idx = ToPostureValue(runtime.Current);
        if (idx > 0U) {
            if (runtime.Degraded[idx]) {
                AddScore(score, runtime.Current, -3);
            } else if (runtime.AccumSec[idx] >= static_cast<double>(config.PostureSettings.EarlyRotateSec)) {
                AddScore(score, runtime.Current, -2);
            }
        }
    }

    // 7) pending 期间轻微偏向 pending，减少频繁改口
    if (runtime.HasPending && IsValidPosture(runtime.Pending)) {
        AddScore(score, runtime.Pending, 2);
    }

    SentryPosture best = SentryPosture::Attack;
    int best_score = score.Attack;
    if (score.Defense > best_score) {
        best = SentryPosture::Defense;
        best_score = score.Defense;
    }
    if (score.Move > best_score) {
        best = SentryPosture::Move;
        best_score = score.Move;
    }

    // 8) 分差迟滞：分差不够大时保持当前姿态，避免抖动
    const int hysteresis = std::max(0, config.PostureSettings.ScoreHysteresis);
    if (IsValidPosture(runtime.Current)) {
        const int current_score = GetScore(score, runtime.Current);
        if (best != runtime.Current && (best_score - current_score) <= hysteresis) {
            best = runtime.Current;
        }
    }

    return best;
}

void Application::UpdatePostureCommand(const bool has_target) {
    postureCommand = 0;
    if (!config.PostureSettings.Enable) return;

    const auto now = std::chrono::steady_clock::now();
    if (!postureHealthInitialized_) {
        postureLastHealth_ = myselfHealth;
        postureHealthInitialized_ = true;
    } else if (myselfHealth < postureLastHealth_) {
        RecordDamageSample(now, static_cast<std::uint16_t>(postureLastHealth_ - myselfHealth));
        lastDamageTime = now;
        postureLastHealth_ = myselfHealth;
    } else {
        postureLastHealth_ = myselfHealth;
    }

    const bool has_target_recent = has_target || HasRecentTarget();
    const auto desired = SelectDesiredPosture(has_target_recent);
    const auto decision = postureManager_.Tick(now, desired, postureState);
    postureCommand = decision.Command;
    const auto& runtime = postureManager_.Runtime();

    const bool desired_changed = desired != postureLastDesired_;
    const bool reason_changed = postureLastReason_ != decision.Reason;
    postureLastDesired_ = desired;
    postureLastReason_ = decision.Reason;

    if (LoggerPtr && (decision.Sent || desired_changed || reason_changed)) {
        LoggerPtr->Info(
            "[Posture] cmd={} desired={} current={} pending={} has_target_recent={} under_fire={} under_fire_burst={} feedback_stale={} reason={}",
            static_cast<int>(postureCommand),
            PostureToString(desired),
            PostureToString(runtime.Current),
            PostureToString(runtime.Pending),
            has_target_recent ? 1 : 0,
            IsUnderFireRecent() ? 1 : 0,
            IsUnderFireBurst() ? 1 : 0,
            runtime.FeedbackStale ? 1 : 0,
            decision.Reason);
    }
}

}  // namespace BehaviorTree
