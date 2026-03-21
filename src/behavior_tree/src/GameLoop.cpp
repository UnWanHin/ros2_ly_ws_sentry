#include "../include/Application.hpp"

#include <algorithm>
#include <cmath>

using namespace LangYa;

namespace BehaviorTree {


    float normalize_angle_0_360(float angle) {
        float normalized = fmod(angle, 360.0f);
        if (normalized < 0)
            normalized += 360.0f;
        return normalized;
    }

    namespace {
    constexpr std::uint8_t kMaxBaseGoalId = LangYa::OccupyArea.ID;
    // 丢 1~2 帧时保留锁角，避免抖动；时间过长会让云台“粘住旧目标”。
    constexpr auto kLostTargetHold = std::chrono::milliseconds(200);

    bool IsValidBaseGoalId(const std::uint8_t goal_id) {
        return goal_id <= kMaxBaseGoalId;
    }
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
        static constexpr auto delta_yaw = 1.0f;
        static constexpr auto buff_yaw = -50.0f + 360.0f;

        
        // 小陀螺策略：
        // 1) 平时低速巡航（当前实现：1）
        // 2) 检测到掉血后，2 秒内走变速序列，提升存活率
        // 3) 通讯协议没有方向位，用 A/B 两套节奏交替模拟“变向”
        const auto rotate_now = std::chrono::steady_clock::now();
        static auto last_damage_rotate_time = std::chrono::steady_clock::time_point{};
        static bool rotate_pattern_flip = false;
        constexpr int kDamageRotateWindowMs = 2000;
        constexpr int kRotatePhaseMs = 180;
        constexpr std::uint8_t kRotatePatternA[6] = {3, 2, 3, 1, 3, 0};
        constexpr std::uint8_t kRotatePatternB[6] = {3, 1, 3, 2, 3, 0};

        if (healthDecreaseDetector.trigger(myselfHealth)) { // 血量减少
            last_damage_rotate_time = rotate_now;
            rotate_pattern_flip = !rotate_pattern_flip;
            rotateTimerClock.tick();
        }

        bool in_damage_rotate_window = false;
        if (last_damage_rotate_time.time_since_epoch().count() != 0) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                rotate_now - last_damage_rotate_time).count();
            in_damage_rotate_window = elapsed_ms <= kDamageRotateWindowMs;
            if (in_damage_rotate_window) {
                const auto phase = static_cast<int>((elapsed_ms / kRotatePhaseMs) % 6);
                const auto* pattern = rotate_pattern_flip ? kRotatePatternA : kRotatePatternB;
                gimbalControlData.FireCode.Rotate = pattern[phase];
            } else {
                gimbalControlData.FireCode.Rotate = 1;
            }
        } else {
            gimbalControlData.FireCode.Rotate = 0;
        }
        if (config.AimDebugSettings.StopRotate) gimbalControlData.FireCode.Rotate = 0;

        static auto last_rotate_log = std::chrono::steady_clock::time_point{};
        if (now_time >= 0) {
            const auto log_now = std::chrono::steady_clock::now();
            if (log_now - last_rotate_log > std::chrono::seconds(2)) {
                LoggerPtr->Debug(
                    "Rotate Speed: {} (damage_window={} pattern_flip={})",
                    gimbalControlData.FireCode.Rotate,
                    in_damage_rotate_window ? 1 : 0,
                    rotate_pattern_flip ? 1 : 0);
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
        const bool find_target = isFindTargetAtomic.load(std::memory_order_relaxed);
        if (find_target) {
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
                    if (now - last_searching_log > std::chrono::seconds(2)) {
                        LoggerPtr->Debug("Searching Target...");
                        last_searching_log = now;
                        gimbalControlData.FireCode.AimMode = 0;
                    }
                    const auto current_time = std::chrono::steady_clock::now();
                    nextAngles = GimbalAnglesType{
                        static_cast<AngleType>(gimbalAngles.Yaw + 4 * delta_yaw),
                        AngleType{-0.0f + pitch_wave.Produce(current_time) * 3.0f}
                    };

                    if (aimMode == AimMode::Outpost) {
                        nextAngles.Pitch += 15.0f;
                    }
                } else if (activeAimData->HasLatchedAngles) {
                    nextAngles = activeAimData->Angles;
                    LoggerPtr->Debug(
                        "Reuse latched aim angles -> Pitch: {}, Yaw: {}",
                        nextAngles.Pitch,
                        nextAngles.Yaw);
                } else {
                    nextAngles = gimbalAngles;
                    LoggerPtr->Debug(
                        "No latched aim angles yet -> Pitch: {}, Yaw: {}",
                        gimbalAngles.Pitch,
                        gimbalAngles.Yaw);
                }
            }else { // 打符模式
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
        // lower_head 只在未锁目标时生效，并且整对角一起切换，避免混用旧 yaw/new pitch。
        if(naviLowerHead && !find_target) {
            nextAngles = GimbalAnglesType{gimbalAngles.Yaw, -15.0f}; //-22.5 - 26.0
        }
        gimbalControlData.GimbalAngles = nextAngles;

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

            const auto now = std::chrono::steady_clock::now();
            if (now - lastTreeTickLogTime_ > std::chrono::seconds(2)) {
                LoggerPtr->Debug("BehaviorTree Root Tick...");
                lastTreeTickLogTime_ = now;
            }
            TreeTickGuarded();
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
        if(aimMode == AimMode::Buff) { // 打符，修改为默认值
            if(BehaviorTree::Area::BuffShoot.near(nowx, nowy, 100, MyTeam)) {
                targetArmor.Type = ArmorType::Hero;
            } else {
                SetAimTargetNormal();
            }
        }else if(aimMode == AimMode::Outpost) { // 打前哨站
            if(BehaviorTree::Area::OutpostShoot.near(nowx, nowy, 100, MyTeam)) {
                targetArmor.Type = ArmorType::Outpost;
            } else {
                SetAimTargetNormal();
            }
        }else { // 普通模式
            if(naviCommandGoal == LangYa::HoleRoad(EnemyTeam)) { // 英雄点位1
                if(BehaviorTree::Area::HoleRoad.near(nowx, nowy, 100, MyTeam)) {
                    targetArmor.Type = ArmorType::Hero;
                } else {
                    SetAimTargetNormal();
                }
            }
            LoggerPtr->Info("Target: {}", static_cast<int>(targetArmor.Type));
        }

    }

    void Application::SetAimTargetNormal() {
        if(hitableTargets.size() > 0) {
            bool hero_find = (std::find(hitableTargets.begin(), hitableTargets.end(), UnitType::Hero) != hitableTargets.end());
            bool sentry_find = (std::find(hitableTargets.begin(), hitableTargets.end(), UnitType::Sentry) != hitableTargets.end());
            bool engnieer_find = (std::find(hitableTargets.begin(), hitableTargets.end(), UnitType::Engineer) != hitableTargets.end());
            bool infantry1_find = (std::find(hitableTargets.begin(),  hitableTargets.end(), UnitType::Infantry1) != hitableTargets.end());
            bool infantry2_find = (std::find(hitableTargets.begin(),  hitableTargets.end(), UnitType::Infantry2) != hitableTargets.end());
            if (hero_find) {
                targetArmor.Type = ArmorType::Hero;
                targetArmor.Distance = enemyRobots[UnitType::Hero].distance_;
            }else if(infantry1_find || infantry2_find) {
                if(infantry1_find && !infantry2_find) {
                    targetArmor.Type = ArmorType::Infantry1;
                    targetArmor.Distance = enemyRobots[UnitType::Infantry1].distance_;
                }else if(!infantry1_find && infantry2_find) {
                    targetArmor.Type = ArmorType::Infantry2;
                    targetArmor.Distance = enemyRobots[UnitType::Infantry2].distance_;
                }else {
                    int delta_distance = enemyRobots[UnitType::Infantry1].distance_ - enemyRobots[UnitType::Infantry2].distance_;
                    int delta_health = enemyRobots[UnitType::Infantry1].currentHealth_ - enemyRobots[UnitType::Infantry2].currentHealth_;
                    if(std::fabs(delta_distance) > 1) {
                        if(enemyRobots[UnitType::Infantry1].distance_ < enemyRobots[UnitType::Infantry2].distance_) {
                            targetArmor.Type = ArmorType::Infantry1;
                            targetArmor.Distance = enemyRobots[UnitType::Infantry1].distance_;
                        }else {
                            targetArmor.Type = ArmorType::Infantry2;
                            targetArmor.Distance = enemyRobots[UnitType::Infantry2].distance_;
                        }
                    } else { 
                        if(delta_health < 0) {
                            targetArmor.Type = ArmorType::Infantry1;
                            targetArmor.Distance = enemyRobots[UnitType::Infantry1].distance_;
                        }else {
                            targetArmor.Type = ArmorType::Infantry2;
                            targetArmor.Distance = enemyRobots[UnitType::Infantry2].distance_;
                        }
                    }
                } 
            }else if(sentry_find){
                targetArmor.Type = ArmorType::Sentry;
                targetArmor.Distance = enemyRobots[UnitType::Sentry].distance_;
            }else if(engnieer_find) {
                targetArmor.Type = ArmorType::Engineer;
                targetArmor.Distance = enemyRobots[UnitType::Engineer].distance_;
            }
            // else {
            //     int min_health = 9999;
            //     UnitType min_health_unit;
            //     for(auto unit : hitableTargets) {
            //         if(enemyRobots[unit].currentHealth_ < min_health) {
            //             min_health = enemyRobots[unit].currentHealth_;
            //             min_health_unit = unit;
            //         }
            //     }
            //     if(min_health_unit == UnitType::Engineer) {
            //         targetArmor.Type = ArmorType::Engineer;
            //         targetArmor.Distance = enemyRobots[UnitType::Engineer].distance_;
            //     }else if(min_health_unit == UnitType::Infantry1) {
            //         targetArmor.Type = ArmorType::Infantry1;
            //         targetArmor.Distance = enemyRobots[UnitType::Infantry1].distance_;
            //     }else if(min_health_unit == UnitType::Infantry2) { 
            //         targetArmor.Type = ArmorType::Infantry2;
            //         targetArmor.Distance = enemyRobots[UnitType::Infantry2].distance_;
            //     }else if(min_health_unit == UnitType::Sentry) {
            //         targetArmor.Type = ArmorType::Sentry;
            //         targetArmor.Distance = enemyRobots[UnitType::Sentry].distance_;
            //     }else if(min_health_unit == UnitType::Hero) {
            //         targetArmor.Type = ArmorType::Hero;
            //         targetArmor.Distance = enemyRobots[UnitType::Hero].distance_;
            //     }
            // }
        }else {
            targetArmor.Type = ArmorType::Hero;
            targetArmor.Distance = 30;
        }
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

    void Application::SetPositionByBaseGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) {
        auto assign_position = [&](const auto& goal_location, const auto& area_location) {
            naviCommandGoal = apply_team_offset ? goal_location(goal_team) : goal_location.ID;
            naviGoalPosition = area_location(goal_team);
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
            SetPositionByBaseGoal(plan[leaguePatrolGoalIndex_], team);
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
            SetPositionByBaseGoal(plan[leaguePatrolGoalIndex_], team);
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
        SetPositionByBaseGoal(plan[leaguePatrolGoalIndex_], team);
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
            const std::uint16_t recovery_exit_min = league.HealthRecoveryExitMin;
            const std::uint16_t recovery_exit_preferred = league.HealthRecoveryExitPreferred;
            const auto recovery_exit_stable = std::chrono::seconds(league.HealthRecoveryExitStableSec);
            const auto recovery_max_hold = std::chrono::seconds(league.HealthRecoveryMaxHoldSec);
            const auto recovery_cooldown = std::chrono::seconds(league.HealthRecoveryCooldownSec);
            auto reset_league_recovery_state = [&]() {
                leagueRecoveryActive_ = false;
                leagueRecoveryStartTime_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
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
            const bool ammo_ready = is_referee_value_ready(hasReceivedAmmoLeft_, lastAmmoLeftRxTime);
            const bool health_low = league.UseHealthRecovery && health_ready &&
                myselfHealth < league.HealthRecoveryThreshold;
            const bool ammo_low = league.UseAmmoRecovery && ammo_ready &&
                ammoLeft <= league.AmmoRecoveryThreshold;
            const bool missing_health_input = league.UseHealthRecovery && !health_ready;
            const bool missing_ammo_input = league.UseAmmoRecovery && !ammo_ready;
            const bool health_recovery_cooldown_active =
                leagueRecoveryCooldownUntil_.time_since_epoch().count() != 0 &&
                now < leagueRecoveryCooldownUntil_;
            const bool effective_health_low = health_low && !health_recovery_cooldown_active;

            if ((missing_health_input || missing_ammo_input) &&
                (now - lastLeagueRecoveryGuardLogTime_ > std::chrono::seconds(2))) {
                LoggerPtr->Warning(
                    "League recovery guard: skip invalid referee inputs (health_ready={} ammo_ready={} stale_timeout_ms={})",
                    health_ready ? 1 : 0,
                    ammo_ready ? 1 : 0,
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
                }

                if (health_ready && myselfHealth >= recovery_exit_min) {
                    if (leagueRecoveryReach350Time_.time_since_epoch().count() == 0) {
                        leagueRecoveryReach350Time_ = now;
                    }
                } else {
                    leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
                }

                const bool reach_preferred =
                    health_ready && myselfHealth >= recovery_exit_preferred;
                const bool reach_min_stable =
                    health_ready &&
                    myselfHealth >= recovery_exit_min &&
                    leagueRecoveryReach350Time_.time_since_epoch().count() != 0 &&
                    (now - leagueRecoveryReach350Time_) >= recovery_exit_stable;
                const bool recovery_timeout =
                    leagueRecoveryStartTime_.time_since_epoch().count() != 0 &&
                    (now - leagueRecoveryStartTime_) >= recovery_max_hold;

                if (reach_preferred || reach_min_stable) {
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
                SetPositionByBaseGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            if (naviCommandGoal == recovery_goal_id &&
                (effective_health_low || ammo_low || missing_health_input || missing_ammo_input)) {
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            if (effective_health_low || (ammo_low && recoveryClock.trigger())) {
                SetPositionByBaseGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
                if (ammo_low) {
                    recoveryClock.tick();
                }
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
            SET_POSITION(BuffShoot, MyTeam); // 打符
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 2; //加速
        }else if (aimMode == AimMode::Outpost) {
            SET_POSITION(OutpostShoot, MyTeam);
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 1; // 正常
        }else { //普通模式
            Random random;
            
                int random_number = random.Get(0, 6);
                if (random_number == 0) SET_POSITION(CastleLeft, MyTeam);
                else if (random_number == 1) SET_POSITION(CastleRight1, MyTeam);
                else if (random_number == 2) SET_POSITION(CastleRight2, MyTeam);
                else SET_POSITION(BuffShoot, MyTeam);
                if(naviCommandGoal == BuffShoot(MyTeam)) naviCommandIntervalClock.reset(Seconds{30});
                else naviCommandIntervalClock.reset(Seconds(10));
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
            SET_POSITION(BuffShoot, MyTeam); // 打符
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 2;
        }else if (aimMode == AimMode::Outpost) {
            SET_POSITION(OutpostShoot, MyTeam); // 打前哨站
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 1;
        }else { //普通模式
            Random random;

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
                        int random_number = random.Get(0, 8);
                        if(random_number == 0) SET_POSITION(MidShoot, MyTeam);
                        else if(random_number == 1) SET_POSITION(BuffAround1, MyTeam);
                        else if(random_number == 2) SET_POSITION(BuffAround2, MyTeam);
                        else if(random_number == 3) SET_POSITION(RightShoot, MyTeam);
                        else if(random_number == 4) SET_POSITION(MidShoot, EnemyTeam);
                        else if(random_number == 5) SET_POSITION(BuffAround1, EnemyTeam);
                        else if(random_number == 6) SET_POSITION(BuffAround2, EnemyTeam);
                        else if(random_number == 7) SET_POSITION(RightShoot, EnemyTeam);
                        else if(random_number == 8) SET_POSITION(LeftShoot, EnemyTeam);
                        if(naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam))
                            naviCommandIntervalClock.reset(Seconds(8));
                        else naviCommandIntervalClock.reset(Seconds(10));
                    }else {
                        SET_POSITION(FlyRoad, EnemyTeam);
                        naviCommandIntervalClock.reset(Seconds(10));
                    }
                    
             // 底盘能量低于5%
            if(teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
                LoggerPtr->Info("!!! Low Energy !!!");
                SET_POSITION(HoleRoad, MyTeam);
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
            SET_POSITION(BuffShoot, MyTeam);
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 2;
        }else if (aimMode == AimMode::Outpost) {
            SET_POSITION(OutpostShoot, MyTeam);
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 1;
        }else { //普通模式
            Random random;
            // 判断英雄是否处于高地
            bool hero_in_central = false;
            std::int16_t hero_x = enemyRobots[UnitType::Hero].position_.X, hero_y = enemyRobots[UnitType::Hero].position_.Y;
            hero_in_central = Area::CentralHighLandRed.isPointInside(hero_x, hero_y) || Area::CentralHighLandBlue.isPointInside(hero_x, hero_y);

            if(hero_in_central) {
                LoggerPtr->Debug("!!!Hero in highland!!!");
                SET_POSITION(BuffAround1, MyTeam);
            }else {
                if(selfOutpostHealth > 200) {
                    int redpx = 982, redpy = 1124;
                    int dx = redpx - enemyRobots[UnitType::Hero].position_.X, dy = redpy - enemyRobots[UnitType::Hero].position_.Y;
                    int len = std::sqrt(dx * dx + dy * dy);
                    if(len < 100) {
                        SET_POSITION(HoleRoad, EnemyTeam);
                        naviCommandIntervalClock.reset(Seconds(2));
                    }else {
                        int random_number = random.Get(0, 8);
                        if(random_number == 0) SET_POSITION(MidShoot, MyTeam);
                        else if(random_number == 1) SET_POSITION(BuffAround1, MyTeam);
                        else if(random_number == 2) SET_POSITION(BuffAround2, MyTeam);
                        else if(random_number == 3) SET_POSITION(RightShoot, MyTeam);
                        else if(random_number == 4) SET_POSITION(MidShoot, EnemyTeam);
                        else if(random_number == 5) SET_POSITION(BuffAround1, EnemyTeam);
                        else if(random_number == 6) SET_POSITION(BuffAround2, EnemyTeam);
                        else if(random_number == 7) SET_POSITION(RightShoot, EnemyTeam);
                        else if(random_number == 8) SET_POSITION(LeftShoot, EnemyTeam);
                        if(naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam))
                            naviCommandIntervalClock.reset(Seconds(8));
                        else naviCommandIntervalClock.reset(Seconds(10));
                    }
                }else {
                    if(selfBaseHealth > 2000){
                        SET_POSITION(HoleRoad, EnemyTeam);
                        naviCommandIntervalClock.reset(Seconds(2));
                    }
                    else {
                        int random_number = random.Get(0, 8);
                        if(random_number == 0) SET_POSITION(MidShoot, MyTeam);
                        else if(random_number == 1) SET_POSITION(BuffAround1, MyTeam);
                        else if(random_number == 2) SET_POSITION(BuffAround2, MyTeam);
                        else if(random_number == 3) SET_POSITION(RightShoot, MyTeam);
                        else if(random_number == 4) SET_POSITION(MidShoot, EnemyTeam);
                        else if(random_number == 5) SET_POSITION(BuffAround1, EnemyTeam);
                        else if(random_number == 6) SET_POSITION(BuffAround2, EnemyTeam);
                        else if(random_number == 7) SET_POSITION(RightShoot, EnemyTeam);
                        else if(random_number == 8) SET_POSITION(LeftShoot, EnemyTeam);
                        if(naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam))
                            naviCommandIntervalClock.reset(Seconds(8));
                        else naviCommandIntervalClock.reset(Seconds(10));
                    }
                }
            }
            
            // 底盘能量低于5%
            if(teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
                LoggerPtr->Info("!!! Low Energy !!!");
                int random_number = random.Get(0, 2);
                if(random_number == 0) SET_POSITION(BuffAround1, MyTeam);
                else if(random_number == 1) SET_POSITION(BuffAround2, MyTeam);
                else if(random_number == 2) SET_POSITION(RightShoot, MyTeam);
                naviCommandIntervalClock.reset(Seconds(10));
                speedLevel = 0;
            } // else speedLevel = 1;
            speedLevel = 1;
            LoggerPtr->Info("> NaviCommandGoal: {}", static_cast<int>(naviCommandGoal));
        }
    }



}
