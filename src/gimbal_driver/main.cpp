// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

/*
 * gimbal_driver 主节点
 *
 * 核心职责：
 * 1) 订阅 /ly/control/*（上位机控制指令）
 * 2) 与下位机串口设备交互（真实设备或虚拟设备）
 * 3) 发布 /ly/gimbal/* 与 /ly/game/*（回传状态）
 *
 * 备注：
 * - /ly/control/posture 为姿态指令输入，/ly/gimbal/posture 仅发布下位机回传状态。
 */
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <rclcpp/utilities.hpp>
#include <rclcpp/executors.hpp>

#include "gimbal_driver/msg/gimbal_angles.hpp"
#include "gimbal_driver/msg/gimbal_yaw.hpp"
#include "gimbal_driver/msg/uwb_pos.hpp"
#include "gimbal_driver/msg/vel.hpp"
#include "gimbal_driver/msg/health.hpp"
#include "gimbal_driver/msg/game_data.hpp"
#include "gimbal_driver/msg/buff_data.hpp"
#include "gimbal_driver/msg/position_data.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include "module/BasicTypes.hpp"
#include "module/IODevice.hpp"
#include "module/ROSTools.hpp"

using namespace LangYa;

namespace
{
    LY_DEF_ROS_TOPIC(ly_control_angles, "/ly/control/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_control_firecode, "/ly/control/firecode", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_control_vel, "/ly/control/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_control_posture, "/ly/control/posture", std_msgs::msg::UInt8);

    LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_gimbal_firecode, "/ly/gimbal/firecode", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_gimbal_vel, "/ly/gimbal/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_gimbal_gimbal_yaw, "/ly/gimbal/gimbal_yaw", gimbal_driver::msg::GimbalYaw);
    LY_DEF_ROS_TOPIC(ly_gimbal_posture, "/ly/gimbal/posture", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_gimbal_capV, "/ly/gimbal/capV", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_game_eventdata, "ly/gimbal/eventdata", std_msgs::msg::UInt32);

    LY_DEF_ROS_TOPIC(ly_me_is_precaution, "/ly/me/is_precaution", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_me_is_at_home, "/ly/me/is_at_home", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_me_is_team_red, "/ly/me/is_team_red", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_me_hp, "/ly/me/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_me_op_hp, "/ly/me/op_hp", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_me_base_hp, "/ly/me/base_hp", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_me_ammo_left, "/ly/me/ammo_left", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_me_uwb_pos, "/ly/me/uwb_pos", std_msgs::msg::UInt16MultiArray);
    LY_DEF_ROS_TOPIC(ly_me_uwb_yaw, "/ly/me/uwb_yaw", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_game_is_start, "/ly/game/is_start", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_game_time_left, "/ly/game/time_left", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_enemy_hp, "/ly/enemy/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_enemy_op_hp, "/ly/enemy/op_hp", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_enemy_base_hp, "/ly/enemy/base_hp", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_game_all, "/ly/game/all", gimbal_driver::msg::GameData);
    LY_DEF_ROS_TOPIC(ly_bullet_speed, "/ly/bullet/speed", std_msgs::msg::Float32);

    LY_DEF_ROS_TOPIC(ly_team_buff, "/ly/team/buff", gimbal_driver::msg::BuffData);
    LY_DEF_ROS_TOPIC(ly_me_rfid, "/ly/me/rfid", std_msgs::msg::UInt32);
    LY_DEF_ROS_TOPIC(ly_position_data, "/ly/position/data", gimbal_driver::msg::PositionData);
        

    using namespace std::chrono_literals;
    class Application
    {
    public:
        inline static constexpr const char Name[] = "gimbal_driver";

    private:
        ROSNode<Name> Node;
        std::atomic_bool DeviceError{ false };
        IODevice<TypedMessage<sizeof(GimbalData)>, GimbalControlData> Device{};
        MultiCallback<GimbalControlData> CallbackGenerator;
        GimbalControlData controlShadow_{};
        std::uint8_t postureCommand_{0}; // 0=不控制, 1=进攻, 2=防御, 3=移动
        std::uint8_t postureState_{0};   // 0=未知, 1=进攻, 2=防御, 3=移动
        int postureTxRepeatCount_{3};
        std::chrono::milliseconds postureTxInterval_{20};
        std::uint8_t posturePendingToSend_{0};
        std::uint8_t postureLastSent_{0};
        int posturePendingRepeat_{0};
        bool hasReserve32_2AnglePrev_{false};
        std::uint16_t reserve32_2AnglePrevRaw_{0};
        std::chrono::steady_clock::time_point reserve32_2AnglePrevTime_{
            std::chrono::steady_clock::time_point::min()
        };
        bool hasDerivedYawVelFiltered_{false};
        float derivedYawVelFilteredDegPerSec_{0.0f};
        std::chrono::steady_clock::time_point postureNextSendTime_{
            std::chrono::steady_clock::time_point::min()
        };

        static bool IsValidPosture(std::uint8_t posture) noexcept {
            return posture >= 1 && posture <= 3;
        }

        static std::int16_t NormalizeAngleRawToSigned180(std::uint16_t angle_raw_unsigned) noexcept {
            // Protocol unit is 0.01 deg. Map [0, 36000) to [-18000, 18000).
            std::int32_t raw = static_cast<std::int32_t>(angle_raw_unsigned) % 36000;
            if (raw < 0) raw += 36000;
            if (raw >= 18000) raw -= 36000;
            return static_cast<std::int16_t>(raw);
        }

        static std::int32_t NormalizeDeltaRaw(std::int32_t delta_raw) noexcept {
            // Shortest-path unwrap in 0.01 deg domain.
            if (delta_raw > 18000) delta_raw -= 36000;
            if (delta_raw < -18000) delta_raw += 36000;
            return delta_raw;
        }

        static std::uint8_t DecodePostureFromReserve16(std::uint16_t reserve16) noexcept {
            // 姿态回读统一约定：Reserve_16 高 8 位。
            const auto high8 = static_cast<std::uint8_t>((reserve16 >> 8) & 0xFFu);
            return IsValidPosture(high8) ? high8 : 0u;
        }

        void ArmPostureTx(std::uint8_t posture) {
            if (!IsValidPosture(posture)) {
                return;
            }
            if (posture == postureLastSent_ && posturePendingRepeat_ == 0) {
                return;
            }
            posturePendingToSend_ = posture;
            posturePendingRepeat_ = std::max(1, postureTxRepeatCount_);
            postureNextSendTime_ = std::chrono::steady_clock::now();
            roslog::info("Posture TX armed: cmd=%u, repeat=%d",
                         posturePendingToSend_, posturePendingRepeat_);
        }

        void MaybeSendPostureTx() {
            if (posturePendingRepeat_ <= 0) {
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now < postureNextSendTime_) {
                return;
            }

            auto tx = controlShadow_;
            tx.Posture = posturePendingToSend_;
            if (!Device.Write(tx)) {
                DeviceError = true;
                return;
            }
            controlShadow_ = tx;

            posturePendingRepeat_--;
            postureNextSendTime_ = now + postureTxInterval_;
            if (posturePendingRepeat_ == 0) {
                postureLastSent_ = posturePendingToSend_;
            }
        }

        void PublishPosture(std::uint8_t posture) {
            using topic = ly_gimbal_posture;
            topic::Msg msg;
            msg.data = posture;
            Node.Publisher<topic>()->publish(msg);
        }

        template<typename TTopic>
        void GenSub(auto modifier)
        {
            Node.GenSubscriber<TTopic>(CallbackGenerator.Generate<TTopic>(modifier));
        }

        //TODO disable cooldown timer
        using clock_t = std::chrono::steady_clock;
           
        struct Timer {
            clock_t::duration interval;
            clock_t::time_point head;

            bool check(const clock_t::time_point now) {
                const auto delta_t = now - head;
                if (delta_t > interval) {
                    head = now;
                    return true;
                }
                return false;
            }
        };

        class StateTimer {
            Timer shoot_timer{};
            Timer cooldown_timer{};
            bool is_shoot{false};

            Timer& get_timer() {
                return is_shoot ? shoot_timer : cooldown_timer;
            }

        public:
            bool can_shoot() const noexcept {
                return is_shoot;
            }

            StateTimer(const clock_t::duration shoot, const clock_t::duration cooldown) {
                const auto now = clock_t::now();
                shoot_timer.head = now;
                shoot_timer.interval = shoot;
                cooldown_timer.head = now;
                cooldown_timer.interval = cooldown;
            }

            bool check() {
                const auto now = clock_t::now();
                if (get_timer().check(now)) {
                    is_shoot = !is_shoot;
                    get_timer().head = now;
                }
                
                return can_shoot();
            }

        };

        StateTimer state_timer{std::chrono::milliseconds(100), std::chrono::milliseconds(2000)}; //簡學長寫的timer，使用的話則關掉下面的false &&
        //TODO 

        void GenSubs()
        {
            GenSub<ly_control_angles>([](GimbalControlData& g, const gimbal_driver::msg::GimbalAngles& m)
                                        {
                                            g.GimbalAngles.Yaw = static_cast<float>(m.yaw);
                                            g.GimbalAngles.Pitch = static_cast<float>(m.pitch);  
                                        });

            GenSub<ly_control_firecode>([this](GimbalControlData& g, const std_msgs::msg::UInt8& m)
                                        {
                                            *reinterpret_cast<std::uint8_t*>(&g.FireCode) = m.data;
                                            if (false && !state_timer.check()) {  
                                                g.FireCode.FireStatus = 0;
                                            }
                                        });

            GenSub<ly_control_vel>([](GimbalControlData& g, const gimbal_driver::msg::Vel& m)
                                   {
                                       g.Velocity.X = static_cast<int8_t>(m.x);
                                       g.Velocity.Y = static_cast<int8_t>(m.y);
                                   });

            GenSub<ly_control_posture>([this](GimbalControlData& g, const std_msgs::msg::UInt8& m)
                                       {
                                           const auto cmd = m.data;
                                           if (cmd != 0 && !IsValidPosture(cmd)) {
                                               roslog::warn("Invalid /ly/control/posture: %u (expect 0/1/2/3)", cmd);
                                               return;
                                           }
                                           postureCommand_ = cmd;
                                           g.Posture = cmd;
                                           // /ly/gimbal/posture 仅由下位机回传驱动，避免命令回环掩盖真实执行状态。
                                           if (cmd == 0) {
                                               posturePendingRepeat_ = 0;
                                               posturePendingToSend_ = 0;
                                               return;
                                           }
                                           ArmPostureTx(cmd);
                                       });
        }

        void  PubGimbalData(const GimbalData& data)
        {
            {
                using topic = ly_gimbal_angles;
                topic::Msg msg;
                msg.yaw = static_cast<float>(data.GimbalAngles.Yaw);
                msg.pitch = static_cast<float>(data.GimbalAngles.Pitch);
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_firecode;
                topic::Msg msg;
                msg.data = *reinterpret_cast<const std::uint8_t*>(&data.FireCode);
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_vel;
                topic::Msg msg;
                msg.x = static_cast<int8_t>(data.Velocity.X);
                msg.y = static_cast<int8_t>(data.Velocity.Y);
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_capV;
                topic::Msg msg;
                msg.data = static_cast<std::uint8_t>(data.CapV);
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubGameData(const GameData& data)
        {
            {
                using topic = ly_game_all;
                topic::Msg msg;
                msg.gamecode = *reinterpret_cast<const std::uint16_t*>(&data.GameCode);
                msg.ammoleft = data.AmmoLeft;
                msg.timeleft = data.TimeLeft;
                msg.selfhealth = data.SelfHealth;
                msg.exteventdata = *static_cast<const std::uint32_t*>(&data.ExtEventData);
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_me_ammo_left;
                topic::Msg msg;
                msg.data = data.AmmoLeft;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_enemy_op_hp;
                topic::Msg msg;
                msg.data = data.GameCode.EnemyOutpostHealth * 25;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_me_is_precaution;
                topic::Msg msg;
                msg.data = data.GameCode.HeroPrecaution;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_is_start;
                topic::Msg msg;
                msg.data = data.GameCode.IsGameBegin;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_me_is_team_red;
                topic::Msg msg;
                msg.data = data.GameCode.IsMyTeamRed;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_me_is_at_home;
                topic::Msg msg;
                msg.data = data.GameCode.IsReturnedHome;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_me_op_hp;
                topic::Msg msg;
                msg.data = data.GameCode.SelfOutpostHealth * 25;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_time_left;
                topic::Msg msg;
                msg.data = data.TimeLeft;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_eventdata;
                topic::Msg msg;
                msg.data = static_cast<std::uint32_t>(data.ExtEventData);
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubRFIDAndBuffData(const RFIDAndBuffData& data){
            {
                using topic = ly_me_rfid;
                topic::Msg msg;
                msg.data = data.RFIDStatus;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_team_buff;
                topic::Msg msg;
                msg.recoverybuff = data.BuffStatus.RecoveryBuff;
                msg.coolingbuff = data.BuffStatus.CoolingBuff;
                msg.defencebuff = data.BuffStatus.DefenceBuff;
                msg.vulnerabilitybuff = data.BuffStatus.VulnerabilityBuff;
                msg.attackbuff = data.BuffStatus.AttackBuff;
                msg.remainingenergy = data.BuffStatus.RemainingEnergy;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubPositionData(const PositionData& data){
            {
                {
                    using topic = ly_position_data;
                    topic::Msg msg;
                    msg.friendcarid = data.Friend.CarId;
                    msg.friendx = data.Friend.X;
                    msg.friendy = data.Friend.Y;
                    msg.enemycarid = data.Enemy.CarId;
                    msg.enemyx = data.Enemy.X;
                    msg.enemyy = data.Enemy.Y;
                    Node.Publisher<topic>()->publish(msg);
                }
                if(data.Friend.CarId == 7) {
                    using topic = ly_me_uwb_pos;
                    std::vector<std::uint16_t> pos = {
                        static_cast<std::uint16_t>(data.Friend.X),
                        static_cast<std::uint16_t>(data.Friend.Y)
                    };
                    topic::Msg msg;
                    msg.data = pos;
                    Node.Publisher<topic>()->publish(msg);
                }
            }
            {
                using topic = ly_bullet_speed;
                topic::Msg msg;
                msg.data = static_cast<float>(data.BulletSpeed) / 100.0f;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubHealthMyselfData(const HealthMyselfData& data){
            {
                using topic = ly_me_hp;
                topic::Msg msg;
                msg.hero = data.HeroMyself;
                msg.engineer = data.EngineerMyself;
                msg.infantry1 = data.Infantry1Myself;
                msg.infantry2 = data.Infantry2Myself;
                msg.reserve = data.BaseMyself;
                msg.sentry = data.SentryMyself;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_me_base_hp;
                topic::Msg msg;
                msg.data = data.BaseMyself;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubHealthEnemyData(const HealthEnemyData& data){
            {
                using topic = ly_enemy_hp;
                topic::Msg msg;
                msg.hero = data.HeroEnemy;
                msg.engineer = data.EngineerEnemy;
                msg.infantry1 = data.Infantry1Enemy;
                msg.infantry2 = data.Infantry2Enemy;
                msg.reserve = data.BaseEnemy;
                msg.sentry = data.SentryEnemy;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_enemy_base_hp;
                topic::Msg msg;
                msg.data = data.BaseEnemy;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubExtendData(const ExtendData& data, std::int16_t yaw_vel_raw, std::int16_t yaw_angle_raw) {
            {
                using topic = ly_me_uwb_yaw;
                topic::Msg msg;
                msg.data = data.UWBAngleYaw;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_gimbal_yaw;
                topic::Msg msg;
                constexpr float kScale = 0.01f;
                msg.yaw_vel = yaw_vel_raw;
                msg.yaw_angle = yaw_angle_raw;
                msg.yaw_vel_deg_s = static_cast<float>(yaw_vel_raw) * kScale;
                msg.yaw_angle_deg = static_cast<float>(yaw_angle_raw) * kScale;
                Node.Publisher<topic>()->publish(msg);
            }

            const auto posture_raw = DecodePostureFromReserve16(data.Reserve_16);
            if (IsValidPosture(posture_raw)) {
                postureState_ = posture_raw;
                PublishPosture(postureState_);
            }
        }

        void LoopRead()
        {
            Device.LoopRead(DeviceError, [this](const TypedMessage<sizeof(GimbalData)>& m)
            {
                switch (m.TypeID)
                {
                    case GimbalData::TypeID:
                        PubGimbalData(m.GetDataAs<GimbalData>());
                        break;

                    case GameData::TypeID:
                    {
                        const auto& game_data = m.GetDataAs<GameData>();
                        PubGameData(game_data);
                        break;
                    }

                    case HealthMyselfData::TypeID:
                        PubHealthMyselfData(m.GetDataAs<HealthMyselfData>());
                        break;

                    case HealthEnemyData::TypeID:
                        PubHealthEnemyData(m.GetDataAs<HealthEnemyData>());
                        break;

                    case RFIDAndBuffData::TypeID:
                        PubRFIDAndBuffData(m.GetDataAs<RFIDAndBuffData>());
                        break;

                    case PositionData::TypeID:
                        PubPositionData(m.GetDataAs<PositionData>());
                        break;
                    case ExtendData::TypeID:
                    {
                        const auto& extend_data = m.GetDataAs<ExtendData>();
                        constexpr float kScale = 0.01f;
                        constexpr float kVelFilterAlpha = 0.30f;
                        constexpr float kVelDecayWhenNoInstant = 0.70f;
                        constexpr float kVelDeadbandDegPerSec = 0.20f;
                        const auto reserve_32_1_u32 = static_cast<std::uint32_t>(extend_data.Reserve_32_1);
                        const auto reserve_32_1_high16_u =
                            static_cast<std::uint16_t>((reserve_32_1_u32 >> 16) & 0xFFFFu);
                        const auto reserve_32_1_high16_i = static_cast<std::int16_t>(reserve_32_1_high16_u);

                        const auto reserve_32_2_u32 = static_cast<std::uint32_t>(extend_data.Reserve_32_2);
                        const auto reserve_32_2_low16_u = static_cast<std::uint16_t>(reserve_32_2_u32 & 0xFFFFu);

                        const auto now = std::chrono::steady_clock::now();
                        const auto yaw_angle_raw_from_r32_2 = NormalizeAngleRawToSigned180(reserve_32_2_low16_u);
                        const auto yaw_angle_deg_from_r32_2 = static_cast<float>(yaw_angle_raw_from_r32_2) * kScale;

                        bool has_instant_vel = false;
                        std::int16_t yaw_vel_instant_raw = 0;
                        float yaw_vel_instant_deg_s = 0.0f;
                        if (hasReserve32_2AnglePrev_) {
                            const auto dt_sec =
                                std::chrono::duration<float>(now - reserve32_2AnglePrevTime_).count();
                            if (dt_sec > 1e-3f && dt_sec < 0.5f) {
                                auto delta_raw = static_cast<std::int32_t>(reserve_32_2_low16_u) -
                                                 static_cast<std::int32_t>(reserve32_2AnglePrevRaw_);
                                delta_raw = NormalizeDeltaRaw(delta_raw);
                                yaw_vel_instant_deg_s = static_cast<float>(delta_raw) * kScale / dt_sec;
                                const auto yaw_vel_instant_i32 = static_cast<std::int32_t>(
                                    std::lround(static_cast<double>(yaw_vel_instant_deg_s / kScale)));
                                yaw_vel_instant_raw = static_cast<std::int16_t>(
                                    std::clamp(yaw_vel_instant_i32, -32768, 32767));
                                has_instant_vel = true;
                            }
                        }
                        reserve32_2AnglePrevRaw_ = reserve_32_2_low16_u;
                        reserve32_2AnglePrevTime_ = now;
                        hasReserve32_2AnglePrev_ = true;

                        if (!hasDerivedYawVelFiltered_) {
                            derivedYawVelFilteredDegPerSec_ = has_instant_vel ? yaw_vel_instant_deg_s : 0.0f;
                            hasDerivedYawVelFiltered_ = true;
                        } else if (has_instant_vel) {
                            derivedYawVelFilteredDegPerSec_ +=
                                kVelFilterAlpha * (yaw_vel_instant_deg_s - derivedYawVelFilteredDegPerSec_);
                        } else {
                            derivedYawVelFilteredDegPerSec_ *= kVelDecayWhenNoInstant;
                        }

                        if (std::abs(derivedYawVelFilteredDegPerSec_) < kVelDeadbandDegPerSec) {
                            derivedYawVelFilteredDegPerSec_ = 0.0f;
                        }

                        const auto yaw_vel_publish_i32 = static_cast<std::int32_t>(
                            std::lround(static_cast<double>(derivedYawVelFilteredDegPerSec_ / kScale)));
                        const auto yaw_vel_publish_raw = static_cast<std::int16_t>(
                            std::clamp(yaw_vel_publish_i32, -32768, 32767));
                        const auto yaw_vel_publish_deg_s = static_cast<float>(yaw_vel_publish_raw) * kScale;
                        const auto yaw_vel_r32_1_high_deg_s =
                            static_cast<float>(reserve_32_1_high16_i) * kScale;

                        static auto last_extend_dump_time = std::chrono::steady_clock::time_point{};
                        if (now - last_extend_dump_time > std::chrono::milliseconds(50)) {
                            last_extend_dump_time = now;
                            const auto compare_raw =
                                static_cast<std::int32_t>(reserve_32_1_high16_i) - static_cast<std::int32_t>(yaw_vel_publish_raw);
                            roslog::info(
                                "YawCompare: angle_raw(r32_2_low16)=%d angle_deg=%.2f "
                                "vel_pub_raw=%d vel_pub_deg_s=%.2f "
                                "vel_inst_raw=%d vel_inst_deg_s=%.2f inst_valid=%s "
                                "vel_r32_1_high_raw=%d vel_r32_1_high_deg_s=%.2f "
                                "diff_raw(r32_1_high-pub)=%d uwb_yaw=%u",
                                static_cast<int>(yaw_angle_raw_from_r32_2),
                                static_cast<double>(yaw_angle_deg_from_r32_2),
                                static_cast<int>(yaw_vel_publish_raw),
                                static_cast<double>(yaw_vel_publish_deg_s),
                                static_cast<int>(yaw_vel_instant_raw),
                                static_cast<double>(yaw_vel_instant_deg_s),
                                has_instant_vel ? "true" : "false",
                                static_cast<int>(reserve_32_1_high16_i),
                                static_cast<double>(yaw_vel_r32_1_high_deg_s),
                                static_cast<int>(compare_raw),
                                static_cast<unsigned int>(extend_data.UWBAngleYaw));
                        }

                        PubExtendData(extend_data, yaw_vel_publish_raw, yaw_angle_raw_from_r32_2);
                        break;
                    }

                    default:
                        roslog::error("Application::LoopRead: invalid type id(%u)",
                                      static_cast<unsigned int>(m.TypeID));
                        break;
                }
            });
        }

        void TestVirtualLoopback(){
            TypedMessage<sizeof(GimbalData)> test_msg{};
            GimbalControlData test_msg2{};
            test_msg.TypeID = GimbalData::TypeID;
            test_msg.GetDataAs<GimbalData>().GimbalAngles.Yaw = 45.0f;
            test_msg2.GimbalAngles.Yaw = 30.0f;

            Device.Write(test_msg2);
            
            Device.LoopRead(DeviceError, [&](const TypedMessage<sizeof(GimbalData)>& received){
                if (memcmp(&test_msg, &received, sizeof(test_msg)) == 0) {
                    RCLCPP_INFO(rclcpp::get_logger("test"), "Virtual loopback test passed");
                }
            });
        }

    public:
        Application() noexcept : CallbackGenerator{
                [this](const auto& data)
                {
                    controlShadow_ = data;
                    if (DeviceError) return;
                    if (!Device.Write(data)) DeviceError = true;
                }
        }
        {
        }

        void Run(int argc, char** argv)
        {
            Node.Initialize(argc, argv);
            GenSubs();
            auto node = Node.GetNode();
            rclcpp::Rate rate(250);
            bool useVirtualDevice = false;
            std::string serialDeviceName{"/dev/ttyACM0"};
            int serialBaudRate = 115200;
            auto getParamCompat = [this](const char* slash_key, const char* dot_key, auto& value, const auto& default_value) {
                using T = std::decay_t<decltype(value)>;
                Node.GetParam<T>(slash_key, value, static_cast<T>(default_value));
                T dot_value = value;
                Node.GetParam<T>(dot_key, dot_value, value);
                value = dot_value;
            };
            getParamCompat("io_config/use_virtual_device", "io_config.use_virtual_device", useVirtualDevice, false);
            getParamCompat("io_config/device_name", "io_config.device_name", serialDeviceName, std::string{"/dev/ttyACM0"});
            getParamCompat("io_config/baud_rate", "io_config.baud_rate", serialBaudRate, 115200);
            int postureRepeatCount = postureTxRepeatCount_;
            int postureRepeatIntervalMs = static_cast<int>(postureTxInterval_.count());
            getParamCompat(
                "io_config/posture_repeat_count",
                "io_config.posture_repeat_count",
                postureRepeatCount,
                postureRepeatCount);
            getParamCompat(
                "io_config/posture_repeat_interval_ms",
                "io_config.posture_repeat_interval_ms",
                postureRepeatIntervalMs,
                postureRepeatIntervalMs);

            if (postureRepeatCount <= 0) {
                roslog::warn("Invalid posture_repeat_count=%d, fallback to 3", postureRepeatCount);
                postureRepeatCount = 3;
            }
            if (postureRepeatIntervalMs <= 0) {
                roslog::warn("Invalid posture_repeat_interval_ms=%d, fallback to 20",
                             postureRepeatIntervalMs);
                postureRepeatIntervalMs = 20;
            }

            postureTxRepeatCount_ = postureRepeatCount;
            postureTxInterval_ = std::chrono::milliseconds(postureRepeatIntervalMs);
            roslog::warn("posture_tx merged mode: repeat_count=%d repeat_interval_ms=%d",
                         postureTxRepeatCount_,
                         static_cast<int>(postureTxInterval_.count()));

            while (rclcpp::ok())
            {
                if (!DeviceError) DeviceError = true;
                std::this_thread::sleep_for(1s);
                if (!Device.Initialize(useVirtualDevice, serialDeviceName, serialBaudRate)) continue;
                DeviceError = false;
                posturePendingRepeat_ = 0;
                postureLastSent_ = 0;
                controlShadow_.Posture = IsValidPosture(postureCommand_) ? postureCommand_ : 0;
                if (IsValidPosture(postureCommand_)) {
                    ArmPostureTx(postureCommand_);
                }
                std::jthread reading{ [this, useVirtualDevice] { useVirtualDevice ? TestVirtualLoopback() : LoopRead(); } };
                while (!DeviceError) {
                    rclcpp::spin_some(node);
                    MaybeSendPostureTx();
                    rate.sleep();
                }
            }
        }
    };
}

int main(int argc, char** argv) try
{
    RCLCPP_INFO(rclcpp::get_logger("main"), "main: running %s", Application::Name);
    Application app{};
    app.Run(argc, argv);
    return 0;
}
catch (const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("main"), "main: %s", e.what());
    return 1;
}
