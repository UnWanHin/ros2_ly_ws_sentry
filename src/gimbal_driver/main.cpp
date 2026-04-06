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
#include <bitset>
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
        std::chrono::steady_clock::time_point postureNextSendTime_{
            std::chrono::steady_clock::time_point::min()
        };

        static bool IsValidPosture(std::uint8_t posture) noexcept {
            return posture >= 1 && posture <= 3;
        }

        struct DecodedGimbalYaw {
            std::int16_t YawVelRaw{0};
            std::int16_t YawAngleRaw{0};
        };

        static std::int16_t DecodeInt16LittleEndian(const std::uint8_t low, const std::uint8_t high) noexcept {
            return static_cast<std::int16_t>(
                static_cast<std::uint16_t>(low) |
                (static_cast<std::uint16_t>(high) << 8));
        }

        static DecodedGimbalYaw DecodeGimbalYawFromReserve32(const std::uint32_t& reserve_32_1) noexcept {
            // Reserve_32_1 byte layout (little-endian):
            // [0]=yaw_vel low8, [1]=yaw_vel high8, [2]=yaw_angle low8, [3]=yaw_angle high8
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(&reserve_32_1);
            return DecodedGimbalYaw{
                DecodeInt16LittleEndian(bytes[0], bytes[1]),
                DecodeInt16LittleEndian(bytes[2], bytes[3])};
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

        void PubExtendData(const ExtendData& data) {
            {
                using topic = ly_me_uwb_yaw;
                topic::Msg msg;
                msg.data = data.UWBAngleYaw;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                const auto decoded = DecodeGimbalYawFromReserve32(data.Reserve_32_1);
                using topic = ly_gimbal_gimbal_yaw;
                topic::Msg msg;
                constexpr float kScale = 0.01f;
                msg.yaw_vel = decoded.YawVelRaw;
                msg.yaw_angle = decoded.YawAngleRaw;
                msg.yaw_vel_deg_s = static_cast<float>(decoded.YawVelRaw) * kScale;
                msg.yaw_angle_deg = static_cast<float>(decoded.YawAngleRaw) * kScale;
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
                        static auto last_game_dump_time = std::chrono::steady_clock::time_point{};
                        const auto now = std::chrono::steady_clock::now();
                        if (now - last_game_dump_time > std::chrono::milliseconds(200)) {
                            last_game_dump_time = now;
                            const auto self_health_swapped = static_cast<std::uint16_t>(
                                (game_data.SelfHealth >> 8) | (game_data.SelfHealth << 8));
                            const auto self_health_publish = game_data.SelfHealth;
                            roslog::info(
                                "GameData selfhealth parse: raw_field=%u publish=%u swapped_candidate=%u",
                                static_cast<unsigned int>(game_data.SelfHealth),
                                static_cast<unsigned int>(self_health_publish),
                                static_cast<unsigned int>(self_health_swapped));
                            roslog::info(
                                "RX GameData raw[12]=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | "
                                "parsed gamecode=0x%04X ammo=%u time=%u self=%u self_swapped=%u ext=0x%08X",
                                static_cast<unsigned int>(m.Data[0]), static_cast<unsigned int>(m.Data[1]),
                                static_cast<unsigned int>(m.Data[2]), static_cast<unsigned int>(m.Data[3]),
                                static_cast<unsigned int>(m.Data[4]), static_cast<unsigned int>(m.Data[5]),
                                static_cast<unsigned int>(m.Data[6]), static_cast<unsigned int>(m.Data[7]),
                                static_cast<unsigned int>(m.Data[8]), static_cast<unsigned int>(m.Data[9]),
                                static_cast<unsigned int>(m.Data[10]), static_cast<unsigned int>(m.Data[11]),
                                static_cast<unsigned int>(*reinterpret_cast<const std::uint16_t*>(&game_data.GameCode)),
                                static_cast<unsigned int>(game_data.AmmoLeft),
                                static_cast<unsigned int>(game_data.TimeLeft),
                                static_cast<unsigned int>(game_data.SelfHealth),
                                static_cast<unsigned int>(self_health_swapped),
                                static_cast<unsigned int>(game_data.ExtEventData));
                        }
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
                        const auto decoded = DecodeGimbalYawFromReserve32(extend_data.Reserve_32_1);

                        static auto last_extend_dump_time = std::chrono::steady_clock::time_point{};
                        const auto now = std::chrono::steady_clock::now();
                        if (now - last_extend_dump_time > std::chrono::milliseconds(200)) {
                            last_extend_dump_time = now;
                            constexpr float kScale = 0.01f;
                            const auto reserve_32_1_b0 = static_cast<unsigned int>(m.Data[4]);
                            const auto reserve_32_1_b1 = static_cast<unsigned int>(m.Data[5]);
                            const auto reserve_32_1_b2 = static_cast<unsigned int>(m.Data[6]);
                            const auto reserve_32_1_b3 = static_cast<unsigned int>(m.Data[7]);
                            const auto reserve_32_1_u32 = static_cast<std::uint32_t>(extend_data.Reserve_32_1);
                            const auto reserve_32_1_low16 = static_cast<std::uint16_t>(reserve_32_1_u32 & 0xFFFFu);
                            const auto reserve_32_1_high16 = static_cast<std::uint16_t>((reserve_32_1_u32 >> 16) & 0xFFFFu);
                            const auto reserve_32_1_bin = std::bitset<32>(reserve_32_1_u32).to_string();
                            const auto reserve_32_1_bin_grouped =
                                reserve_32_1_bin.substr(0, 8) + " " +
                                reserve_32_1_bin.substr(8, 8) + " " +
                                reserve_32_1_bin.substr(16, 8) + " " +
                                reserve_32_1_bin.substr(24, 8);
                            const auto reserve_32_1_low16_bin = std::bitset<16>(reserve_32_1_low16).to_string();
                            const auto reserve_32_1_high16_bin = std::bitset<16>(reserve_32_1_high16).to_string();
                            const auto posture_high8 =
                                static_cast<unsigned int>((extend_data.Reserve_16 >> 8) & 0xFFu);
                            roslog::info(
                                "RX ExtendData raw[12]=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                                static_cast<unsigned int>(m.Data[0]), static_cast<unsigned int>(m.Data[1]),
                                static_cast<unsigned int>(m.Data[2]), static_cast<unsigned int>(m.Data[3]),
                                static_cast<unsigned int>(m.Data[4]), static_cast<unsigned int>(m.Data[5]),
                                static_cast<unsigned int>(m.Data[6]), static_cast<unsigned int>(m.Data[7]),
                                static_cast<unsigned int>(m.Data[8]), static_cast<unsigned int>(m.Data[9]),
                                static_cast<unsigned int>(m.Data[10]), static_cast<unsigned int>(m.Data[11]));
                            roslog::info(
                                "ExtendData parse: uwb_yaw=%u reserve16=0x%04X posture_high8=%u reserve32_1=0x%08X "
                                "bytes[%02X %02X %02X %02X] reserve32_1_bin=%s "
                                "low16_bin=%s high16_bin=%s -> yaw_vel_raw=%d yaw_angle_raw=%d "
                                "yaw_vel_deg_s=%.2f yaw_angle_deg=%.2f reserve32_2=0x%08X",
                                static_cast<unsigned int>(extend_data.UWBAngleYaw),
                                static_cast<unsigned int>(extend_data.Reserve_16),
                                posture_high8,
                                static_cast<unsigned int>(extend_data.Reserve_32_1),
                                reserve_32_1_b0, reserve_32_1_b1, reserve_32_1_b2, reserve_32_1_b3,
                                reserve_32_1_bin_grouped.c_str(),
                                reserve_32_1_low16_bin.c_str(),
                                reserve_32_1_high16_bin.c_str(),
                                static_cast<int>(decoded.YawVelRaw),
                                static_cast<int>(decoded.YawAngleRaw),
                                static_cast<double>(decoded.YawVelRaw) * kScale,
                                static_cast<double>(decoded.YawAngleRaw) * kScale,
                                static_cast<unsigned int>(extend_data.Reserve_32_2));
                        }

                        PubExtendData(extend_data);
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
