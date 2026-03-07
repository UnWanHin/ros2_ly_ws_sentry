#include <chrono>
#include <thread>
#include <algorithm>
#include <rclcpp/utilities.hpp>
#include <rclcpp/executors.hpp>

#include "gimbal_driver/msg/gimbal_angles.hpp"
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
        std::uint8_t postureCommand_{0}; // 0=不控制, 1=进攻, 2=防御, 3=移动
        std::uint8_t postureState_{0};   // 0=未知, 1=进攻, 2=防御, 3=移动
        std::uint8_t postureTxTypeID_{PostureControlTypeID};
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
            roslog::info("Posture TX armed: cmd=%u, repeat=%d, type_id=%u",
                         posturePendingToSend_, posturePendingRepeat_, postureTxTypeID_);
        }

        void MaybeSendPostureTx() {
            if (posturePendingRepeat_ <= 0) {
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now < postureNextSendTime_) {
                return;
            }

            PostureControlMessage frame{};
            frame.TypeID = postureTxTypeID_;
            frame.Data[0] = posturePendingToSend_;
            frame.Tail = 0;
            if (!Device.WriteRaw(frame)) {
                DeviceError = true;
                return;
            }

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

        void GenSubs()
        {
            GenSub<ly_control_angles>([](GimbalControlData& g, const gimbal_driver::msg::GimbalAngles& m)
                                        {
                                          g.GimbalAngles.Yaw = static_cast<float>(m.yaw);
                                          g.GimbalAngles.Pitch = static_cast<float>(m.pitch);
                                      });

            GenSub<ly_control_firecode>([](GimbalControlData& g, const std_msgs::msg::UInt8& m)
                                        {
                                            *reinterpret_cast<std::uint8_t*>(&g.FireCode) = m.data;
                                        });

            GenSub<ly_control_vel>([](GimbalControlData& g, const gimbal_driver::msg::Vel& m)
                                   {
                                       g.Velocity.X = static_cast<int8_t>(m.x);
                                       g.Velocity.Y = static_cast<int8_t>(m.y);
                                   });

            // 姿态控制单独走 Topic，不复用当前串口写结构，避免改包长导致链路不兼容。
            Node.GenSubscriber<ly_control_posture>(
                [this](ly_control_posture::CallbackArg m)
                {
                    const auto cmd = m->data;
                    if (cmd != 0 && !IsValidPosture(cmd)) {
                        roslog::warn("Invalid /ly/control/posture: %u (expect 0/1/2/3)", cmd);
                        return;
                    }
                    postureCommand_ = cmd;
                    // 在下位机尚未回传姿态前，先把控制量透传到 /ly/gimbal/posture 便于联调。
                    PublishPosture(postureCommand_);
                    if (cmd == 0) {
                        posturePendingRepeat_ = 0;
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

            // 约定: ExtendData.Reserve_16 的低 2 bit 为姿态状态(1/2/3)，0 表示未知。
            // 这样不改报文长度，先做兼容接入。
            const auto posture_raw = static_cast<std::uint8_t>(data.Reserve_16 & 0x03u);
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
                        PubGameData(m.GetDataAs<GameData>());
                        break;

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
                        PubExtendData(m.GetDataAs<ExtendData>());
                        break;

                    default:
                        roslog::error("Application::LoopRead: invalid type id({})", m.TypeID);
                        break;
                }
            });
        }

        void TestVirtualLoopback(){
            TypedMessage<sizeof(GimbalData)> test_msg{};
            TypedMessage<sizeof(GimbalControlData)> test_msg3{};
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
                [this](const auto& data) { if (DeviceError) return; if (!Device.Write(data)) DeviceError = true; }
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
            Node.GetParam<bool>("io_config/use_virtual_device", useVirtualDevice, false);
            roslog::warn("posture_tx fixed mode: type_id=%u repeat_count=%d repeat_interval_ms=%d",
                         postureTxTypeID_, postureTxRepeatCount_,
                         static_cast<int>(postureTxInterval_.count()));

            while (rclcpp::ok())
            {
                if (!DeviceError) DeviceError = true;
                std::this_thread::sleep_for(1s);
                if (!Device.Initialize(useVirtualDevice)) continue;
                DeviceError = false;
                postureLastSent_ = 0;
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
