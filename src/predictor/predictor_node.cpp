/*
 * predictor_node
 *
 * 链路位置：
 * /ly/tracker/results -> predictor/controller -> /ly/predictor/target
 *
 * 主要功能：
 * - 根据 tracker 结果做状态预测
 * - 结合当前目标类型(/ly/bt/target)与弹速(/ly/bullet/speed)输出控制目标
 * - 发布 debug_filter 供可视化和调参
 */
#include <rclcpp/rclcpp.hpp>
#include <RosTools/RosTools.hpp>
#include "Logger/Logger.hpp"
#include <TimeStamp/TimeStamp.hpp>
#include <auto_aim_common/Location.hpp>
#include <auto_aim_common/TrackerType.hpp>
#include <auto_aim_common/PredictionType.hpp>
#include <auto_aim_common/DetectionType.hpp>

// [ROS 2] 消息頭文件
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <auto_aim_common/msg/car_tracker.hpp>
#include <auto_aim_common/msg/armor_tracker.hpp>
#include <auto_aim_common/msg/trackers.hpp>
#include <auto_aim_common/msg/target.hpp>
#include <auto_aim_common/msg/debug_filter.hpp>

#include "predictor/predictor.hpp"
#include "controller/controller.hpp"
#include "solver/solver.hpp"

#include <atomic>
#include <limits>

using namespace LangYa;
using namespace ly_auto_aim;

namespace {
    LY_DEF_ROS_TOPIC(ly_tracker_results, "/ly/tracker/results", auto_aim_common::msg::Trackers);
    LY_DEF_ROS_TOPIC(ly_predictor_target, "/ly/predictor/target", auto_aim_common::msg::Target);
    LY_DEF_ROS_TOPIC(ly_predictor_debug, "/ly/predictor/debug", auto_aim_common::msg::DebugFilter);
    LY_DEF_ROS_TOPIC(ly_bt_target, "/ly/bt/target", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_bullet_speed, "/ly/bullet/speed", std_msgs::msg::Float32);
    
    constexpr const char AppName[] = "predictor_node";
    std::atomic<ArmorType> automic_target;
    std::atomic<float> atomic_bullet_speed{23.0f};

    class PredictorNode {
        public:
            // 【修改 1】構造函數清空！只做最基本的 node 初始化
            PredictorNode() : node() { 
            }

            // 【修改 2】Init 函數，延後初始化
            void Init() {
                solver = ly_auto_aim::solver::createSolver(); 
                predictor = ly_auto_aim::predictor::createPredictor();
                controller = ly_auto_aim::controller::createController();
                
                location::Location::registerSolver(solver);
                
                controller->registPredictFunc([this](Time::TimeStamp timestamp) {
                    return predictor->predict(timestamp);
                });

                node.GenSubscriber<ly_tracker_results>([this](const auto_aim_common::msg::Trackers::ConstSharedPtr msg) { 
                    predictor_callback(msg); 
                });
                node.GenSubscriber<ly_bt_target>([this](const std_msgs::msg::UInt8::ConstSharedPtr msg) { 
                    get_target_callback(msg); 
                });
                node.GenSubscriber<ly_bullet_speed>([this](const std_msgs::msg::Float32::ConstSharedPtr msg) { 
                    get_bullet_speed_callback(msg); 
                });

                publish_timer_ = node.create_wall_timer(
                    std::chrono::milliseconds(10),
                    [this]() { publish_timer_callback(); });
                
                RCLCPP_INFO(node.get_logger(), "Predictor Modules Initialized Successfully!");
            }

            ~PredictorNode() = default;

            void get_target_callback(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
                automic_target = static_cast<ArmorType>(msg->data);
            }

            void get_bullet_speed_callback(const std_msgs::msg::Float32::ConstSharedPtr msg) {
                atomic_bullet_speed = static_cast<float>(msg->data);
            }

            void convertMsgToTrackResults(const auto_aim_common::msg::Trackers::ConstSharedPtr& msg, TrackResultPairs& track_results, GimbalAngleType gimbal_angle){
                track_results.first.clear();
                track_results.second.clear();
                for (const auto& armor_tra_msg : msg->armor_trackers) {
                    TrackResult armor_track_result;
                    armor_track_result.car_id = armor_tra_msg.car_id;
                    armor_track_result.armor_id = armor_tra_msg.armor_id;
                    armor_track_result.yaw = armor_tra_msg.yaw;
                    armor_track_result.location.imu = gimbal_angle;
                    XYZ armor_xyz(armor_tra_msg.x, armor_tra_msg.y, armor_tra_msg.z); 
                    armor_track_result.location.xyz_imu = armor_xyz;
                    track_results.first.emplace_back(std::move(armor_track_result));
                }
                for (const auto& car_tra_msg : msg->car_trackers) {
                    CarTrackResult car_track_result;
                    car_track_result.car_id = car_tra_msg.car_id;
                    car_track_result.bounding_rect.x = car_tra_msg.bounding_rect.x;
                    car_track_result.bounding_rect.y = car_tra_msg.bounding_rect.y;
                    car_track_result.bounding_rect.width = car_tra_msg.bounding_rect.width;
                    car_track_result.bounding_rect.height = car_tra_msg.bounding_rect.height;
                    track_results.second.emplace_back(std::move(car_track_result));
                }
            }

            void predictor_callback(const auto_aim_common::msg::Trackers::ConstSharedPtr msg){
                GimbalAngleType gimbal_angle{msg->pitch, msg->yaw};

                // 增加一個保護，避免還沒初始化就調用
                if(!location::Location::isSolverRegistered()){
                    return; 
                }

                TrackResultPairs track_results;
                convertMsgToTrackResults(msg, track_results, gimbal_angle);
                double msg_time_sec = rclcpp::Time(msg->header.stamp).seconds();
                Time::TimeStamp timestamp(msg_time_sec);

                std::lock_guard<std::mutex> lock(data_mutex);
                predictor->update(track_results, timestamp);
                last_gimbal_angle_ = gimbal_angle;
                last_tracker_header_ = msg->header;
                last_update_time_ = node.now();
                has_tracker_input_ = true;
                if (!track_results.first.empty()) {
                    last_observation_time_ = last_update_time_;
                }
            }

            void publish_timer_callback() {
                if (!has_tracker_input_ || !location::Location::isSolverRegistered()) {
                    return;
                }

                auto_aim_common::msg::Target target_msg;
                auto_aim_common::msg::DebugFilter debug_filter_msg;
                bool publish_debug = false;

                {
                    std::lock_guard<std::mutex> lock(data_mutex);

                    const auto now = node.now();
                    const auto target = static_cast<int>(automic_target.load());
                    const auto bullet_speed = static_cast<float>(atomic_bullet_speed.load());
                    const Time::TimeStamp timestamp = now;
                    const auto predictions = predictor->predict(timestamp);
                    const bool has_predictions = !predictions.empty();
                    const bool observation_fresh =
                        last_observation_time_.nanoseconds() != 0 &&
                        (now - last_observation_time_) <= coast_timeout_;

                    target_msg.header = last_tracker_header_;
                    target_msg.header.stamp = now;
                    target_msg.buff_follow = false;

                    if (!has_predictions && !observation_fresh) {
                        const auto nan = std::numeric_limits<float>::quiet_NaN();
                        target_msg.status = false;
                        target_msg.yaw = nan;
                        target_msg.pitch = nan;
                    } else {
                        const auto control_result =
                            controller->control(last_gimbal_angle_, target, bullet_speed);
                        target_msg.status = control_result.valid && observation_fresh;
                        target_msg.yaw = control_result.yaw_actual_want;
                        target_msg.pitch = control_result.pitch_actual_want;

                        for (const auto& prediction : predictions) {
                            XYZ car_XYZ = prediction.center;
                            debug_filter_msg.position.x = car_XYZ.x;
                            debug_filter_msg.position.y = car_XYZ.y;
                            debug_filter_msg.position.z = car_XYZ.z;
                            debug_filter_msg.yaw = prediction.theta;
                            debug_filter_msg.v_yaw = prediction.omega;
                            debug_filter_msg.velocity.x = prediction.vx;
                            debug_filter_msg.velocity.y = prediction.vy;
                            debug_filter_msg.radius_1 = prediction.r1;
                            debug_filter_msg.radius_2 = prediction.r2;
                        }
                        publish_debug = target_msg.status && has_predictions;
                    }
                }

                node.Publisher<ly_predictor_target>()->publish(target_msg);
                if (publish_debug) {
                    node.Publisher<ly_predictor_debug>()->publish(debug_filter_msg);
                }
            }

        public: 
            ROSNode<AppName> node;
        private:
            std::shared_ptr<ly_auto_aim::solver::Solver> solver;
            std::unique_ptr<ly_auto_aim::predictor::Predictor> predictor;
            std::shared_ptr<ly_auto_aim::controller::Controller> controller;
            std::mutex data_mutex;
            rclcpp::TimerBase::SharedPtr publish_timer_{};
            GimbalAngleType last_gimbal_angle_{0.0, 0.0};
            std_msgs::msg::Header last_tracker_header_{};
            rclcpp::Time last_update_time_{};
            rclcpp::Time last_observation_time_{};
            std::atomic_bool has_tracker_input_{false};
            const rclcpp::Duration coast_timeout_{rclcpp::Duration::from_seconds(0.25)};
    };
}

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    rclcpp::init(argc, argv);
    auto app = std::make_shared<PredictorNode>();
    std::shared_ptr<rclcpp::Node> node_ptr(&app->node, [](auto*){});

    ly_auto_aim::controller::global_controller_node = node_ptr;
    ly_auto_aim::solver::global_predictor_solver_node = node_ptr;
    ly_auto_aim::predictor::global_predictor_node = node_ptr;

    app->Init();

    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
