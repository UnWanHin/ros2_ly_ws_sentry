// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

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
#include <auto_aim_common/msg/predictor_vis.hpp>

#include "predictor/predictor.hpp"
#include "controller/controller.hpp"
#include "solver/solver.hpp"

#include <atomic>
#include <cstdint>
#include <cmath>
#include <limits>

using namespace LangYa;
using namespace ly_auto_aim;

namespace {
    LY_DEF_ROS_TOPIC(ly_tracker_results, "/ly/tracker/results", auto_aim_common::msg::Trackers);
    LY_DEF_ROS_TOPIC(ly_predictor_target, "/ly/predictor/target", auto_aim_common::msg::Target);
    LY_DEF_ROS_TOPIC(ly_predictor_debug, "/ly/predictor/debug", auto_aim_common::msg::DebugFilter);
    LY_DEF_ROS_TOPIC(ly_predictor_vis, "/ly/predictor/vis", auto_aim_common::msg::PredictorVis);
    LY_DEF_ROS_TOPIC(ly_bt_target, "/ly/bt/target", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_bullet_speed, "/ly/bullet/speed", std_msgs::msg::Float32);
    
    constexpr const char AppName[] = "predictor_node";
    std::atomic<ArmorType> automic_target;
    std::atomic<float> atomic_bullet_speed{23.0f};

    const char* InvalidReasonToString(const ly_auto_aim::controller::ControlInvalidReason reason) {
        using ly_auto_aim::controller::ControlInvalidReason;
        switch (reason) {
            case ControlInvalidReason::None: return "none";
            case ControlInvalidReason::NoPrediction: return "no_prediction";
            case ControlInvalidReason::InvalidCar: return "invalid_car";
            case ControlInvalidReason::InvalidCarAfterFlyTime: return "invalid_car_after_flytime";
            case ControlInvalidReason::NoArmorFallbackBallisticFail: return "no_armor_fallback_ballistic_fail";
            case ControlInvalidReason::InvalidArmor: return "invalid_armor";
            case ControlInvalidReason::ArmorBallisticFail: return "armor_ballistic_fail";
            case ControlInvalidReason::UnstableTrack: return "unstable_track";
            default: return "unknown";
        }
    }

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
                    const double t_sec = rclcpp::Time(timestamp).seconds();
                    return predictor->predict(Time::TimeStamp(t_sec));
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
                const double msg_time_sec = rclcpp::Time(msg->header.stamp).seconds();
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
                auto_aim_common::msg::PredictorVis predictor_vis_msg;
                bool publish_target = false;
                bool publish_debug = false;
                bool publish_vis = false;

                {
                    std::lock_guard<std::mutex> lock(data_mutex);

                    const auto now = node.now();
                    const double now_sec = now.seconds();
                    const auto target = static_cast<int>(automic_target.load());
                    const auto bullet_speed = static_cast<float>(atomic_bullet_speed.load());
                    const Time::TimeStamp timestamp(now_sec);
                    const auto predictions = predictor->predict(timestamp);
                    const bool has_predictions = !predictions.empty();
                    const bool observation_fresh =
                        last_observation_time_.nanoseconds() != 0 &&
                        (now - last_observation_time_) <= coast_timeout_;
                    const auto observation_age_ms =
                        last_observation_time_.nanoseconds() == 0
                            ? -1.0
                            : (now - last_observation_time_).seconds() * 1000.0;
                    const bool can_log_invalid_reason =
                        last_invalid_reason_log_time_.nanoseconds() == 0 ||
                        (now - last_invalid_reason_log_time_) > invalid_reason_log_interval_;

                    target_msg.header = last_tracker_header_;
                    target_msg.header.stamp = now;
                    target_msg.buff_follow = false;
                    predictor_vis_msg.header = target_msg.header;
                    predictor_vis_msg.has_predictions = has_predictions;
                    predictor_vis_msg.aimed_car_id = -1;
                    predictor_vis_msg.aimed_armor_id = -1;
                    predictor_vis_msg.cars.clear();

                    if (has_predictions) {
                        predictor_vis_msg.cars.reserve(predictions.size());
                        for (const auto& prediction : predictions) {
                            auto_aim_common::msg::PredictorCarVis car_vis_msg;
                            car_vis_msg.car_id = prediction.id;
                            car_vis_msg.stable = prediction.stable;
                            car_vis_msg.center.x = prediction.center.x;
                            car_vis_msg.center.y = prediction.center.y;
                            car_vis_msg.center.z = prediction.center.z;
                            car_vis_msg.armors.reserve(prediction.armors.size());
                            for (const auto& armor : prediction.armors) {
                                auto_aim_common::msg::PredictorArmorVis armor_vis_msg;
                                armor_vis_msg.id = armor.id;
                                armor_vis_msg.status = static_cast<std::int32_t>(armor.status);
                                armor_vis_msg.center.x = armor.center.x;
                                armor_vis_msg.center.y = armor.center.y;
                                armor_vis_msg.center.z = armor.center.z;
                                armor_vis_msg.yaw = static_cast<float>(armor.yaw);
                                armor_vis_msg.theta = static_cast<float>(armor.theta);
                                car_vis_msg.armors.push_back(std::move(armor_vis_msg));
                            }
                            predictor_vis_msg.cars.push_back(std::move(car_vis_msg));
                        }
                    }
                    publish_vis = true;

                    if (!has_predictions && !observation_fresh) {
                        // Keep the predictor's robust stale/no-prediction handling,
                        // but do not publish invalid targets to behavior_tree.
                        target_msg.status = false;
                    } else {
                        const auto control_result =
                            controller->control(last_gimbal_angle_, target, bullet_speed);
                        target_msg.status = control_result.valid;
                        target_msg.yaw = control_result.yaw_actual_want;
                        target_msg.pitch = control_result.pitch_actual_want;

                        const bool finite_target =
                            std::isfinite(target_msg.yaw) && std::isfinite(target_msg.pitch);
                        if (target_msg.status && finite_target) {
                            publish_target = true;
                            for (const auto& prediction : predictions) {
                                debug_filter_msg.tracking = true;
                                XYZ car_XYZ = prediction.center;
                                debug_filter_msg.position.x = car_XYZ.x;
                                debug_filter_msg.position.y = car_XYZ.y;
                                debug_filter_msg.position.z = car_XYZ.z;
                                debug_filter_msg.yaw = prediction.theta;
                                debug_filter_msg.v_yaw = prediction.omega;
                                debug_filter_msg.velocity.x = prediction.vx;
                                debug_filter_msg.velocity.y = prediction.vy;
                                debug_filter_msg.velocity.z = 0.0;
                                debug_filter_msg.radius_1 = prediction.r1;
                                debug_filter_msg.radius_2 = prediction.r2;
                                debug_filter_msg.z_2 = prediction.z2;
                            }
                            publish_debug = has_predictions;
                        }

                        if ((!target_msg.status || !finite_target) && can_log_invalid_reason) {
                            RCLCPP_INFO(
                                node.get_logger(),
                                "predictor target suppressed reason=%s has_predictions=%s observation_fresh=%s observation_age_ms=%.1f finite_target=%s yaw=%.2f pitch=%.2f",
                                InvalidReasonToString(control_result.invalid_reason),
                                has_predictions ? "true" : "false",
                                observation_fresh ? "true" : "false",
                                observation_age_ms,
                                finite_target ? "true" : "false",
                                target_msg.yaw,
                                target_msg.pitch);
                            last_invalid_reason_log_time_ = now;
                        }
                    }

                    if (!target_msg.status && !has_predictions && !observation_fresh &&
                        can_log_invalid_reason) {
                        RCLCPP_INFO(
                            node.get_logger(),
                            "predictor status=false reason=no_predictions_and_stale has_predictions=false observation_fresh=false observation_age_ms=%.1f",
                            observation_age_ms);
                        last_invalid_reason_log_time_ = now;
                    }
                }

                if (publish_target) {
                    node.Publisher<ly_predictor_target>()->publish(target_msg);
                }
                if (publish_debug) {
                    node.Publisher<ly_predictor_debug>()->publish(debug_filter_msg);
                }
                if (publish_vis) {
                    node.Publisher<ly_predictor_vis>()->publish(predictor_vis_msg);
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
            const rclcpp::Duration coast_timeout_{rclcpp::Duration::from_seconds(0.5)};
            rclcpp::Time last_invalid_reason_log_time_{};
            const rclcpp::Duration invalid_reason_log_interval_{rclcpp::Duration::from_seconds(0.5)};
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
