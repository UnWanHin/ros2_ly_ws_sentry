// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

/*
 * car_tracker_solver_node
 *
 * 链路位置：
 * /ly/detector/armors -> tracker+solver -> /ly/tracker/results
 *
 * 主要功能：
 * - 检测结果融合、目标关联与跟踪
 * - 在统一时间源下执行坐标解算，输出给 predictor
 */
// [ROS 2] 引入頭文件
#include <rclcpp/rclcpp.hpp>
#include "car_tracker/tracker.hpp"
#include "car_tracker/tracker_matcher.hpp"
#include "solver/solver.hpp"
#include <RosTools/RosTools.hpp>

// [ROS 2] 消息頭文件
#include <auto_aim_common/msg/armors.hpp>
#include <auto_aim_common/DetectionType.hpp>
#include <auto_aim_common/TrackerType.hpp>
#include <auto_aim_common/msg/trackers.hpp>
#include <auto_aim_common/msg/car_tracker.hpp>
#include <auto_aim_common/msg/armor_tracker.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <Logger/Logger.hpp>
#include <fmt/format.h>

using namespace LangYa;
using namespace ly_auto_aim;

namespace {
    rclcpp::Node::SharedPtr global_node_ptr = nullptr;
}

namespace roslog_adapter {
    static rclcpp::Logger get_logger() {
        if (global_node_ptr) {
            return global_node_ptr->get_logger();
        }
        return rclcpp::get_logger("car_tracker_solver");
    }
    template <typename... Args>
    void warn(const char* format_str, const Args&... args) {
        RCLCPP_WARN(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void info(const char* format_str, const Args&... args) {
        RCLCPP_INFO(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
}
#define roslog roslog_adapter

namespace{

    LY_DEF_ROS_TOPIC(ly_detector_armors, "/ly/detector/armors", auto_aim_common::msg::Armors);
    LY_DEF_ROS_TOPIC(ly_tracker_results, "/ly/tracker/results", auto_aim_common::msg::Trackers);

    constexpr const char AppName[] = "car_tracker_solver";

    class CarTrackerSolverNode{

    public:
        // 【修改 1】構造函數清空，只初始化 node
        CarTrackerSolverNode() : node() {
        }

        // 【修改 2】新增 Init 函數，延後初始化
        void Init() {
            // 現在這裡執行時，全域指針已經有值了，Solver 才能讀到參數！
            tracker = ly_auto_aim::tracker::createTracker(); 
            solver = ly_auto_aim::solver::createSolver();
            
            location::Location::registerSolver(solver);
            
            // 訂閱也放在這裡，確保 solver 準備好之後才開始收數據
            node.GenSubscriber<ly_detector_armors>([this](const auto_aim_common::msg::Armors::ConstSharedPtr msg) { 
                detection_callback(msg); 
            });

            RCLCPP_INFO(node.get_logger(), "Tracker Solver Initialized Successfully!");
        }

        ~CarTrackerSolverNode() = default;

        void convertToDetections(const auto_aim_common::msg::Armors::ConstSharedPtr& msg, Detections& detections) {
            detections.clear();
            detections.reserve(msg->armors.size());
            for (const auto& armor : msg->armors) {
                detections.emplace_back(Detection{
                    .tag_id = armor.type,
                    .corners = {
                        {armor.corners_x[0], armor.corners_y[0]},
                        {armor.corners_x[1], armor.corners_y[1]},
                        {armor.corners_x[2], armor.corners_y[2]},
                        {armor.corners_x[3], armor.corners_y[3]}
                    },
                    // 這裡可以傳遞更多信息如果需要
                });
            }
        }

        void convertToCarDetections(const auto_aim_common::msg::Armors::ConstSharedPtr& msg, CarDetections& car_detections) {
            car_detections.clear();
            car_detections.reserve(msg->cars.size());
            for (const auto& car : msg->cars) {
                car_detections.emplace_back(CarDetection{
                    .bounding_rect = {
                        car.bounding_rect.x,
                        car.bounding_rect.y,
                        car.bounding_rect.width,
                        car.bounding_rect.height
                    },
                    .tag_id = car.car_id
                });
            }
        }

        void publish_all(const auto& track_results, auto& tracks_msg){
            for(const auto& armor_track_result : track_results.first){
                auto_aim_common::msg::ArmorTracker armor_tracker_msg;
                XYZ armor_xyz = armor_track_result.location.xyz_imu;
                armor_tracker_msg.x = armor_xyz.x;
                armor_tracker_msg.y = armor_xyz.y;
                armor_tracker_msg.z = armor_xyz.z;
                armor_tracker_msg.yaw = armor_track_result.yaw;
                armor_tracker_msg.armor_id = armor_track_result.armor_id;
                armor_tracker_msg.car_id = armor_track_result.car_id;
                tracks_msg.armor_trackers.emplace_back(std::move(armor_tracker_msg));
            }
            for(const auto& car_track_result : track_results.second){
                auto_aim_common::msg::CarTracker car_tracker_msg;
                car_tracker_msg.car_id = car_track_result.car_id;
                car_tracker_msg.bounding_rect.x = car_track_result.bounding_rect.x;
                car_tracker_msg.bounding_rect.y = car_track_result.bounding_rect.y;
                car_tracker_msg.bounding_rect.width = car_track_result.bounding_rect.width;
                car_tracker_msg.bounding_rect.height = car_track_result.bounding_rect.height;
                tracks_msg.car_trackers.emplace_back(std::move(car_tracker_msg));
            }
        }

        void detection_callback(const auto_aim_common::msg::Armors::ConstSharedPtr msg){
            // 雙重保險：如果 solver 還沒初始化，直接返回
            if (!solver) return;

            auto_aim_common::msg::Trackers trackers_msg;
            trackers_msg.header.stamp = msg->header.stamp;
            trackers_msg.header.frame_id = msg->header.frame_id;
            trackers_msg.pitch = msg->pitch; 
            trackers_msg.yaw = msg->yaw;

            std::vector<Detection> detections;
            convertToDetections(msg, detections);
            std::vector<CarDetection> car_detections;
            convertToCarDetections(msg, car_detections);

            GimbalAngleType gimbal_angle{msg->pitch, msg->yaw};
            
            tracker->merge(detections);
            tracker->merge(car_detections);
            
            // =========================================================================
            // 【安全修復】 
            // 這裡為了防止 "Different Time Sources" 報錯，我們也統一轉成 double
            // 這跟 Predictor 的修復邏輯是一樣的，確保萬無一失
            // =========================================================================
            double msg_time_sec = rclcpp::Time(msg->header.stamp).seconds();
            Time::TimeStamp timestamp(msg_time_sec);
            // =========================================================================

            auto track_results = tracker->getTrackResult(timestamp, gimbal_angle);
            
            // 這裡如果 solver 參數沒讀對，之前會崩潰
            if (!track_results.first.empty()) {
                solver->solve_all(track_results, gimbal_angle);
            }
            
            publish_all(track_results, trackers_msg);
            
            node.Publisher<ly_tracker_results>()->publish(trackers_msg);
        }
    
    public:
        ROSNode<AppName> node;
    private:
        std::mutex data_mutex;
        std::unique_ptr<ly_auto_aim::tracker::Tracker> tracker;
        std::shared_ptr<ly_auto_aim::solver::Solver> solver;
        ly_auto_aim::solver::CameraIntrinsicsParameterPack cameraIntrinsics{};
    };
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    auto app = std::make_shared<CarTrackerSolverNode>();
    std::shared_ptr<rclcpp::Node> node_ptr(&app->node, [](auto*){});
    
    // 【修改 3】先賦值，後初始化
    ly_auto_aim::solver::global_tracker_solver_node = node_ptr;
    global_node_ptr = node_ptr; 

    // 執行初始化邏輯 (此時 global_tracker_solver_node 已就緒)
    app->Init();

    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
