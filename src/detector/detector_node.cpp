// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

/*
 * detector_node
 *
 * 核心职责：
 * 1) 输入图像（相机/视频），执行装甲板+车辆检测
 * 2) 输出 /ly/detector/armors 供 tracker_solver 使用
 * 3) 读取 /ly/me/is_team_red 与 /ly/bt/target 做目标过滤
 *
 * 兼容策略：
 * - 参数同时支持 dot/slash 两种命名，统一读 auto_aim_config.yaml。
 */
#include <auto_aim_common/msg/armor.hpp>
#include <auto_aim_common/msg/armors.hpp>
#include <auto_aim_common/msg/debug_filter.hpp>
#include <auto_aim_common/msg/predictor_vis.hpp>
#include <gimbal_driver/msg/gimbal_angles.hpp>
#include <auto_aim_common/msg/angle_image.hpp>
#include <Logger/Logger.hpp>
#include <RosTools/RosTools.hpp>

#include <thread>
#include <chrono>
#include <iostream> 
#include <filesystem>
#include <initializer_list>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// TODO check AI content
// ROS2 核心头文件
#include "rclcpp/rclcpp.hpp"

// 标准消息与传感器消息
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// CvBridge (ROS2 版本)
#include "cv_bridge/cv_bridge.h"

// 你的自定义消息 (假设包名为 auto_aim_common)
// 注意：ROS1 的 .msg 文件在 ROS2 中生成的头文件带 .hpp 后缀，且在 msg 命名空间下
#include "auto_aim_common/msg/armors.hpp"
#include "auto_aim_common/msg/angle_image.hpp"

#include <boost/lockfree/stack.hpp>
#include <boost/atomic.hpp>
#include <Eigen/Geometry>

#include "armor_detector/armor_filter.hpp"
#include "armor_detector/armor_refinder.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "car_and_armor_detector.hpp"
#include "car_detector/car_finder.hpp"
#include <auto_aim_common/DetectionType.hpp>
#include <VideoStreamer/VideoStreamer.hpp>
#include "module/Camera.hpp"

namespace {

LY_DEF_ROS_TOPIC(ly_aa_enable, "/ly/aa/enable", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_ra_enable, "/ly/ra/enable", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_outpost_enable, "/ly/outpost/enable", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_camera_image, "/ly/camera/image", sensor_msgs::msg::Image);
LY_DEF_ROS_TOPIC(ly_backcamera_image, "/ly/backcamera/image", sensor_msgs::msg::Image);
LY_DEF_ROS_TOPIC(ly_me_is_team_red, "/ly/me/is_team_red", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_bt_target, "/ly/bt/target", std_msgs::msg::UInt8);
LY_DEF_ROS_TOPIC(ly_detector_armors, "/ly/detector/armors", auto_aim_common::msg::Armors);
LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
LY_DEF_ROS_TOPIC(ly_compressed_image, "/ly/compressed/image", sensor_msgs::msg::CompressedImage);
LY_DEF_ROS_TOPIC(ly_ra_angle_image, "/ly/ra/angle_image", auto_aim_common::msg::AngleImage);
LY_DEF_ROS_TOPIC(ly_outpost_armors, "/ly/outpost/armors", auto_aim_common::msg::Armors);
LY_DEF_ROS_TOPIC(ly_predictor_debug, "/ly/predictor/debug", auto_aim_common::msg::DebugFilter);
LY_DEF_ROS_TOPIC(ly_predictor_vis, "/ly/predictor/vis", auto_aim_common::msg::PredictorVis);

/// 配置变量
bool pub_image = true;
bool web_show = true;
bool draw_image = true;
bool debug_mode = false;
bool debug_team_blue = false;
bool save_video = false;
bool use_video = false;
bool use_ros_bag = false;
std::string video_path;
std::string camera_sn;
bool overlay_predictor_center = false;
bool overlay_predictor_full = false;
int overlay_predictor_timeout_ms = 120;

/// 宏常量与状态量
std::atomic_bool myTeamRed{false};
std::atomic_bool aa_enable{true};
std::atomic_bool ra_enable{false};
std::atomic_bool outpost_enable{false};

constexpr const char AppName[] = "detector";
std::shared_ptr<LangYa::ROSNode<AppName>> global_node = nullptr;

ly_auto_aim::CarAndArmorDetector carAndArmorDetector{};
ly_auto_aim::ArmorFilter filter{};
ly_auto_aim::ArmorRefinder finder{};
ly_auto_aim::CarFinder carFinder{};
ly_auto_aim::CameraIntrinsicsParameterPack cameraIntrinsics{};
std::unique_ptr<ly_auto_aim::PoseSolver> poseSolver{};

LangYa::Camera Cam;
Eigen::Vector3d overlay_camera_offset = Eigen::Vector3d::Zero();
Eigen::Matrix3d overlay_camera_rotation_matrix = Eigen::Matrix3d::Identity();

std::mutex predictor_debug_mutex;
auto_aim_common::msg::DebugFilter latest_predictor_debug;
rclcpp::Time latest_predictor_debug_stamp;
bool has_predictor_debug = false;

std::mutex predictor_vis_mutex;
auto_aim_common::msg::PredictorVis latest_predictor_vis;
rclcpp::Time latest_predictor_vis_stamp;
bool has_predictor_vis = false;

struct GimbalAnglesType {
    float yaw{0.0f};
    float pitch{0.0f};
}; 

#pragma region image_queue
class ImageQueue {
private:
    std::queue<cv::Mat> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    const size_t max_size_ = 10; 

public:
    void push(cv::Mat image) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) {
            queue_.pop(); 
        }
        queue_.push(image.clone()); 
        cond_.notify_all(); 
    }

    cv::Mat wait_and_pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return !queue_.empty(); });
        auto image = queue_.front();
        queue_.pop();
        return image;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
};
#pragma endregion image_quene

#pragma region image_stack
constexpr size_t MAX_STACK_SIZE = 30; 
#pragma endregion image_stack

std::atomic<float> gimbal_angles_yaw;
std::atomic<float> gimbal_angles_pitch;

std::atomic<ly_auto_aim::ArmorType> atomic_target{};
ly_auto_aim::PNPAimResult aim_result{};

template<typename T>
bool TryReadParam(const std::initializer_list<const char*>& keys, T& value) {
    for (const auto* key : keys) {
        if (global_node->has_parameter(key) && global_node->get_parameter(key, value)) {
            return true;
        }
    }
    return false;
}

template<typename T>
void LoadParamCompat(const std::initializer_list<const char*>& keys, T& value, const T& default_value) {
    if (TryReadParam(keys, value)) {
        return;
    }
    const auto* primary = *keys.begin();
    if (!global_node->has_parameter(primary)) {
        global_node->declare_parameter<T>(primary, default_value);
    }
    global_node->get_parameter(primary, value);
    roslog::error("Parameter '{}' missing, fallback default applied.", primary);
}

void LoadSolverIntrinsicsParam() {
    std::vector<double> intrinsic_flat;
    LoadParamCompat<std::vector<double>>(
        {"solver_config.camera_intrinsic_matrix", "solver_config/camera_intrinsic_matrix"},
        intrinsic_flat,
        {}
    );
    if (intrinsic_flat.size() == 9) {
        cameraIntrinsics.FocalLength[0] = static_cast<float>(intrinsic_flat[0]);
        cameraIntrinsics.FocalLength[1] = static_cast<float>(intrinsic_flat[4]);
        cameraIntrinsics.PrincipalPoint[0] = static_cast<float>(intrinsic_flat[2]);
        cameraIntrinsics.PrincipalPoint[1] = static_cast<float>(intrinsic_flat[5]);
    } else {
        roslog::warn("solver_config camera_intrinsic_matrix size invalid (need 9, got {}). Use builtin defaults.", intrinsic_flat.size());
    }

    std::vector<double> distortion_vec;
    LoadParamCompat<std::vector<double>>(
        {"solver_config.camera_distortion_coefficients", "solver_config/camera_distortion_coefficients"},
        distortion_vec,
        {}
    );
    if (distortion_vec.size() == 5) {
        cameraIntrinsics.RadialDistortion[0] = static_cast<float>(distortion_vec[0]);
        cameraIntrinsics.RadialDistortion[1] = static_cast<float>(distortion_vec[1]);
        cameraIntrinsics.TangentialDistortion[0] = static_cast<float>(distortion_vec[2]);
        cameraIntrinsics.TangentialDistortion[1] = static_cast<float>(distortion_vec[3]);
        cameraIntrinsics.RadialDistortion[2] = static_cast<float>(distortion_vec[4]);
    } else {
        roslog::warn("solver_config camera_distortion_coefficients size invalid (need 5, got {}). Use builtin defaults.", distortion_vec.size());
    }
}

void LoadSolverExtrinsicsParam() {
    std::vector<double> offset_vec;
    LoadParamCompat<std::vector<double>>(
        {"solver_config.camera_offset", "solver_config/camera_offset"},
        offset_vec,
        {0.0, 0.0, 0.0});
    if (offset_vec.size() == 3) {
        overlay_camera_offset = Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]) * 0.001;
    } else {
        overlay_camera_offset = Eigen::Vector3d::Zero();
        roslog::warn("solver_config camera_offset size invalid (need 3, got {}). Use zeros.", offset_vec.size());
    }

    std::vector<double> rotation_vec;
    LoadParamCompat<std::vector<double>>(
        {"solver_config.camera_rotation", "solver_config/camera_rotation"},
        rotation_vec,
        {0.0, 0.0, 0.0});
    if (rotation_vec.size() == 3) {
        cv::Mat rotationVector(3, 1, CV_64F);
        for (int i = 0; i < 3; ++i) {
            rotationVector.at<double>(i) = rotation_vec[i] * M_PI / 180.0;
        }
        cv::Mat rotationMatrix;
        cv::Rodrigues(rotationVector, rotationMatrix);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                overlay_camera_rotation_matrix(i, j) = rotationMatrix.at<double>(i, j);
            }
        }
    } else {
        overlay_camera_rotation_matrix = Eigen::Matrix3d::Identity();
        roslog::warn("solver_config camera_rotation size invalid (need 3, got {}). Use identity.", rotation_vec.size());
    }
}

void InitialParam(){
    // 兼容点号/斜杠两种参数命名，保证一份 YAML 覆盖全链路。
    LoadParamCompat<bool>({"detector_config.show", "detector_config/show"}, pub_image, true);
    LoadParamCompat<bool>({"detector_config.draw", "detector_config/draw"}, draw_image, true);
    LoadParamCompat<bool>({"detector_config.debug_mode", "detector_config/debug_mode"}, debug_mode, false);
    LoadParamCompat<bool>({"detector_config.debug_team_blue", "detector_config/debug_team_blue"}, debug_team_blue, false);
    LoadParamCompat<bool>({"detector_config.save_video", "detector_config/save_video"}, save_video, false);
    LoadParamCompat<bool>({"detector_config.web_show", "detector_config/web_show"}, web_show, true);
    LoadParamCompat<bool>({"detector_config.use_video", "detector_config/use_video"}, use_video, false);
    LoadParamCompat<bool>({"detector_config.use_ros_bag", "detector_config/use_ros_bag"}, use_ros_bag, false);
    LoadParamCompat<std::string>({"detector_config.video_path", "detector_config/video_path"}, video_path, "");
    LoadParamCompat<std::string>({"camera_param.camera_sn", "camera_param/camera_sn"}, camera_sn, std::string(""));
    LoadParamCompat<bool>(
        {"detector_config.overlay_predictor_center", "detector_config/overlay_predictor_center"},
        overlay_predictor_center,
        false);
    LoadParamCompat<bool>(
        {"detector_config.overlay_predictor_full", "detector_config/overlay_predictor_full"},
        overlay_predictor_full,
        false);
    LoadParamCompat<int>(
        {"detector_config.overlay_predictor_timeout_ms", "detector_config/overlay_predictor_timeout_ms"},
        overlay_predictor_timeout_ms,
        120);
    overlay_predictor_timeout_ms = std::max(0, overlay_predictor_timeout_ms);
}

void ConfigureCamera() {
    Cam.Configure().AutoExposure.Value = GX_EXPOSURE_AUTO_OFF;
    LoadParamCompat<double>({"camera_param.ExposureTime", "camera_param/ExposureTime"}, Cam.Configure().ExposureTime.Value, 4000.0);
    Cam.Configure().AutoGain.Value = GX_GAIN_AUTO_OFF;
    LoadParamCompat<double>({"camera_param.Gain", "camera_param/Gain"}, Cam.Configure().Gain.Value, 12.0);
    LoadParamCompat<double>({"camera_param.RedBalanceRatio", "camera_param/RedBalanceRatio"}, Cam.Configure().RedBalanceRatio.Value, 1.2266);
    LoadParamCompat<double>({"camera_param.GreenBalanceRatio", "camera_param/GreenBalanceRatio"}, Cam.Configure().GreenBalanceRatio.Value, 1.0);
    LoadParamCompat<double>({"camera_param.BlueBalanceRatio", "camera_param/BlueBalanceRatio"}, Cam.Configure().BlueBalanceRatio.Value, 1.3711);
}

std::string ResolvePathFromDetectorShare(const std::string& configured_path,
                                         const std::string& detector_share_dir) {
    if (configured_path.empty()) {
        return configured_path;
    }

    std::filesystem::path raw(configured_path);
    if (std::filesystem::exists(raw)) {
        return std::filesystem::absolute(raw).string();
    }
    if (raw.is_absolute()) {
        return configured_path;
    }

    const std::filesystem::path from_share = std::filesystem::path(detector_share_dir) / raw;
    if (std::filesystem::exists(from_share)) {
        return from_share.string();
    }

    const std::filesystem::path from_extras = std::filesystem::path(detector_share_dir) / "Extras" / raw.filename();
    if (std::filesystem::exists(from_extras)) {
        return from_extras.string();
    }
    return configured_path;
}


void my_team_callback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    myTeamRed = msg->data;
}

void gimbal_callback(const gimbal_driver::msg::GimbalAngles::ConstSharedPtr msg) {
    gimbal_angles_yaw = msg->yaw;
    gimbal_angles_pitch = msg->pitch;
}

void get_target_callback(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
    atomic_target = static_cast<ly_auto_aim::ArmorType>(msg->data);
}

void aa_enable_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
    aa_enable = msg->data;
}

void ra_enable_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
    ra_enable = msg->data;
}

void outpost_enable_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
    outpost_enable = msg->data;
}

void predictor_debug_callback(const auto_aim_common::msg::DebugFilter::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(predictor_debug_mutex);
    latest_predictor_debug = *msg;
    latest_predictor_debug_stamp = global_node->now();
    has_predictor_debug = true;
}

void predictor_vis_callback(const auto_aim_common::msg::PredictorVis::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(predictor_vis_mutex);
    latest_predictor_vis = *msg;
    latest_predictor_vis_stamp = global_node->now();
    has_predictor_vis = true;
}

void DrawArmor(cv::Mat &image, const ly_auto_aim::ArmorObject &armor) {
    for (std::size_t point_index = 0; point_index < 4; ++point_index) {
        cv::circle(image, armor.apex[point_index], 5, cv::Scalar(0, 0, 255), -1);
    }
    
    cv::line(image, armor.apex[0], armor.apex[2], cv::Scalar(255, 0, 0), 2);
    cv::line(image, armor.apex[1], armor.apex[3], cv::Scalar(0, 255, 0), 2);

    const std::string type_text = std::to_string(armor.type);
    cv::Point text_org = armor.apex[0] + cv::Point2f(10, 30); 
    cv::putText(image, type_text, text_org, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
}

void DrawCarBBox(cv::Mat& image, const ly_auto_aim::CarDetection& car){
    cv::rectangle(image, car.bounding_rect, cv::Scalar(0, 255, 0), 2);
}

void DrawAllArmor(cv::Mat &image, const std::vector<ly_auto_aim::ArmorObject> &armors) {
    for (std::size_t armor_index = 0; armor_index < armors.size(); ++armor_index) {
        DrawArmor(image, armors[armor_index]);
    }
}

void DrawAllCar(cv::Mat &image, const std::vector<ly_auto_aim::CarDetection> &cars) {
    for (std::size_t car_index = 0; car_index < cars.size(); ++car_index) {
        DrawCarBBox(image, cars[car_index]);
    }
}

bool ProjectWorldPointToImage(
    const Eigen::Vector3d& world,
    const GimbalAnglesType& gimbal_angles,
    const cv::Mat& image,
    cv::Point& pixel) {
    if (image.empty()) {
        return false;
    }
    const double yaw_rad = static_cast<double>(gimbal_angles.yaw) * M_PI / 180.0;
    const double pitch_rad = static_cast<double>(gimbal_angles.pitch) * M_PI / 180.0;
    const Eigen::Matrix3d world_to_gimbal =
        Eigen::AngleAxisd(-yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d gimbal = world_to_gimbal * world;
    const Eigen::Vector3d camera = overlay_camera_rotation_matrix.inverse() * (gimbal - overlay_camera_offset);
    if (!camera.allFinite() || camera.x() < 1e-4) {
        return false;
    }

    const double fx = cameraIntrinsics.FocalLength[0];
    const double fy = cameraIntrinsics.FocalLength[1];
    const double cx = cameraIntrinsics.PrincipalPoint[0];
    const double cy = cameraIntrinsics.PrincipalPoint[1];
    if (fx <= 1e-6 || fy <= 1e-6) {
        return false;
    }

    const double px = cx - camera.y() / camera.x() * fx;
    const double py = cy - camera.z() / camera.x() * fy;
    if (!std::isfinite(px) || !std::isfinite(py)) {
        return false;
    }
    const int x = static_cast<int>(std::lround(px));
    const int y = static_cast<int>(std::lround(py));
    if (x < 0 || y < 0 || x >= image.cols || y >= image.rows) {
        return false;
    }
    pixel = cv::Point(x, y);
    return true;
}

void DrawPredictorCenterOverlay(cv::Mat& image, const GimbalAnglesType& gimbal_angles, const rclcpp::Time& now) {
    if (!overlay_predictor_center || image.empty()) {
        return;
    }

    auto_aim_common::msg::DebugFilter debug_msg;
    rclcpp::Time debug_stamp;
    {
        std::lock_guard<std::mutex> lock(predictor_debug_mutex);
        if (!has_predictor_debug) {
            return;
        }
        debug_msg = latest_predictor_debug;
        debug_stamp = latest_predictor_debug_stamp;
    }
    if (!debug_msg.tracking) {
        return;
    }
    if (overlay_predictor_timeout_ms > 0 &&
        (now - debug_stamp) > rclcpp::Duration::from_seconds(overlay_predictor_timeout_ms / 1000.0)) {
        return;
    }

    const Eigen::Vector3d world(debug_msg.position.x, debug_msg.position.y, debug_msg.position.z);
    cv::Point pixel;
    if (!ProjectWorldPointToImage(world, gimbal_angles, image, pixel)) {
        return;
    }

    cv::circle(image, pixel, 7, cv::Scalar(180, 180, 180), -1);
    cv::circle(image, pixel, 10, cv::Scalar(120, 255, 120), 2);
}

void DrawPredictorFullOverlay(cv::Mat& image, const GimbalAnglesType& gimbal_angles, const rclcpp::Time& now) {
    if (!overlay_predictor_full || image.empty()) {
        return;
    }

    auto_aim_common::msg::PredictorVis vis_msg;
    rclcpp::Time vis_stamp;
    {
        std::lock_guard<std::mutex> lock(predictor_vis_mutex);
        if (!has_predictor_vis) {
            return;
        }
        vis_msg = latest_predictor_vis;
        vis_stamp = latest_predictor_vis_stamp;
    }
    if (overlay_predictor_timeout_ms > 0 &&
        (now - vis_stamp) > rclcpp::Duration::from_seconds(overlay_predictor_timeout_ms / 1000.0)) {
        return;
    }
    if (!vis_msg.has_predictions) {
        return;
    }

    constexpr int kArmorStatusNonexist = auto_aim_common::msg::PredictorArmorVis::STATUS_NONEXIST;
    constexpr int kArmorStatusUnseen = auto_aim_common::msg::PredictorArmorVis::STATUS_UNSEEN;
    constexpr int kArmorStatusAvailable = auto_aim_common::msg::PredictorArmorVis::STATUS_AVAILABLE;

    for (const auto& car : vis_msg.cars) {
        cv::Point center_pixel;
        if (ProjectWorldPointToImage(
                Eigen::Vector3d(car.center.x, car.center.y, car.center.z),
                gimbal_angles,
                image,
                center_pixel)) {
            cv::circle(image, center_pixel, 4, cv::Scalar(150, 150, 150), -1);
        }

        for (const auto& armor : car.armors) {
            cv::Point armor_pixel;
            if (!ProjectWorldPointToImage(
                    Eigen::Vector3d(armor.center.x, armor.center.y, armor.center.z),
                    gimbal_angles,
                    image,
                    armor_pixel)) {
                continue;
            }

            cv::Scalar color(150, 150, 150);
            if (armor.status == kArmorStatusAvailable) {
                color = cv::Scalar(0, 255, 0);
            } else if (armor.status == kArmorStatusUnseen) {
                color = cv::Scalar(0, 0, 255);
            } else if (armor.status == kArmorStatusNonexist) {
                color = cv::Scalar(180, 180, 180);
            }
            cv::circle(image, armor_pixel, 6, color, -1);

            if (car.car_id == vis_msg.aimed_car_id && armor.id == vis_msg.aimed_armor_id) {
                cv::circle(image, armor_pixel, 10, cv::Scalar(0, 255, 255), 2);
            }
            cv::putText(
                image,
                std::to_string(armor.id),
                armor_pixel + cv::Point(8, -8),
                cv::FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1);
        }
    }
}

struct TimedArmors {
    rclcpp::Time TimeStamp;
    std::vector<ly_auto_aim::ArmorObject> Armors;
    GimbalAnglesType TimeAngles{};
};

struct AngleFrame{
    cv::Mat image;
    GimbalAnglesType angles;
};

ImageQueue callbackQueue;
void ImageLoop() {
    cv::Mat image;
    cv::Mat sub_image;
    cv::Mat concat_image;
    ImageQueue image_queue;
    boost::lockfree::stack<AngleFrame> angle_image_stack(MAX_STACK_SIZE);

    std::jthread imagepub_thread{[&] {
        rclcpp::WallRate rate(80); // ROS2 推荐使用 WallRate
        while (rclcpp::ok()) {
            if (!image_queue.empty()) {
                cv::Mat publish_image = image_queue.wait_and_pop();
                
                // ROS2 中消息头处理
                std_msgs::msg::Header header;
                header.stamp = global_node->now();
                header.frame_id = "camera";

                auto compressed_msg = cv_bridge::CvImage(header, "bgr8", publish_image)
                                        .toCompressedImageMsg(cv_bridge::JPG);

                // global_node.Publisher 映射到 ROS2 的 publish 调用
                global_node->Publisher<ly_compressed_image>()->publish(*compressed_msg);
                rate.sleep();
            }
        }
    }};

    std::jthread ra_imagepub_thread{[&] {
        rclcpp::WallRate rate(100); 
        while (rclcpp::ok()) {
            if (!angle_image_stack.empty()) {
                AngleFrame angle_frame;
                if (!angle_image_stack.pop(angle_frame)) continue;

                std_msgs::msg::Header header;
                header.stamp = global_node->now();

                auto image_msg = cv_bridge::CvImage(header, "bgr8", angle_frame.image).toImageMsg();
                
                auto_aim_common::msg::AngleImage angle_image_msg;
                angle_image_msg.image = *image_msg;
                angle_image_msg.yaw = angle_frame.angles.yaw;   // 注意 ROS2 字段通常为小写
                angle_image_msg.pitch = angle_frame.angles.pitch;

                global_node->Publisher<ly_ra_angle_image>()->publish(angle_image_msg);

                // ROS2 的 spinOnce 等价物
                rclcpp::spin_some(global_node);
            }
            rate.sleep();
        }
    }};

    std::jthread detect_thread {
        [&] {
            rclcpp::Rate rate(78);
            while (rclcpp::ok()) {
                if(web_show && !concat_image.empty()){
                    VideoStreamer::setFrame(concat_image);
                } else if(web_show && !image.empty()) {
                    VideoStreamer::setFrame(image);
                }
                rate.sleep();

                TimedArmors armors;
                std::vector<ly_auto_aim::CarDetection> cars;
                auto_aim_common::msg::Armors armor_list_msg; // ROS2 增加 ::msg::
                std::vector<ly_auto_aim::ArmorObject> filtered_armors{};
                ly_auto_aim::ArmorObject target_armor;

                if(!use_ros_bag){
                    if (!Cam.GetImage(image)) continue;
                }

                rclcpp::spin_some(global_node);

                if(use_ros_bag){
                    image = callbackQueue.wait_and_pop();
                }
                if (image.empty()) continue;

                if(ra_enable && !image.empty()){
                    GimbalAnglesType temp_angles{
                        gimbal_angles_yaw.load(),
                        gimbal_angles_pitch.load()
                    };
                    angle_image_stack.push({image.clone(), temp_angles});
                }

                armors.TimeAngles.yaw = gimbal_angles_yaw;
                armors.TimeAngles.pitch = gimbal_angles_pitch;

                // ROS2 获取当前时间
                armors.TimeStamp = global_node->now();

                armor_list_msg.armors.clear();
                armor_list_msg.cars.clear();
                armor_list_msg.header.frame_id = "camera";
                armor_list_msg.header.stamp = armors.TimeStamp;
                armor_list_msg.yaw = armors.TimeAngles.yaw;
                armor_list_msg.pitch = armors.TimeAngles.pitch;
                armor_list_msg.is_available_armor_for_predictor = false;
                armor_list_msg.target_armor_index_for_predictor = -1;

                auto publish_result = [&]() {
                    if(aa_enable){
                        global_node->Publisher<ly_detector_armors>()->publish(armor_list_msg);
                    }else if(outpost_enable){
                        global_node->Publisher<ly_outpost_armors>()->publish(armor_list_msg);
                    }
                };

                auto log_detection_summary = [&](const std::size_t raw_count,
                                                 const std::size_t filtered_count,
                                                 const std::size_t car_count,
                                                 const char* stage) {
                    static auto last_log_time = std::chrono::steady_clock::time_point{};
                    const auto log_now = std::chrono::steady_clock::now();
                    if (log_now - last_log_time < std::chrono::seconds(1)) {
                        return;
                    }
                    last_log_time = log_now;
                    RCLCPP_INFO(
                        global_node->get_logger(),
                        "detector summary: stage=%s raw_armors=%zu filtered_armors=%zu cars=%zu target_type=%d aa_enable=%s outpost_enable=%s",
                        stage,
                        raw_count,
                        filtered_count,
                        car_count,
                        static_cast<int>(atomic_target.load()),
                        aa_enable.load() ? "true" : "false",
                        outpost_enable.load() ? "true" : "false");
                };

                armors.Armors.clear();
                
                if (!carAndArmorDetector.Detect(image, armors.Armors, cars)) {
                    log_detection_summary(0, 0, 0, "detect_fail");
                    publish_result();
                    continue;
                }

                filtered_armors.clear();
                filter.is_team_red = myTeamRed;

                if (debug_mode && debug_team_blue) {
                    filter.is_team_red = false;
                } else if( debug_mode && !debug_team_blue){
                    filter.is_team_red = true;
                }

                if(!carFinder.FindCar(cars, armor_list_msg.cars)){
                    RCLCPP_DEBUG(global_node->get_logger(), "car_finder> failed to find car");
                }

                ly_auto_aim::ArmorType target = atomic_target;
                if (!filter.Filter(armors.Armors, target, filtered_armors)) {
                    log_detection_summary(armors.Armors.size(), 0, armor_list_msg.cars.size(), "filter_empty");
                    publish_result();
                    continue;
                }

                if (!finder.ReFindAndSolveAll(*poseSolver, filtered_armors, target, target_armor, armor_list_msg)) {
                    // 以前的 ROS_WARN 替换
                }

                if(draw_image) DrawAllArmor(image, filtered_armors);
                if(draw_image) DrawAllCar(image, cars);
                DrawPredictorCenterOverlay(image, armors.TimeAngles, armors.TimeStamp);
                DrawPredictorFullOverlay(image, armors.TimeAngles, armors.TimeStamp);

                log_detection_summary(
                    armors.Armors.size(),
                    filtered_armors.size(),
                    armor_list_msg.cars.size(),
                    "publish");

                publish_result();
            }
        }
    };


}


#pragma region using image in rosbag
void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
        roslog::error("cv_bridge exception: %s", e.what());
        return;
    }
    // Cam.SetImage(cv_ptr->image);
    callbackQueue.push(cv_ptr->image);
}
#pragma endregion using image in rosbag

void camera_benchmark(std::size_t step = 1000) {
    cv::Mat image;
    auto begin = std::chrono::high_resolution_clock::now();
    for (std::size_t i = 0;i < step; i++){
        Cam.GetImage(image);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto cost = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    auto avg = cost * 1.0f / step;
    roslog::info("benchmark: step({}) cost({}) avg({})", step, cost, avg);
}

} // namespace

int main(int argc, char **argv) try {
    rclcpp::init(argc, argv);

    global_node = std::make_shared<LangYa::ROSNode<AppName>>();
    std::string detector_share_dir;
    try {
        detector_share_dir = ament_index_cpp::get_package_share_directory("detector");
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Cannot find detector package share directory: " << e.what() << std::endl;
        return -1;
    }

    std::string classifier_path;
    std::string detector_path;
    std::string car_model_path;

    InitialParam();
    LoadSolverIntrinsicsParam();
    LoadSolverExtrinsicsParam();
    poseSolver = std::make_unique<ly_auto_aim::PoseSolver>(cameraIntrinsics);

    /// 初始化节点和模块

    if(!use_ros_bag){
        if (use_video) {
            std::cout << "[DEBUG] Opening Video: " << video_path << std::endl;
            if(video_path.empty()){
                std::cerr << "[ERROR] Video path is empty! Check your YAML file." << std::endl;
                return -1;
            }
            if (!Cam.Initialize(video_path)) {
                std::cerr << "[ERROR] Failed to initialize video source: " << video_path << std::endl;
                return -1;
            }
        } else {
            ConfigureCamera();
            if (camera_sn.empty()) {
                std::cerr << "[ERROR] camera_param.camera_sn (or camera_param/camera_sn) is empty." << std::endl;
                return -1;
            }
            if (!Cam.Initialize("", camera_sn)) {
                std::cerr << "[ERROR] Failed to initialize camera with SN: " << camera_sn << std::endl;
                return -1;
            }
        }
    }

    if (use_ros_bag){
        global_node->GenSubscriber<ly_compressed_image>(image_callback);
    } 

    if(web_show){
        VideoStreamer::init();
    }
    
    LoadParamCompat<std::string>({"detector_config.classifier_path", "detector_config/classifier_path"}, classifier_path, "");
    LoadParamCompat<std::string>({"detector_config.detector_path", "detector_config/detector_path"}, detector_path, "");
    LoadParamCompat<std::string>({"detector_config.car_model_path", "detector_config/car_model_path"}, car_model_path, "");

    classifier_path = ResolvePathFromDetectorShare(classifier_path, detector_share_dir);
    detector_path = ResolvePathFromDetectorShare(detector_path, detector_share_dir);
    car_model_path = ResolvePathFromDetectorShare(car_model_path, detector_share_dir);

    auto ensure_path_exists = [](const char* label, const std::string& path) -> bool {
        if (path.empty()) {
            std::cerr << "[ERROR] " << label << " path is empty" << std::endl;
            return false;
        }
        if (!std::filesystem::exists(path)) {
            std::cerr << "[ERROR] " << label << " path does not exist: " << path << std::endl;
            return false;
        }
        return true;
    };

    if (!ensure_path_exists("Classifier", classifier_path) ||
        !ensure_path_exists("Detector", detector_path) ||
        !ensure_path_exists("CarModel", car_model_path)) {
        return -1;
    }
    
    std::cout << "[DEBUG] Loading Classifier: " << classifier_path << std::endl;
    if (!carAndArmorDetector.armorDetector.Corrector.Classifier.LoadModel(classifier_path)) return -1;
    
    std::cout << "[DEBUG] Loading Detector: " << detector_path << std::endl;
    if (!carAndArmorDetector.armorDetector.Detector.LoadModel(detector_path)) return -1;

    std::cout << "[DEBUG] Loading CarModel: " << car_model_path << std::endl;
    // 這裡加上 try-catch 因為 yolo_detector 內部沒有保護
    try {
        if (!carAndArmorDetector.carDetector.LoadModel(car_model_path)) return -1;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] CarModel Crash: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "[ERROR] CarModel Unknown Crash" << std::endl;
        return -1;
    }

    std::cout << "[DEBUG] All Models Loaded Successfully!" << std::endl;

    global_node->GenSubscriber<ly_me_is_team_red>(my_team_callback);
    global_node->GenSubscriber<ly_bt_target>(get_target_callback);
    global_node->GenSubscriber<ly_gimbal_angles>(gimbal_callback);
    global_node->GenSubscriber<ly_aa_enable>(aa_enable_callback);
    // [修復] 補上缺失的訂閱者，否則 ra_enable/outpost_enable 永遠是 false
    global_node->GenSubscriber<ly_ra_enable>(ra_enable_callback);
    global_node->GenSubscriber<ly_outpost_enable>(outpost_enable_callback);
    global_node->GenSubscriber<ly_predictor_debug>(predictor_debug_callback);
    global_node->GenSubscriber<ly_predictor_vis>(predictor_vis_callback);

    // 進入圖像循環
    ImageLoop();

    if (web_show) {
        VideoStreamer::cleanup();
    }

    rclcpp::shutdown();
    return 0;
}
catch (const std::exception &e) {
    std::cerr << "Error in main: " << e.what() << std::endl;
    return 1;
}
