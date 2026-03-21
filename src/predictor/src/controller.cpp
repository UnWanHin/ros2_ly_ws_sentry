// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "controller/controller.hpp"
#include <cmath>
#include <Logger/Logger.hpp>
#include <fmt/format.h>

// [ROS 2] 定義全局變量
namespace ly_auto_aim::controller {
    rclcpp::Node::SharedPtr global_controller_node = nullptr;
}

// [ROS 2] 定義本地適配器，避免與 Logger.hpp 衝突
namespace roslog_fmt {
    static rclcpp::Logger get_logger() {
        if (ly_auto_aim::controller::global_controller_node) {
            return ly_auto_aim::controller::global_controller_node->get_logger();
        }
        return rclcpp::get_logger("controller");
    }
    template <typename... Args>
    void debug(const char* format_str, const Args&... args) {
        RCLCPP_DEBUG(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void warn(const char* format_str, const Args&... args) {
        RCLCPP_WARN(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void info(const char* format_str, const Args&... args) {
        RCLCPP_INFO(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void error(const char* format_str, const Args&... args) {
        RCLCPP_ERROR(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
}
// [ROS 2] 宏替換：讓文件內的 roslog 調用指向我們定義的 roslog_fmt
#define roslog roslog_fmt

auto ly_auto_aim::controller::createController() -> std::shared_ptr<ly_auto_aim::controller::Controller>
{
    return std::make_unique<ly_auto_aim::controller::Controller>();
}

using namespace ly_auto_aim::controller;

void Controller::registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc)
{
    this->predictFunc = predictFunc;
}

// [ROS 2] 構造函數
Controller::Controller() {
    if (!global_controller_node) {
        std::cerr << "Controller Error: global_controller_node is NULL" << std::endl;
        return;
    }
    auto node = global_controller_node;
    std::string ns = "controller_config.";

    if (!node->has_parameter(ns + "bullet_speed")) node->declare_parameter(ns + "bullet_speed", 23.0);
    if (!node->get_parameter(ns + "bullet_speed", bullet_speed)) {
        roslog::warn("Bullet speed not set, using default value");
        bullet_speed = 23.0;
    }

    double shoot_delay_val = 0.1;
    if (!node->has_parameter(ns + "shoot_delay")) node->declare_parameter(ns + "shoot_delay", 0.1);
    if (!node->get_parameter(ns + "shoot_delay", shoot_delay_val)) {
        roslog::warn("Shoot delay not set, using default value");
        shoot_delay_val = 0.1;
    }
    shootDelay = rclcpp::Duration::from_seconds(shoot_delay_val);
    
    // 读取射击表相关参数
    loadShootTableParams(node);
}

// [ROS 2] 參數類型適配
void Controller::loadShootTableParams(rclcpp::Node::SharedPtr node)
{
    std::string ns = "controller_config.";
    std::string table_ns = ns + "shoot_table_adjust.";

    // 读取射击表调整开关
    if (!node->has_parameter(table_ns + "enable")) node->declare_parameter(table_ns + "enable", false);
    if(!node->get_parameter(table_ns + "enable", shoot_table_adjust)) {
        roslog::warn("Shoot table adjust not set, using default value false");
        shoot_table_adjust = false;
    }
    
    if(shoot_table_adjust) {
        // 初始化参数向量
        pitch_param.resize(6, 0.0);
        yaw_param.resize(6, 0.0);
        
        auto load_param = [&](const std::string& key, double& val) {
            if (!node->has_parameter(key)) node->declare_parameter(key, 0.0);
            node->get_parameter(key, val);
        };

        // 读取pitch参数
        load_param(table_ns + "pitch.intercept", pitch_param[0]);
        load_param(table_ns + "pitch.coef_z", pitch_param[1]);
        load_param(table_ns + "pitch.coef_d", pitch_param[2]);
        load_param(table_ns + "pitch.coef_z2", pitch_param[3]);
        load_param(table_ns + "pitch.coef_zd", pitch_param[4]);
        load_param(table_ns + "pitch.coef_d2", pitch_param[5]);
        
        // 读取yaw参数
        load_param(table_ns + "yaw.intercept", yaw_param[0]);
        load_param(table_ns + "yaw.coef_z", yaw_param[1]);
        load_param(table_ns + "yaw.coef_d", yaw_param[2]);
        load_param(table_ns + "yaw.coef_z2", yaw_param[3]);
        load_param(table_ns + "yaw.coef_zd", yaw_param[4]);
        load_param(table_ns + "yaw.coef_d2", yaw_param[5]);
        
        roslog::info("Shoot table adjustment enabled");
        roslog::info("Pitch params: intercept={}, coef_z={}, coef_d={}", 
                    pitch_param[0], pitch_param[1], pitch_param[2]);
        roslog::info("Yaw params: intercept={}, coef_z={}, coef_d={}", 
                    yaw_param[0], yaw_param[1], yaw_param[2]);
    } else {
        roslog::info("Shoot table adjustment disabled");
    }
}

double Controller::fitPitch(double z_height, double horizontal_distance) const
{
    if (!shoot_table_adjust || pitch_param.size() < 6) {
        return 0.0;
    }
    
    // Model 3: Full 2nd Order 参数 (PITCH)
    double intercept = pitch_param[0];
    double coef_z = pitch_param[1];
    double coef_d = pitch_param[2];
    double coef_z2 = pitch_param[3];
    double coef_zd = pitch_param[4];
    double coef_d2 = pitch_param[5];

    // 计算特征值
    double z2 = z_height * z_height;
    double d2 = horizontal_distance * horizontal_distance;
    double zd = z_height * horizontal_distance;
    
    // 完整二阶多项式: intercept + z + d + z² + z*d + d²
    return intercept + 
           coef_z * z_height + 
           coef_d * horizontal_distance + 
           coef_z2 * z2 + 
           coef_zd * zd + 
           coef_d2 * d2;
}

double Controller::fitYaw(double z_height, double horizontal_distance) const
{
    if (!shoot_table_adjust || yaw_param.size() < 6) {
        return 0.0;
    }
    
    // Model 3: Full 2nd Order 参数 (YAW)
    double intercept = yaw_param[0];
    double coef_z = yaw_param[1];
    double coef_d = yaw_param[2];
    double coef_z2 = yaw_param[3];
    double coef_zd = yaw_param[4];
    double coef_d2 = yaw_param[5];

    // 计算特征值
    double z2 = z_height * z_height;
    double d2 = horizontal_distance * horizontal_distance;
    double zd = z_height * horizontal_distance;

    // 完整二阶多项式: intercept + z + d + z² + z*d + d²
    return intercept + 
           coef_z * z_height + 
           coef_d * horizontal_distance + 
           coef_z2 * z2 + 
           coef_zd * zd + 
           coef_d2 * d2;
}

bool Controller::calcPitchYawWithShootTable(double& pitch, double& yaw, double& time, 
                                           double target_x, double target_y, double target_z)
{
    // 先进行基础弹道计算
    bool success = calcPitchYaw(pitch, yaw, time, target_x, target_y, target_z);
    
    if (success && shoot_table_adjust) {
        // 计算水平距离和高度
        double horizontal_distance = std::sqrt(target_x * target_x + target_y * target_y);
        
        // 应用射击表补偿（以弧度为单位）
        double pitch_compensation = fitPitch(target_z, horizontal_distance) * PI / 180.0;
        double yaw_compensation = fitYaw(target_z, horizontal_distance) * PI / 180.0;
        
        pitch += pitch_compensation;
        yaw += yaw_compensation;
        
        // roslog::info("Applied shoot table compensation - pitch: {} rad, yaw: {} rad",
        //             pitch_compensation, yaw_compensation);
    }
    
    return success;
}

const int camera_halfwidth=640;
const int camera_halfheight=512;

static bool thetaInRange(double theta_deg, double range_deg)
{
    theta_deg = std::remainder(theta_deg, 360.0);
    if(std::abs(theta_deg) < range_deg)
        return true;
    else
        return false;
} 

ControlResult Controller::control(const GimbalAngleType& gimbal_angle, int target, float bullet_speed)
{
    ControlResult result;
    result.yaw_setpoint = gimbal_angle.yaw;
    result.pitch_setpoint = gimbal_angle.pitch;
    result.pitch_actual_want = gimbal_angle.pitch;
    result.yaw_actual_want = gimbal_angle.yaw;
    result.valid = false;
    result.shoot_flag = false;
    result.invalid_reason = ControlInvalidReason::None;

    // [ROS 2] 時間適配
    if (!global_controller_node) return result;
    rclcpp::Time now = global_controller_node->now();

    Predictions predictions_for_time = predictFunc(now + flyTime + shootDelay);
    if (predictions_for_time.empty())
    {
        result.invalid_reason = ControlInvalidReason::NoPrediction;
        roslog::debug("No prediction");
        return result;
    }
    bool is_valid_car_id = std::any_of(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (!is_valid_car_id)
    {
        roslog::debug("Invalid car id");

        bool found = false;
        double min_distance = std::numeric_limits<double>::max();
        for(const auto& prediction : predictions_for_time){
            if(prediction.id == target){
                aim_armor_id.first = target;
                aim_armor_id.second = -1;
                found = true;
            } 
        }
        if(!found)
        {
            for (const auto& prediction : predictions_for_time)
            {
                double distance = sqrt(prediction.center.x * prediction.center.x + prediction.center.y * prediction.center.y);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    aim_armor_id.first = prediction.id;
                    aim_armor_id.second = -1;
                    found = true;
                }
            }
        }
    }
    auto it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (it == predictions_for_time.end())
    {
        result.invalid_reason = ControlInvalidReason::InvalidCar;
        roslog::debug("New car id invalid(shouldn't happen)");
        return result;
    }
    //calc new flytime
    double distance = sqrt(it->center.x * it->center.x + it->center.y * it->center.y);
    if (distance > 0.0)
    {
        constexpr float stable_bullet_speed = 23.0f;
        if(bullet_speed - stable_bullet_speed < 1.0f && bullet_speed - stable_bullet_speed > -1.0f){
            this->bullet_speed = stable_bullet_speed;
        }
        this->bullet_speed = stable_bullet_speed;
        double calculated_time_sec = distance / this->bullet_speed;
        
        // [ROS 2] Duration 修復
        flyTime = rclcpp::Duration::from_seconds(calculated_time_sec);
        
        predictions_for_time = predictFunc(now + flyTime + shootDelay);
    }
    it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (it == predictions_for_time.end())
    {
        result.invalid_reason = ControlInvalidReason::InvalidCarAfterFlyTime;
        roslog::debug("New car id invalid after flytime update");
        return result;
    }
    bool is_valid_armor_id = std::any_of(it->armors.begin(), it->armors.end(),
        [&](const auto& armor) { return (armor.id == aim_armor_id.second) && (armor.status == Armor::AVAILABLE); });
    if (!is_valid_armor_id)
    {
        roslog::debug("Invalid armor id");
        //重新选择一个距离最近的
        double min_distance = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto& armor : it->armors)
        {
            if (armor.status == Armor::AVAILABLE)
            {
                double distance = sqrt(armor.center.x * armor.center.x + armor.center.y * armor.center.y);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    aim_armor_id.second = armor.id;
                    found = true;
                }
            }
        }
        if (!found)
        {
            roslog::debug("No available armor, fallback to car center");
            double pitch = 0.0;
            double yaw = 0.0;
            double time = 0.0;
            // 使用带射击表补偿的弹道计算
            bool success = calcPitchYawWithShootTable(pitch, yaw, time, 
                                                     it->center.x + x_offset, 
                                                     it->center.y + y_offset, 
                                                     it->center.z + z_offset);
            if (!success)
            {
                result.invalid_reason = ControlInvalidReason::NoArmorFallbackBallisticFail;
                roslog::warn("calcPitchYawWithShootTable failed");
                return result;
            }
            result.pitch_setpoint = pitch * 180 / PI;
            result.yaw_setpoint = yaw * 180 / PI;
            result.yaw_setpoint = result.yaw_setpoint + std::round((gimbal_angle.yaw - result.yaw_setpoint) / 360.0f) * 360.0f;
            result.pitch_actual_want = result.pitch_setpoint;
            result.yaw_actual_want = result.yaw_setpoint;
            result.valid = true;
            result.shoot_flag = false;
            // [ROS 2] Duration 修復
            flyTime = rclcpp::Duration::from_seconds(time);
            return result;
        }
    }
    auto armor_it = std::find_if(it->armors.begin(), it->armors.end(),
        [&](const auto& armor) { return armor.id == aim_armor_id.second; });
    if (armor_it == it->armors.end())
    {
        result.invalid_reason = ControlInvalidReason::InvalidArmor;
        roslog::debug("Invalid armor id (shoudln't happen)");
        return result;
    }
    //should calc new time
    //but now we just use the old time
    double pitch = 0.0;
    double yaw = 0.0;
    double time = 0.0;
    // 使用带射击表补偿的弹道计算
    if (!calcPitchYawWithShootTable(pitch, yaw, time, armor_it->center.x, armor_it->center.y, armor_it->center.z))
    {
        result.invalid_reason = ControlInvalidReason::ArmorBallisticFail;
        roslog::warn("calcPitchYawWithShootTable failed");
        return result;
    }
    result.pitch_setpoint = pitch * 180 / PI;
    result.yaw_setpoint = yaw * 180 / PI;
    result.yaw_setpoint = result.yaw_setpoint + std::round((gimbal_angle.yaw - result.yaw_setpoint) / 360.0f) * 360.0f;
    result.pitch_actual_want = result.pitch_setpoint;
    result.yaw_actual_want = result.yaw_setpoint;
    result.valid = true;
    float yaw_diff = std::remainder(result.yaw_actual_want - gimbal_angle.yaw, 360.0f);
    const bool unstable_track = !it->stable;
    if (unstable_track) {
        roslog::debug(
            "Control degraded but allowed: car_id={}, armor_id={}, stable={}, yaw_diff_deg={}",
            aim_armor_id.first, aim_armor_id.second, it->stable, yaw_diff);
    } else {
        result.invalid_reason = ControlInvalidReason::None;
        roslog::debug(
            "Control valid: car_id={}, armor_id={}, yaw_diff_deg={}",
            aim_armor_id.first, aim_armor_id.second, yaw_diff);
    }
    result.invalid_reason = ControlInvalidReason::None;
    if(result.pitch_setpoint < -2.0f ) result.pitch_actual_want -= 1.0f; /// 高打低偏置补丁

    return result;
}

bool Controller::judgeAimNew(bool request)
{
    aim_new = false;
    if(((!aiming) && request) || (aiming && (!request)))
    {
        accumulate_aim_request++;
    }
    else
    {
        accumulate_aim_request = 0;
    }
    if(accumulate_aim_request > waitFrame)
    {
        accumulate_aim_request = 0;
        aiming = request;
        if(aiming)
            aim_new = true;
    }
    return aim_new;
}

bool Controller::calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z)
{
    double distance = sqrt(target_x * target_x + target_y * target_y);
    double theta = pitch;
    double delta_z = 0.0;
    // 首先计算空气阻力系数 K
    double k1 = C_D * RHO * (PI * bullet_diameter * bullet_diameter) / 8 / bullet_mass;
    for (int i = 0; i < max_iter; i++)
    {
        // 计算炮弹的飞行时间
        double t = (exp(k1 * distance) - 1) / (k1 * bullet_speed * cos(theta));

        delta_z = target_z - bullet_speed * sin(theta) * t / cos(theta) + 0.5 * GRAVITY * t * t / cos(theta) / cos(theta);

        // 不断更新theta，直到小于某一个阈值
        if (fabs(delta_z) < tol)
        {
            time = t;
            break;
        }

        // 更新角度
        theta -= delta_z / (-(bullet_speed * t) / pow(cos(theta), 2) + GRAVITY * t * t / (bullet_speed * bullet_speed) * sin(theta) / pow(cos(theta), 3));
    }
    if(fabs(delta_z) > tol)
    {
        //不更新pitch和yaw
        roslog::warn("calcPitchYaw failed");
        return false;//计算失败
    }
    else
    {
        pitch = theta;
        yaw = atan2(target_y, target_x);
        return true;
    }
}
