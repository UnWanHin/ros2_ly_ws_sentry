// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

/*
 * buff_hitter_node（打符）
 *
 * 功能：
 * - 输入视觉角度图像/云台状态，计算打符目标角度
 * - 输出 /ly/buff/target（供上层决策或外部桥接）
 *
 * 说明：
 * - 本节点默认不直接写 /ly/control/*，由上层决策决定是否接管火控。
 */
#include "module/RosTools.hpp"
#include "module/BasicTypes.hpp"
#include "module/BuffCalculator.hpp"
#include "module/BuffController.hpp"
#include "module/BuffDetector.hpp"
#include "module/Timer.hpp"

// auto_aim_common - 根据实际文件后缀决定
#include "auto_aim_common/msg/armor.hpp"
#include "auto_aim_common/msg/armors.hpp"
#include "auto_aim_common/msg/target.hpp"
#include "auto_aim_common/msg/angle_image.hpp"

// gimbal_driver - 去掉重复，根据实际文件后缀决定
#include "gimbal_driver/msg/gimbal_angles.hpp"
#include "gimbal_driver/msg/uwb_pos.hpp"
#include "gimbal_driver/msg/vel.hpp"
#include "gimbal_driver/msg/health.hpp"
#include "gimbal_driver/msg/game_data.hpp"
#include "gimbal_driver/msg/buff_data.hpp"
#include "gimbal_driver/msg/position_data.hpp"

// std_msgs - 全小写，无下划线，ROS 2 标准是 .hpp
#include <std_msgs/msg/float32.hpp>
// #include <std_msgs/msg/builtin_uint8.hpp>  // 如果这个文件存在
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>

// 其他
#include <cv_bridge/cv_bridge.h>  // ROS 2 中是 .hpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int32.hpp>  // 添加这个
#include <sensor_msgs/image_encodings.hpp>

// C++ 标准库
#include <deque>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>

// TODO copy to auto aim detector
LY_DEF_ROS_TOPIC(ly_aa_enable, "/ly/aa/enable", std_msgs::msg::Bool);

// rune aim
LY_DEF_ROS_TOPIC(ly_ra_enable, "/ly/ra/enable", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_ra_mode, "/ly/ra/mode", std_msgs::msg::UInt8);

// LY_DEF_ROS_TOPIC(ly_camera_image, "/ly/camera/image", sensor_msgs::msg::Image);
LY_DEF_ROS_TOPIC(ly_ra_image, "/ly/ra/image", sensor_msgs::msg::Image);
LY_DEF_ROS_TOPIC(ly_compressed_image, "/ly/compressed/image", sensor_msgs::msg::CompressedImage);

LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
LY_DEF_ROS_TOPIC(ly_gimbal_firecode, "/ly/gimbal/firecode", std_msgs::msg::UInt8);
LY_DEF_ROS_TOPIC(ly_bullet_speed, "/ly/bullet/speed", std_msgs::msg::Float32);

LY_DEF_ROS_TOPIC(ly_buff_target, "/ly/buff/target", auto_aim_common::msg::Target)

LY_DEF_ROS_TOPIC(ly_ra_angle_image, "/ly/ra/angle_image", auto_aim_common::msg::AngleImage);

// LY_DEF_ROS_TOPIC(ly_control_angles, "/ly/control/angles", gimbal_driver::msg::GimbalAngles);
// LY_DEF_ROS_TOPIC(ly_control_firecode, "/ly/control/firecode", std_msgs::msg::UInt8);

template<typename T, std::size_t MaxN = 10>
struct SimpleAvg {
    T _s;
    std::queue<T> _q;

    T GetSum() {
        return _s;
    }

    T GetCount() {
        return _q.size();
    }

    T GetAvg() {
        return static_cast<T>(GetSum() / GetCount());
    }

    /// @brief 添加新元素，获取平均值
    T Add(const T v) {
        if (_q.size() >= MaxN) {
            _s -= _q.front();
            _q.pop();
        }
        _s += v;
        _q.push(v);
        return GetAvg();
    }
};

struct RA_MultiThreadVariables {
    bool Enable{true};
    cv::Mat Image{};
    bool buffHitterShoot{};
    bool buff_follow{};
    // LangYa::FireCodeType FireCode{};
    gimbal_driver::msg::GimbalAngles GimbalAngles{};
};

using namespace LangYa;
using namespace power_rune;

class Application {
public:
    inline static constexpr const char Name[] = "ra_hitter";

    bool udp_enable ;
    bool web_debug_enable ;
    bool shoot_enable ;
    bool record_enable ;
    bool force_shoot ;
    bool buff_success = false;
    // 新增共享变量
    std::shared_ptr<float> buff_pitch = std::make_shared<float>(0.0);
    std::shared_ptr<float> buff_yaw = std::make_shared<float>(0.0);
    std::shared_ptr<BuffDetector> buff_detector_ptr;  //同下
    std::shared_ptr<BuffCalculator> buff_calculator_ptr;   //属于solver 越級了, 后面加新模塊buff再移
    BuffController buff_controller;

    //todo max_count 越大，相邻两次打弹之间时间间隔越久，保证不会同一片扇叶打两次就行
#define MAX_COUNT 25
    Timer timer{ MAX_COUNT };
#undef MAX_COUNT

    auto& buff_detector() {
        return *buff_detector_ptr;
    }

    auto& buff_calculator() {
        return *buff_calculator_ptr;
    }

private:
    ROSNode<Name> Node{};
    MultiCallback<RA_MultiThreadVariables> CallbackGenerator;
    bool aa_enable{true};
    std::atomic<float> latest_bullet_speed_mps_{22.9f};
    double filtered_bullet_speed_mps_{22.9};
    double min_bullet_speed_mps_{20.0};
    double bullet_speed_alpha_{0.5};
    bool dynamic_bullet_speed_enable_{true};
    bool has_filtered_bullet_speed_{true};
    int enemy_color_{0};  // 0 red, 1 blue
    int default_buff_mode_{0};  // 0 auto, 1 small, 2 big
    int buff_mode_{1};          // active mode
    std::atomic<int> requested_buff_mode_{0};  // 0: follow default, 1: small, 2: big
    bool mode_switch_topic_enable_{true};
    bool reload_big_buff_{true};
    bool two_target_enable_{true};
    double two_target_cycle_timeout_sec_{1.0};
    bool dual_target_active_{false};
    bool switch_to_second_{false};
    bool prev_shoot_cmd_{false};
    int primary_target_idx_{0};
    int secondary_target_idx_{1};
    int selected_target_idx_{0};
    std::chrono::steady_clock::time_point dual_cycle_start_{std::chrono::steady_clock::now()};

    void bullet_speed_callback(const std_msgs::msg::Float32::ConstSharedPtr msg) {
        if (!msg) return;
        if (std::isfinite(msg->data)) {
            latest_bullet_speed_mps_.store(msg->data, std::memory_order_relaxed);
        }
    }

    void ra_mode_callback(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
        if (!msg) return;
        const int mode = static_cast<int>(msg->data);
        if (mode >= 0 && mode <= 2) {
            requested_buff_mode_.store(mode, std::memory_order_relaxed);
        }
    }

    int resolve_buff_mode() const {
        if (!mode_switch_topic_enable_) {
            return default_buff_mode_;
        }
        const int requested = requested_buff_mode_.load(std::memory_order_relaxed);
        if (requested == 1 || requested == 2) {
            return requested;
        }
        return default_buff_mode_;
    }

    float get_buff_solver_bullet_speed() {
        const double raw_speed = static_cast<double>(
            latest_bullet_speed_mps_.load(std::memory_order_relaxed));
        if (dynamic_bullet_speed_enable_ && std::isfinite(raw_speed) && raw_speed > min_bullet_speed_mps_) {
            if (!has_filtered_bullet_speed_) {
                filtered_bullet_speed_mps_ = raw_speed;
                has_filtered_bullet_speed_ = true;
            } else {
                filtered_bullet_speed_mps_ =
                    bullet_speed_alpha_ * raw_speed + (1.0 - bullet_speed_alpha_) * filtered_bullet_speed_mps_;
            }
        }
        const double safe_speed = std::max(filtered_bullet_speed_mps_, min_bullet_speed_mps_);
        return static_cast<float>(safe_speed);
    }

    void aa_enable_callback(const std_msgs::msg::Bool& msg) {
        aa_enable = msg.data; //ROS1 aa_enable = msg->data;
        roslog::info("aa_enable: {}", aa_enable);
    }

    template<typename TTopic>
    void GenSub(auto modifier, std::size_t queueSize = 3) {
        // for topic, generate a subscriber
        // when target messages come in, use that message as argument for modifier
        Node.GenSubscriber<TTopic>(CallbackGenerator.Generate<TTopic>(modifier)); //, queueSize);
    }

    void GenSubs() {
        // GenSub<ly_gimbal_angles>([](RA_MultiThreadVariables &g, const gimbal_driver::msg::GimbalAngles &m) {
        //     g.GimbalAngles.Yaw = static_cast<float>(m.Yaw);
        //     g.GimbalAngles.Pitch = static_cast<float>(m.Pitch);
        // });
        GenSub<ly_ra_enable>([](RA_MultiThreadVariables &g, const std_msgs::msg::Bool &m) {
            roslog::info("received messasge: {}", m.data);
             g.Enable = m.data; 
            });
        // GenSub<ly_gimbal_firecode>([](RA_MultiThreadVariables &g, const std_msgs::msg::UInt8 &m) {
        //     *reinterpret_cast<std::uint8_t *>(&g.FireCode) = m.data;
        // });
        // GenSub<ly_ra_image>(
        //     [](RA_MultiThreadVariables &g, const sensor_msgs::msg::Image &m) {
        //         static SimpleAvg<double, 20> delay_avg{};
        //         g.Image = cv_bridge::toCvCopy(m, sensor_msgs::msg::Image_encodings::BGR8)->image;
        //         const auto now = ros::Time::now().toSec() * 1000;
        //         const auto tar = m.header.stamp.toSec() * 1000;
        //         const auto delay = now - tar;
                
        //         roslog::warn("image delay:{} ms", delay_avg.Add(delay));
        //     }, 1);
        GenSub<ly_ra_angle_image>(
            [](RA_MultiThreadVariables &g, const auto_aim_common::msg::AngleImage &m) {
                g.GimbalAngles.pitch = static_cast<float>(m.pitch); //g.GimbalAngles.Pitch = static_cast<float>(m.Pitch);
                g.GimbalAngles.yaw = static_cast<float>(m.yaw);
                // 這個是老的ros1命名空間 g.Image = cv_bridge::toCvCopy(m.image, sensor_msgs::msg::Image_encodings::BGR8)->image;
                g.Image = cv_bridge::toCvCopy(m.image, sensor_msgs::image_encodings::BGR8)->image;
                // static SimpleAvg<double, 20> delay_avg{};
                // const auto now = ros::Time::now().toSec() * 1000;
                // const auto tar = m.image.header.stamp.toSec() * 1000;
                // const auto delay = now - tar;
                // roslog::warn("angle image delay:{} ms", delay_avg.Add(delay));
            }, 1);
    }

    void PubData(const bool& hitBuff, const gimbal_driver::msg::GimbalAngles &angles) {
        {
            using topic = ly_buff_target;
            topic::Msg msg;
            msg.status = hitBuff; // true for hit, 0 for miss
            msg.yaw = static_cast<float>(angles.yaw);
            msg.pitch = static_cast<float>(angles.pitch);
            Node.Publisher<topic>()->publish(msg);
        }
        // {
        //     using topic = ly_control_firecode;
        //     topic::Msg msg;
        //     msg.data = *reinterpret_cast<const std::uint8_t *>(&firecode);
        //     Node.Publisher<topic>().publish(msg);
        // }
    }

public:
    Application() noexcept :
        CallbackGenerator{[this](const auto &data) {
        }} { }

    void Init(const auto str) {
        param::Param param(str);
        param = param[param["car_name"].String()];
        udp_enable = param["UDP"]["enable"].Bool();
        web_debug_enable = param["web_debug"].Bool();
        shoot_enable = param["shoot_enable"].Bool();
        record_enable = param["record_enable"].Bool();
        force_shoot = param["force_shoot"].Bool();
        roslog::info("vars: {} {} {} {} {} ", 
            udp_enable, web_debug_enable, shoot_enable, record_enable, force_shoot);
        const auto buff_param = param["buff"];
        if (buff_param.exists("dynamic_bullet_speed_enable")) {
            dynamic_bullet_speed_enable_ = buff_param["dynamic_bullet_speed_enable"].Bool();
        }
        if (buff_param.exists("min_bullet_speed")) {
            min_bullet_speed_mps_ = buff_param["min_bullet_speed"].Double();
        }
        if (buff_param.exists("bullet_speed_alpha")) {
            bullet_speed_alpha_ = buff_param["bullet_speed_alpha"].Double();
        }
        double default_bullet_speed_mps = 22.9;
        if (param.exists("controller") && param["controller"].exists("bullet_speed")) {
            default_bullet_speed_mps = param["controller"]["bullet_speed"].Double();
        }
        if (buff_param.exists("default_bullet_speed")) {
            default_bullet_speed_mps = buff_param["default_bullet_speed"].Double();
        }
        if (!std::isfinite(default_bullet_speed_mps) || default_bullet_speed_mps <= 0.0) {
            default_bullet_speed_mps = 22.9;
        }
        if (!std::isfinite(min_bullet_speed_mps_) || min_bullet_speed_mps_ < 0.0) {
            min_bullet_speed_mps_ = 20.0;
        }
        if (!std::isfinite(bullet_speed_alpha_)) {
            bullet_speed_alpha_ = 0.5;
        }
        bullet_speed_alpha_ = std::clamp(bullet_speed_alpha_, 0.0, 1.0);
        latest_bullet_speed_mps_.store(static_cast<float>(default_bullet_speed_mps), std::memory_order_relaxed);
        filtered_bullet_speed_mps_ = std::max(default_bullet_speed_mps, min_bullet_speed_mps_);
        has_filtered_bullet_speed_ = true;
        roslog::info(
            "buff bullet speed cfg: dynamic={}, default={} min={} alpha={}",
            dynamic_bullet_speed_enable_, filtered_bullet_speed_mps_, min_bullet_speed_mps_, bullet_speed_alpha_);

        if (param.exists("enemy_color")) {
            const auto enemy_color = param["enemy_color"].String();
            if (enemy_color == "blue") {
                enemy_color_ = 1;
            } else {
                enemy_color_ = 0;
            }
        }

        if (buff_param.exists("default_mode")) {
            default_buff_mode_ = buff_param["default_mode"].Int();
        }
        if (default_buff_mode_ != 0 && default_buff_mode_ != 1 && default_buff_mode_ != 2) {
            default_buff_mode_ = 0;
        }
        buff_mode_ = default_buff_mode_;
        if (buff_param.exists("mode_switch_topic_enable")) {
            mode_switch_topic_enable_ = buff_param["mode_switch_topic_enable"].Bool();
        }
        if (buff_param.exists("reload_big_buff")) {
            reload_big_buff_ = buff_param["reload_big_buff"].Bool();
        }
        if (buff_param.exists("two_target_enable")) {
            two_target_enable_ = buff_param["two_target_enable"].Bool();
        }
        if (buff_param.exists("two_target_cycle_timeout_sec")) {
            two_target_cycle_timeout_sec_ = buff_param["two_target_cycle_timeout_sec"].Double();
        }
        if (!std::isfinite(two_target_cycle_timeout_sec_) || two_target_cycle_timeout_sec_ < 0.1) {
            two_target_cycle_timeout_sec_ = 1.0;
        }

        const auto resolve_model_path = [](const std::string& input_path) {
            std::filesystem::path model_path(input_path);
            if (!model_path.empty() && model_path.is_relative() && !std::filesystem::exists(model_path)) {
                const auto pkg_share = std::filesystem::path(
                    ament_index_cpp::get_package_share_directory("buff_hitter"));
                const auto ws_root = pkg_share.parent_path().parent_path().parent_path().parent_path();
                const auto candidate = ws_root / model_path;
                if (std::filesystem::exists(candidate)) {
                    model_path = candidate;
                }
            }
            return model_path.string();
        };

        std::string red_buff_model_path;
        std::string blue_buff_model_path;
        if (buff_param.exists("red_buff_model_path")) {
            red_buff_model_path = buff_param["red_buff_model_path"].String();
        } else if (buff_param.exists("buff_model_path")) {
            red_buff_model_path = buff_param["buff_model_path"].String();
        }
        if (buff_param.exists("blue_buff_model_path")) {
            blue_buff_model_path = buff_param["blue_buff_model_path"].String();
        } else if (buff_param.exists("buff_model_path")) {
            blue_buff_model_path = buff_param["buff_model_path"].String();
        }
        red_buff_model_path = resolve_model_path(red_buff_model_path);
        blue_buff_model_path = resolve_model_path(blue_buff_model_path);
        roslog::info("buff model path: red={}, blue={}", red_buff_model_path, blue_buff_model_path);
        roslog::info(
            "buff mode cfg: enemy_color={} default_mode={} topic_switch={} reload_big_buff={} two_target_enable={} two_target_timeout={}",
            enemy_color_, default_buff_mode_, mode_switch_topic_enable_, reload_big_buff_,
            two_target_enable_, two_target_cycle_timeout_sec_);

        buff_detector_ptr = std::make_shared<BuffDetector>(red_buff_model_path, blue_buff_model_path);
        buff_calculator_ptr = std::make_shared<BuffCalculator>(param);   //属于solver 越級了, 后面加新模塊buff再移
    }

    void Run(int argc, char **argv) {
    GenSubs();
    Node.GenSubscriber<ly_aa_enable>([this](const std_msgs::msg::Bool& msg){ aa_enable_callback(msg); });
    Node.GenSubscriber<ly_ra_mode>(
        [this](const std_msgs::msg::UInt8::ConstSharedPtr msg) { ra_mode_callback(msg); });
    Node.GenSubscriber<ly_bullet_speed>(
        [this](const std_msgs::msg::Float32::ConstSharedPtr msg) { bullet_speed_callback(msg); });
    
    using namespace std::chrono_literals;
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(Node.get_node_base_interface());  // ✅ 加入這行
        
        { // before work validation
            /// 假如辅瞄和打符都启用，默认禁用打符
            if(aa_enable) {
                std::this_thread::sleep_for(10ms);  // 避免 busy loop
                continue;
            }

            const auto global_var_copy = CallbackGenerator.GetCopy();
            if (!global_var_copy.Enable) {
                std::this_thread::sleep_for(5s);
                continue;
            }
        }

        auto last_var = CallbackGenerator.GetCopy();
        // roslog::warn("in main1");
        const auto& image = last_var.Image;

        if(image.empty()) {
            std::this_thread::sleep_for(10ms);  // 避免 busy loop
            continue;
        }

        // Do your work here
        buff_success = false;
        // roslog::warn("PITCH_NOW:{} YAW_NOW:{} ",last_var.GimbalAngles.pitch, last_var.GimbalAngles.yaw); 
        
        if (buff_detector().buffDetect(image, enemy_color_) == false)
        {
            roslog::error("buff_detector fail");
            dual_target_active_ = false;
            switch_to_second_ = false;
            prev_shoot_cmd_ = false;
            selected_target_idx_ = 0;
            buff_success = false;
        }
        else
        {
            // roslog::warn("buff_detector success!!");
            const int detected_target_count = buff_detector().getDetectedArmorCount();
            bool can_solve = true;
            if (detected_target_count <= 0) {
                dual_target_active_ = false;
                switch_to_second_ = false;
                prev_shoot_cmd_ = false;
                selected_target_idx_ = 0;
                buff_success = false;
                roslog::warn("buff_detector found no valid target");
                can_solve = false;
            }

            if (can_solve) {
                if (!two_target_enable_ || detected_target_count == 1) {
                    selected_target_idx_ = 0;
                    dual_target_active_ = false;
                    switch_to_second_ = false;
                } else {
                    const auto now_tp = std::chrono::steady_clock::now();
                    if (!dual_target_active_) {
                        dual_target_active_ = true;
                        switch_to_second_ = false;
                        prev_shoot_cmd_ = false;
                        dual_cycle_start_ = now_tp;
                        primary_target_idx_ = 0;
                        secondary_target_idx_ = 1;
                    }
                    const double elapsed_sec =
                        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - dual_cycle_start_).count() /
                        1000.0;
                    if (elapsed_sec > two_target_cycle_timeout_sec_) {
                        dual_cycle_start_ = now_tp;
                        switch_to_second_ = false;
                        prev_shoot_cmd_ = false;
                    }
                    selected_target_idx_ = switch_to_second_ ? secondary_target_idx_ : primary_target_idx_;
                    if (selected_target_idx_ >= detected_target_count || selected_target_idx_ < 0) {
                        selected_target_idx_ = 0;
                    }
                }

                auto buffCameraPoints{
                    buff_detector().getCameraPointsByIndex(static_cast<std::size_t>(selected_target_idx_))}; //R标+裝甲板5點
                if (buffCameraPoints.size() != 6) {
                    buff_success = false;
                    can_solve = false;
                    roslog::warn("invalid buff camera points size: {}", buffCameraPoints.size());
                }

                if (can_solve) {
                    // buff_calculator
                    //这里应該要整入time了
                    buff_calculator().buff_frame.set(image, std::chrono::steady_clock::now(), 
                        last_var.GimbalAngles.pitch, last_var.GimbalAngles.yaw, 0); // ✅ 修正為小寫：pitch, yaw, roll

                    const int resolved_mode = resolve_buff_mode();
                    if (resolved_mode != buff_mode_) {
                        reload_big_buff_ = !reload_big_buff_;
                        roslog::info(
                            "buff mode switch: {} -> {}, reload_toggle={}",
                            buff_mode_, resolved_mode, reload_big_buff_);
                        buff_mode_ = resolved_mode;
                    }

                    const float solver_bullet_speed = get_buff_solver_bullet_speed();
                    bool buffResult =
                        buff_calculator().calculate(
                            buff_calculator().buff_frame,
                            buffCameraPoints,
                            buff_mode_,
                            solver_bullet_speed,
                            reload_big_buff_);
                    if (!buffResult)
                    {
                        roslog::error("buff_calculator fail");
                        //TODO 改这里(问许家俊)
                        // *buff_pitch = last_var.GimbalAngles.pitch + 90.0;
                        // *buff_yaw = last_var.GimbalAngles.yaw + 90.0;
                        buff_success = false;
                    }
                    else
                    {
                        // roslog::warn("buff_calculator success");
                        // 更新共享变量
                        *buff_pitch = buff_calculator().get_predictPitch();
                        *buff_yaw = buff_calculator().get_predictYaw();
                        buff_success = true;
                    }
                }
            }
        }
    
        // roslog::warn("buff_success: {}", buff_success);
        if (buff_success){
            if(buff_calculator().is_shift){
                timer.reset2();
            }
            driver::ParsedSerialData parsed_data{};
            parsed_data.pitch_now = static_cast<float>(last_var.GimbalAngles.pitch);
            parsed_data.yaw_now = static_cast<float>(last_var.GimbalAngles.yaw);
            const auto control_result = buff_controller.buff_control(parsed_data, buff_pitch, buff_yaw);
            if (!control_result.valid) {
                buff_success = false;
                prev_shoot_cmd_ = false;
                last_var.buffHitterShoot = false;
                last_var.buff_follow = true;
                PubData(last_var.buffHitterShoot, last_var.GimbalAngles);
            } else {
                const bool shoot_cmd_now = (control_result.shoot_flag != 0);
                if (dual_target_active_ && !switch_to_second_ && !prev_shoot_cmd_ && shoot_cmd_now) {
                    switch_to_second_ = true;
                }
                prev_shoot_cmd_ = shoot_cmd_now;

                if (timer.call()) {
                    roslog::warn("###############caall~!!!!!!!!!!");
                    last_var.buffHitterShoot = (control_result.shoot_flag != 0);
                } else {
                    last_var.buffHitterShoot = false;
                }
                last_var.buff_follow = true;
                auto copy = last_var.GimbalAngles;
                copy.pitch = control_result.pitch_setpoint;
                copy.yaw = control_result.yaw_setpoint;
                PubData(last_var.buffHitterShoot, copy);
            }
        }
        else
        {
            prev_shoot_cmd_ = false;
            last_var.buffHitterShoot = false;
            last_var.buff_follow = true;
            PubData(last_var.buffHitterShoot, last_var.GimbalAngles);
        }
        
        std::this_thread::sleep_for(1ms);  // 避免 busy loop
        }
    }
};

int main(int argc, char **argv) try {
    rclcpp::init(argc, argv);
    roslog::info("main: running %s", Application::Name);
    Application app{};
    // [修復] 使用 ament_index_cpp 動態獲取包路徑，不再硬編碼機器路徑
    std::string config_path = ament_index_cpp::get_package_share_directory("buff_hitter") + "/config/config.json";
    app.Init(config_path);
    app.Run(argc, argv);
    rclcpp::shutdown();
    return 0;
}
catch (const std::exception &e) {
    roslog::error("main: {}", e.what());
    rclcpp::shutdown();
    return 1;
}
