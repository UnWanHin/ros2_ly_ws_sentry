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
#include <ament_index_cpp/get_package_share_directory.hpp>

// TODO copy to auto aim detector
LY_DEF_ROS_TOPIC(ly_aa_enable, "/ly/aa/enable", std_msgs::msg::Bool);

// rune aim
LY_DEF_ROS_TOPIC(ly_ra_enable, "/ly/ra/enable", std_msgs::msg::Bool);

// LY_DEF_ROS_TOPIC(ly_camera_image, "/ly/camera/image", sensor_msgs::msg::Image);
LY_DEF_ROS_TOPIC(ly_ra_image, "/ly/ra/image", sensor_msgs::msg::Image);
LY_DEF_ROS_TOPIC(ly_compressed_image, "/ly/compressed/image", sensor_msgs::msg::CompressedImage);

LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
LY_DEF_ROS_TOPIC(ly_gimbal_firecode, "/ly/gimbal/firecode", std_msgs::msg::UInt8);

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
    inline static constexpr const auto bullet_speed = 22.9; // m/s

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
    //BuffController buff_controller;

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

        std::string buff_model_path = (param["buff"])["buff_model_path"].String();
        std::filesystem::path model_path(buff_model_path);
        if (model_path.is_relative() && !std::filesystem::exists(model_path)) {
            // 优先按工作区根目录解析，避免 launch 工作目录差异导致模型找不到。
            const auto pkg_share = std::filesystem::path(ament_index_cpp::get_package_share_directory("buff_hitter"));
            const auto ws_root = pkg_share.parent_path().parent_path().parent_path().parent_path();
            const auto candidate = ws_root / model_path;
            if (std::filesystem::exists(candidate)) {
                model_path = candidate;
            }
        }
        buff_model_path = model_path.string();
        roslog::info("buff model path: {}", buff_model_path);
        buff_detector_ptr = std::make_shared<BuffDetector>(buff_model_path);  //同下
        buff_calculator_ptr = std::make_shared<BuffCalculator>(param);   //属于solver 越級了, 后面加新模塊buff再移
    }

    void Run(int argc, char **argv) {
    GenSubs();
    Node.GenSubscriber<ly_aa_enable>([this](const std_msgs::msg::Bool& msg){ aa_enable_callback(msg); });
    
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
        
        if (buff_detector().buffDetect(image) == false)
        {
            roslog::error("buff_detector fail");
            buff_success = false;
        }
        else
        {
            // roslog::warn("buff_detector success!!");
            auto buffCameraPoints{ buff_detector().getCameraPoints() }; //R标+裝甲板5點
            // buff_calculator
            //这里应該要整入time了
            buff_calculator().buff_frame.set(image, std::chrono::steady_clock::now(), 
                last_var.GimbalAngles.pitch, last_var.GimbalAngles.yaw, 0); // ✅ 修正為小寫：pitch, yaw, roll
            
            bool buffResult = buff_calculator().calculate(buff_calculator().buff_frame, buffCameraPoints, bullet_speed);
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
    
        // roslog::warn("buff_success: {}", buff_success);
        if (buff_success){
            if(buff_calculator().is_shift){
                timer.reset2();
            }
            if(timer.call())
            {
                roslog::warn("###############caall~!!!!!!!!!!");
                last_var.buffHitterShoot = true;
            }
            last_var.buff_follow = true;
            auto copy = last_var.GimbalAngles;
            copy.pitch = *buff_pitch; 
            copy.yaw = *buff_yaw + std::round((last_var.GimbalAngles.yaw - *buff_yaw) / 360.0f) * 360.0f; 
            // roslog::warn("target yaw: {}, target pitch: {}", copy.yaw, copy.pitch);  // ✅ 修正為小寫
            PubData(last_var.buffHitterShoot, copy);
        }
        else
        {
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
