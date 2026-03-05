#include "../include/Application.hpp"
#include <filesystem>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "Node.hpp"
    
using namespace LangYa;
using namespace BehaviorTree;

namespace BehaviorTree {

#ifndef BT_DIAG_CONSOLE
#define BT_DIAG_CONSOLE 0
#endif

#if BT_DIAG_CONSOLE
#define BT_DIAG_LOG(...) std::fprintf(stderr, __VA_ARGS__)
#else
#define BT_DIAG_LOG(...) ((void)0)
#endif
    
    Application::Application(int argc, char **argv) {
        BT_DIAG_LOG("[BT_DIAG] ctor enter\n");
        // 1. [ROS 2] 初始化 ROS Context
        // 雖然通常在 main 裡做，但在這裡確保 options 可以傳入
        if (!rclcpp::ok()) {
            rclcpp::init(argc, argv);
        }
        BT_DIAG_LOG("[BT_DIAG] rclcpp init ok\n");

        // 2. [ROS 2] 創建節點實例
        // [DIAG] 极简 Node 构造，排除 NodeOptions 影响
        node_ = std::make_shared<rclcpp::Node>(nodeName);
        BT_DIAG_LOG("[BT_DIAG] node created\n");

        // 3. 初始化日誌 (Logger)
        if(InitLogger()) {
            LoggerPtr->Info("Logger Init Success!");
            RCLCPP_INFO(node_->get_logger(), "Logger Init Success!");
        } else {
            // 如果日誌初始化失敗，至少用 ROS logger 報錯
            RCLCPP_ERROR(node_->get_logger(), "Logger Init Failed!");
            throw std::runtime_error("Logger Init Failed!");
        }
        BT_DIAG_LOG("[BT_DIAG] logger initialized\n");

        // 4. [ROS 2] 動態獲取 Config 和 XML 路徑
        try {
            std::string pkg_path = ament_index_cpp::get_package_share_directory("behavior_tree");
            behavior_tree_file_ = pkg_path + "/Scripts/main.xml";
            config_file_       = pkg_path + "/Scripts/config.json";
            
            LoggerPtr->Info("Loading BT from: {}", behavior_tree_file_);
            LoggerPtr->Info("Loading Config from: {}", config_file_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Error: Could not find package 'behavior_tree'.");
            throw e;
        }
        BT_DIAG_LOG("[BT_DIAG] paths resolved\n");

        // 5. 初始化配置與通訊
        SubscribeMessageAll();  // 建立訂閱
        BT_DIAG_LOG("[BT_DIAG] subscriptions ready\n");

        // [修復] 初始化所有發布者指針 — 必須在 PublishMessageAll() 之前完成
        // 每個指針對應 Topic.hpp 中的 topic 名稱和 Application.hpp 中聲明的類型
        pub_aa_enable_       = node_->create_publisher<std_msgs::msg::Bool>(ly_aa_enable::Name, 10);
        pub_ra_enable_       = node_->create_publisher<std_msgs::msg::Bool>(ly_ra_enable::Name, 10);
        pub_outpost_enable_  = node_->create_publisher<std_msgs::msg::Bool>(ly_outpost_enable::Name, 10);
        pub_gimbal_control_  = node_->create_publisher<gimbal_driver::msg::GimbalAngles>(ly_control_angles::Name, 10);
        pub_gimbal_firecode_ = node_->create_publisher<std_msgs::msg::UInt8>(ly_control_firecode::Name, 10);
        pub_gimbal_posture_  = node_->create_publisher<std_msgs::msg::UInt8>(ly_control_posture::Name, 10);
        pub_gimbal_vel_      = node_->create_publisher<gimbal_driver::msg::Vel>(ly_control_vel::Name, 10);
        pub_navi_vel_        = node_->create_publisher<gimbal_driver::msg::Vel>(ly_navi_vel::Name, 10);
        pub_navi_goal_       = node_->create_publisher<std_msgs::msg::UInt8>(ly_navi_goal::Name, 10);
        pub_navi_goal_pos_   = node_->create_publisher<std_msgs::msg::UInt16MultiArray>(ly_navi_goal_pos::Name, 10);
        pub_navi_speed_level_= node_->create_publisher<std_msgs::msg::UInt8>(ly_navi_speed_level::Name, 10);
        pub_navi_lower_head_ = node_->create_publisher<std_msgs::msg::UInt8>(ly_navi_lower_head::Name, 10);
        pub_bt_target_       = node_->create_publisher<std_msgs::msg::UInt8>(ly_bt_target::Name, 10);
        BT_DIAG_LOG("[BT_DIAG] publishers ready\n");

        ConfigurationInit();    // 讀取 config.json
        BT_DIAG_LOG("[BT_DIAG] config ready\n");

        // 6. 註冊與加載行為樹
        if(!RegisterTreeNodes()) {
            throw std::runtime_error("Behavior Tree Node Register Failed!");
        }
        BT_DIAG_LOG("[BT_DIAG] register nodes ready\n");
        
        // 建議這裡也把 LoadBehaviorTree 加上，除非你是在 GameLoop 裡動態加載
        if (!LoadBehaviorTree()) {
            LoggerPtr->Error("Behavior Tree Load Failed!");
             throw std::runtime_error("Behavior Tree Load Failed!");
        }
        BT_DIAG_LOG("[BT_DIAG] tree loaded\n");

        LoggerPtr->Info("Application Start!");
    }

    Application::~Application() {
        if(LoggerPtr) {
            LoggerPtr->Info("Application Stop!");
            LoggerPtr->Flush();
        }
        // ROS 2 節點會由智能指針自動釋放
    }

    void Application::Run() {
        // 1. 等待比賽開始 (這通常會阻塞，直到收到裁判系統消息)
        WaitBeforeGame();

        // 2. 記錄比賽開始時間
        gameStartTime = std::chrono::steady_clock::now();

        // 3. 進入主循環 (處理業務邏輯和 BT Tick)
        GameLoop();
    }
}
