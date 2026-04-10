// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <cctype>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "Node.hpp"
    
using namespace LangYa;
using namespace BehaviorTree;

namespace BehaviorTree {

namespace {

std::string NormalizeProfile(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string ReadStringParameter(rclcpp::Node& node, const std::string& name, const std::string& default_value) {
    if (!node.has_parameter(name)) {
        node.declare_parameter(name, default_value);
    }
    std::string value = default_value;
    (void)node.get_parameter(name, value);
    return value;
}

bool ReadBoolParameter(rclcpp::Node& node, const std::string& name, const bool default_value) {
    if (!node.has_parameter(name)) {
        node.declare_parameter(name, default_value);
    }
    rclcpp::Parameter param;
    if (!node.get_parameter(name, param)) {
        return default_value;
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        return param.as_bool();
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return param.as_int() != 0;
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        auto value = NormalizeProfile(param.as_string());
        if (value == "true" || value == "1" || value == "yes" || value == "on") {
            return true;
        }
        if (value == "false" || value == "0" || value == "no" || value == "off") {
            return false;
        }
    }
    return default_value;
}

int ReadIntParameter(rclcpp::Node& node, const std::string& name, const int default_value) {
    if (!node.has_parameter(name)) {
        node.declare_parameter(name, default_value);
    }
    rclcpp::Parameter param;
    if (!node.get_parameter(name, param)) {
        return default_value;
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<int>(param.as_int());
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return static_cast<int>(param.as_double());
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        try {
            return std::stoi(param.as_string());
        } catch (...) {
            return default_value;
        }
    }
    return default_value;
}

std::string ResolveBehaviorTreePath(const std::string& pkg_path, const std::string& configured_path) {
    if (configured_path.empty()) {
        return {};
    }

    const std::filesystem::path path(configured_path);
    if (path.is_absolute()) {
        return path.lexically_normal().string();
    }

    return (std::filesystem::path(pkg_path) / path).lexically_normal().string();
}

std::string DefaultConfigPathForProfile(const std::string& pkg_path, const std::string& profile_override) {
    const auto normalized = NormalizeProfile(profile_override);
    if (normalized == "league") {
        return (std::filesystem::path(pkg_path) / "Scripts/ConfigJson/league_competition.json").string();
    }
    return (std::filesystem::path(pkg_path) / "Scripts/config.json").string();
}

}  // namespace

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
        rclcpp::NodeOptions node_options;
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        node_ = std::make_shared<rclcpp::Node>(nodeName, node_options);
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
            // 说明：
            // competition_profile/bt_config_file/bt_tree_file 都支持 launch 参数覆盖，
            // 便于在不改包内文件的情况下切换联赛/区域赛配置。
            competitionProfileOverride_ = ReadStringParameter(*node_, "competition_profile", "");
            const std::string tree_override = ReadStringParameter(*node_, "bt_tree_file", "");
            const std::string config_override = ReadStringParameter(*node_, "bt_config_file", "");
            debugBypassGameStart_ = ReadBoolParameter(*node_, "debug_bypass_is_start", false);
            publishNaviGoal_ = ReadBoolParameter(*node_, "publish_navi_goal", true);
            waitForGameStartTimeoutSec_ = std::max(0, ReadIntParameter(*node_, "wait_for_game_start_timeout_sec", 0));
            leagueRefereeStaleTimeoutMs_ = std::max(0, ReadIntParameter(*node_, "league_referee_stale_timeout_ms", 0));

            const std::string default_tree_file = pkg_path + "/Scripts/main.xml";
            // 按 profile 选择默认 JSON：
            // league -> league_competition.json
            // 其他 -> Scripts/config.json（后续再由 JSON 内字段修正 profile）。
            const std::string default_config_file = DefaultConfigPathForProfile(pkg_path, competitionProfileOverride_);

            behavior_tree_file_ = tree_override.empty()
                ? default_tree_file
                : ResolveBehaviorTreePath(pkg_path, tree_override);
            config_file_ = config_override.empty()
                ? default_config_file
                : ResolveBehaviorTreePath(pkg_path, config_override);
            
            LoggerPtr->Info("Loading BT from: {}", behavior_tree_file_);
            LoggerPtr->Info("Loading Config from: {}", config_file_);
            if (competitionProfileOverride_.empty()) {
                LoggerPtr->Info("Competition profile override: <config/default>");
            } else {
                LoggerPtr->Info("Competition profile override: {}", competitionProfileOverride_);
            }
            LoggerPtr->Info("debug_bypass_is_start: {}", debugBypassGameStart_ ? "true" : "false");
            LoggerPtr->Info("publish_navi_goal: {}", publishNaviGoal_ ? "true" : "false");
            LoggerPtr->Info("wait_for_game_start_timeout_sec: {}", waitForGameStartTimeoutSec_);
            LoggerPtr->Info("league_referee_stale_timeout_ms: {}", leagueRefereeStaleTimeoutMs_);
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
        // 这里集中初始化，避免运行期因空指针导致发布失败。
        pub_aa_enable_       = node_->create_publisher<std_msgs::msg::Bool>(ly_aa_enable::Name, 10);
        pub_ra_enable_       = node_->create_publisher<std_msgs::msg::Bool>(ly_ra_enable::Name, 10);
        pub_outpost_enable_  = node_->create_publisher<std_msgs::msg::Bool>(ly_outpost_enable::Name, 10);
        pub_gimbal_control_  = node_->create_publisher<gimbal_driver::msg::GimbalAngles>(ly_control_angles::Name, 10);
        pub_gimbal_firecode_ = node_->create_publisher<std_msgs::msg::UInt8>(ly_control_firecode::Name, 10);
        pub_gimbal_posture_  = node_->create_publisher<std_msgs::msg::UInt8>(ly_control_posture::Name, 10);
        pub_gimbal_vel_      = node_->create_publisher<gimbal_driver::msg::Vel>(ly_control_vel::Name, 10);
        pub_navi_vel_        = node_->create_publisher<gimbal_driver::msg::Vel>(ly_navi_vel::Name, 10);
        pub_navi_target_rel_ = node_->create_publisher<auto_aim_common::msg::RelativeTarget>(ly_navi_target_rel::Name, 10);
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

        const auto now_ns = NowSteadyNs();
        runtimeLastLoopBeatNs_.store(now_ns, std::memory_order_relaxed);
        runtimeTickStartNs_.store(now_ns, std::memory_order_relaxed);
        runtimeTickEndNs_.store(now_ns, std::memory_order_relaxed);
        runtimeLastSafePublishNs_.store(0, std::memory_order_relaxed);
        runtimeRecoveryWindowStart_ = std::chrono::steady_clock::now();
        runtimeLastSoftRecoverTime_ = std::chrono::steady_clock::time_point{};

        LoggerPtr->Info("Application Start!");
    }

    Application::~Application() {
        StopRuntimeGuard();
        if(LoggerPtr) {
            LoggerPtr->Info("Application Stop!");
            LoggerPtr->Flush();
        }
        // ROS 2 節點會由智能指針自動釋放
    }

    void Application::Run() {
        // 1. 等待比賽開始 (這通常會阻塞，直到收到裁判系統消息)
        WaitBeforeGame();
        leagueRouteCompatAfterGatePending_ = IsLeagueRouteCompatEnabled();
        leagueRouteCompatActive_ = false;
        leagueRouteCompatUntil_ = std::chrono::steady_clock::time_point{};
        leagueRouteCompatHasPendingGoal_ = false;
        leagueRouteCompatPendingBaseGoal_ = LangYa::Home.ID;
        leagueRouteCompatPendingHoldSec_ = 1;

        // 2. 記錄比賽開始時間
        gameStartTime = std::chrono::steady_clock::now();

        StartRuntimeGuard();

        // 3. 進入主循環 (處理業務邏輯和 BT Tick)
        GameLoop();

        StopRuntimeGuard();
    }
}
