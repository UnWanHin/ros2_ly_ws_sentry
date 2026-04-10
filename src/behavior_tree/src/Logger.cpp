// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include <filesystem>
#include <cstdlib>
#include <vector>

using namespace Utils::Logger;

namespace BehaviorTree {

    namespace {
    std::string ResolveLogDir() {
        std::vector<std::string> candidates;
        if (const char* bt_log_dir = std::getenv("BT_LOG_DIR"); bt_log_dir && *bt_log_dir) {
            candidates.emplace_back(bt_log_dir);
        }
        if (const char* ros_log_dir = std::getenv("ROS_LOG_DIR"); ros_log_dir && *ros_log_dir) {
            candidates.emplace_back(ros_log_dir);
        }
        if (const char* home_dir = std::getenv("HOME"); home_dir && *home_dir) {
            candidates.emplace_back(std::string(home_dir) + "/Log");
        }
        candidates.emplace_back("/tmp");

        for (const auto& dir : candidates) {
            if (dir.empty()) {
                continue;
            }
            try {
                std::filesystem::create_directories(dir);
                if (std::filesystem::exists(dir) && std::filesystem::is_directory(dir)) {
                    return dir;
                }
            } catch (...) {
                // try next candidate
            }
        }
        return "/tmp";
    }
    } // namespace

    std::string GenerateLogFilename() {
        // 获取当前时间点
        auto now = std::chrono::system_clock::now();
        // 转换为时间戳
        auto timestamp = std::chrono::system_clock::to_time_t(now);
        // 转换为本地时间
        std::tm time_info = *std::localtime(&timestamp);
    
        const std::string log_dir = ResolveLogDir();

        // 使用 stringstream 构建文件名
        std::ostringstream oss;
        oss << log_dir << "/BT_"
            << std::setw(4) << std::setfill('0') << (time_info.tm_year + 1900)
            << std::setw(2) << std::setfill('0') << (time_info.tm_mon + 1)
            << std::setw(2) << std::setfill('0') << time_info.tm_mday << "_"
            << std::setw(2) << std::setfill('0') << time_info.tm_hour
            << std::setw(2) << std::setfill('0') << time_info.tm_min
            << std::setw(2) << std::setfill('0') << time_info.tm_sec
            << ".log";
    
        return oss.str();
    } 
    bool Application::InitLogger() {
        LoggerPtr = std::make_shared<Logger>();
        auto filename = GenerateLogFilename();  // 动态生成文件名
        auto consolePolicy = std::make_shared<ConsoleLogPolicy>();
        LoggerPtr->AddPolicy(consolePolicy);
        try {
            auto filePolicy = std::make_shared<FileLogPolicy>(filename);
            LoggerPtr->AddPolicy(filePolicy);
        } catch (const std::exception& ex) {
            if (node_) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "File logger disabled (%s). Fallback to console only.",
                    ex.what());
            }
        }
        return true;
    }
}
