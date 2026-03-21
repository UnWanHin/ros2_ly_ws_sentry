// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include <filesystem>
#include <cstdlib>

using namespace Utils::Logger;

namespace BehaviorTree {

    std::string GenerateLogFilename() {
        // 获取当前时间点
        auto now = std::chrono::system_clock::now();
        // 转换为时间戳
        auto timestamp = std::chrono::system_clock::to_time_t(now);
        // 转换为本地时间
        std::tm time_info = *std::localtime(&timestamp);
    
        // [ROS 2] 用環境變量 $HOME 構建路徑，不再硬編碼 /home/hustlyrm
        const char* home_dir = std::getenv("HOME");
        std::string log_dir = home_dir ? std::string(home_dir) + "/Log" : "/tmp";

        // 確保目錄存在
        std::filesystem::create_directories(log_dir);

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
        auto filePolicy = std::make_shared<FileLogPolicy>(filename);
        auto consolePolicy = std::make_shared<ConsoleLogPolicy>();
        LoggerPtr->AddPolicy(filePolicy);
        LoggerPtr->AddPolicy(consolePolicy);
        return true;
    }
}