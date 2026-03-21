// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <fmt/format.h>
#include <fmt/chrono.h>

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>


namespace LangYa::roslog
{
inline rclcpp::Logger& getLogger() {
    static rclcpp::Logger logger = rclcpp::get_logger("LangYa");
    return logger;
}

#define LY_DEF_ROS_LOG(name, rosName)\
    template<typename ...Args> \
    void name(const std::string& format_str, Args &&... args) \
    { \
        const std::string formatted = fmt::format(format_str, std::forward<Args>(args)...); \
        RCLCPP_##rosName(getLogger(), "%s", formatted.c_str()); \
    } \
    // generate ros log function

    LY_DEF_ROS_LOG(error, ERROR);
    LY_DEF_ROS_LOG(warn, WARN);
    LY_DEF_ROS_LOG(info, INFO);
    LY_DEF_ROS_LOG(debug, DEBUG);
#undef LY_DEF_ROS_LOG

}