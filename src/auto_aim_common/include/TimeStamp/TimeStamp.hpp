// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace Time {

class DurationExt : public rclcpp::Duration {
public:
    using rclcpp::Duration::Duration;

    double count() const { return this->seconds() * 1000.0; }
    double toSeconds() const { return this->seconds(); }

    DurationExt operator-(const DurationExt& other) const {
        return DurationExt(std::chrono::nanoseconds(this->nanoseconds() - other.nanoseconds()));
    }

    std::string toString() const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << this->seconds() << "s";
        return ss.str();
    }
};

using TimeStamp = rclcpp::Time;
using TimeDuration = rclcpp::Duration;

inline std::string toString(const TimeStamp& timestamp) {
    if (timestamp.nanoseconds() == 0) return "0";

    auto seconds = timestamp.seconds();
    auto ns = timestamp.nanoseconds() % 1000000000LL;
    time_t raw_time = static_cast<time_t>(seconds);
    struct tm* time_info = localtime(&raw_time);

    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", time_info);

    std::stringstream ss;
    ss << buffer << "." << std::setw(9) << std::setfill('0') << ns;
    return ss.str();
}

inline DurationExt operator-(const TimeStamp& t1, const TimeStamp& t2) {
    return DurationExt(std::chrono::nanoseconds(t1.nanoseconds() - t2.nanoseconds()));
}

inline TimeStamp now() {
    return rclcpp::Clock(RCL_SYSTEM_TIME).now();
}

} // namespace Time