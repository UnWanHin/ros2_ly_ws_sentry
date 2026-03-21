// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

// auto_aim_common/include/Logger/Logger.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <array>
#include <cstdio>
#include <sstream>
#include <string_view>
#include <type_traits>
#include <utility>

namespace roslog {

inline rclcpp::Logger get_logger() {
    static const rclcpp::Logger logger = rclcpp::get_logger("LY");
    return logger;
}

namespace detail {

inline bool has_brace_placeholder(const std::string_view fmt) {
    return fmt.find("{}") != std::string_view::npos;
}

inline bool has_printf_placeholder(const std::string_view fmt) {
    for (std::size_t index = 0; index < fmt.size(); ++index) {
        if (fmt[index] != '%') {
            continue;
        }
        if (index + 1 < fmt.size() && fmt[index + 1] == '%') {
            ++index;
            continue;
        }
        return true;
    }
    return false;
}

template<typename... Args>
inline std::string format_printf(const char* fmt, Args&&... args) {
    char buffer[1024];
    std::snprintf(buffer, sizeof(buffer), fmt, std::forward<Args>(args)...);
    return std::string(buffer);
}

template<typename T, typename = void>
struct is_streamable : std::false_type {};

template<typename T>
struct is_streamable<T, std::void_t<decltype(std::declval<std::ostream&>() << std::declval<const T&>())>>
    : std::true_type {};

template<typename T>
inline std::string to_string_any(T&& value) {
    using ValueType = std::decay_t<T>;

    if constexpr (std::is_same_v<ValueType, std::string>) {
        return value;
    } else if constexpr (std::is_same_v<ValueType, std::string_view>) {
        return std::string(value);
    } else if constexpr (std::is_same_v<ValueType, const char*> || std::is_same_v<ValueType, char*>) {
        return value ? std::string(value) : std::string("<null>");
    } else if constexpr (std::is_same_v<ValueType, bool>) {
        return value ? "true" : "false";
    } else if constexpr (std::is_integral_v<ValueType> && std::is_unsigned_v<ValueType>) {
        return std::to_string(static_cast<unsigned long long>(value));
    } else if constexpr (std::is_integral_v<ValueType> && std::is_signed_v<ValueType>) {
        return std::to_string(static_cast<long long>(value));
    } else if constexpr (std::is_floating_point_v<ValueType>) {
        return std::to_string(static_cast<long double>(value));
    } else if constexpr (is_streamable<ValueType>::value) {
        std::ostringstream stream;
        stream << value;
        return stream.str();
    } else {
        return "<unprintable>";
    }
}

template<typename... Args>
inline std::string format_braces(const std::string_view fmt, Args&&... args) {
    constexpr std::size_t arg_count = sizeof...(Args);
    const std::array<std::string, arg_count> values{to_string_any(std::forward<Args>(args))...};

    std::string output;
    output.reserve(fmt.size() + arg_count * 8U);

    std::size_t arg_index = 0;
    for (std::size_t index = 0; index < fmt.size(); ++index) {
        if (fmt[index] == '{' && index + 1 < fmt.size() && fmt[index + 1] == '}') {
            if (arg_index < values.size()) {
                output += values[arg_index++];
            } else {
                output += "{}";
            }
            ++index;
            continue;
        }
        output.push_back(fmt[index]);
    }

    if (arg_index < values.size()) {
        output += " [extra args:";
        for (; arg_index < values.size(); ++arg_index) {
            output.push_back(' ');
            output += values[arg_index];
        }
        output.push_back(']');
    }

    return output;
}

template<typename... Args>
inline std::string format_message(const char* fmt, Args&&... args) {
    if (fmt == nullptr) {
        return {};
    }
    const std::string_view sv(fmt);
    if (has_brace_placeholder(sv) && !has_printf_placeholder(sv)) {
        return format_braces(sv, std::forward<Args>(args)...);
    }
    return format_printf(fmt, std::forward<Args>(args)...);
}

}  // namespace detail

template<typename... Args>
inline void error(const char* fmt, Args&&... args) {
    const auto message = detail::format_message(fmt, std::forward<Args>(args)...);
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
}

template<typename... Args>
inline void warn(const char* fmt, Args&&... args) {
    const auto message = detail::format_message(fmt, std::forward<Args>(args)...);
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
}

template<typename... Args>
inline void info(const char* fmt, Args&&... args) {
    const auto message = detail::format_message(fmt, std::forward<Args>(args)...);
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

template<typename... Args>
inline void debug(const char* fmt, Args&&... args) {
    const auto message = detail::format_message(fmt, std::forward<Args>(args)...);
    RCLCPP_DEBUG(get_logger(), "%s", message.c_str());
}

inline void error(const char* msg) { RCLCPP_ERROR(get_logger(), "%s", msg); }
inline void warn(const char* msg)  { RCLCPP_WARN(get_logger(), "%s", msg); }
inline void info(const char* msg)  { RCLCPP_INFO(get_logger(), "%s", msg); }
inline void debug(const char* msg) { RCLCPP_DEBUG(get_logger(), "%s", msg); }

inline void error(const std::string& msg) { RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); }
inline void warn(const std::string& msg)  { RCLCPP_WARN(get_logger(), "%s", msg.c_str()); }
inline void info(const std::string& msg)  { RCLCPP_INFO(get_logger(), "%s", msg.c_str()); }
inline void debug(const std::string& msg) { RCLCPP_DEBUG(get_logger(), "%s", msg.c_str()); }

} // namespace roslog
