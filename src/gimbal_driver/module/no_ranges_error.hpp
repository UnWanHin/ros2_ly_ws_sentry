// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <algorithm>
#include <ranges>


namespace ly{
/// @brief copy( <源容器>, <目的容器>.begin() )
/// @tparam ...Args 
/// @param ...args 
/// @return 
template<typename ...Args>
auto copy(Args &&... args) {
    return std::ranges::copy(std::forward<Args>(args)...);
}

/// @brief fill( <容器>, <值> )
/// @tparam ...Args 
/// @param ...args 
/// @return 
template<typename ...Args>
auto fill(Args &&... args) {
    return std::ranges::fill(std::forward<Args>(args)...);
}

/// @brief find( <容器>, <值> )
/// @tparam ...Args 
/// @param ...args 
/// @return 
template<typename ...Args>
auto find(Args &&... args) {
    return std::ranges::find(std::forward<Args>(args)...);
}
}