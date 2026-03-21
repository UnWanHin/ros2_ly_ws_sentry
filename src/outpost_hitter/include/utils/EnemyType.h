// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <map>

namespace LY_UTILS
{
    enum ENEMY_TYPE
    {
        UNKNOW_TYPE = 0,
        Hero = 1,
        Engineer = 2,
        Infantry3 = 3,
        Infantry4 = 4,
        Infantry5 = 5,
        Sentry = 6,
        Outpost = 7,
        Base = 8,
        FALSE_TYPE = 9
    };

    // 重载 + 运算符
    inline ENEMY_TYPE operator+(ENEMY_TYPE lhs, ENEMY_TYPE rhs)
    {
        int sum = static_cast<int>(lhs) + static_cast<int>(rhs);
        // 确保结果在枚举范围内
        sum %= (FALSE_TYPE + 1);
        return static_cast<ENEMY_TYPE>(sum);
    }

    // 重载 <= 运算符
    inline bool operator<=(ENEMY_TYPE lhs, ENEMY_TYPE rhs)
    {
        return static_cast<int>(lhs) <= static_cast<int>(rhs);
    }

    // 重载 >= 运算符
    inline bool operator>=(ENEMY_TYPE lhs, ENEMY_TYPE rhs)
    {
        return static_cast<int>(lhs) >= static_cast<int>(rhs);
    }

    // 重载 ++ 运算符（前置自增）
    inline ENEMY_TYPE &operator++(ENEMY_TYPE &val)
    {
        int nextVal = static_cast<int>(val) + 1;
        if (nextVal > static_cast<int>(FALSE_TYPE))
        {
            nextVal = static_cast<int>(FALSE_TYPE);
        }
        val = static_cast<ENEMY_TYPE>(nextVal);
        return val;
    }

    // 重载 ++ 运算符（后置自增）
    inline ENEMY_TYPE operator++(ENEMY_TYPE &val, int)
    {
        ENEMY_TYPE temp = val;
        ++val;
        return temp;
    }

    enum class ARMOR_SIZE
    {
        BIG_ARMOR,
        SMALL_ARMOR,
        UNKNOW
    };

    typedef std::unordered_map<ENEMY_TYPE, ARMOR_SIZE> ARMOR_SIZE_MAP;
}