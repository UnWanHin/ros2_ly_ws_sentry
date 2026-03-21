// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <chrono>
#include <thread>

namespace LangYa {
    using TimePoint = std::chrono::steady_clock::time_point;
    using Milliseconds = std::chrono::milliseconds;
    using Seconds = std::chrono::seconds;

    template<typename T>
    concept HasComparisonAndAssignment = requires(T a, T b) {
        { a > b }  -> std::convertible_to<bool>;
        { a < b }  -> std::convertible_to<bool>;
        { a == b } -> std::convertible_to<bool>;
        { a = b }  -> std::same_as<T&>;
    };

    // 频率计时器
    class RateClock final {
    public:
        RateClock(int frequency) {
            reset(frequency);
        }
        void reset(int frequency) {
            if (frequency <= 0) {
                throw std::invalid_argument("Frequency must be positive.");
            }
            interval_ = Milliseconds(1000 / frequency);
            last_tick_time = std::chrono::steady_clock::now();
        }
        void sleep() {
            auto now_tick_time = std::chrono::steady_clock::now();
            if (now_tick_time - last_tick_time < interval_) {
                std::this_thread::sleep_for(interval_ - (now_tick_time - last_tick_time));
            }
            last_tick_time = now_tick_time;
        }
        bool trigger() {
            auto now_tick_time = std::chrono::steady_clock::now();
            return (now_tick_time - last_tick_time) >= interval_;
        }
        void tick() {
            last_tick_time = std::chrono::steady_clock::now();
        }
    private:
        TimePoint last_tick_time;
        Milliseconds interval_;
    };

    // 秒计时器
    class TimerClock final {
    public:
        TimerClock(Seconds interval) {
            reset(interval);
        }
        void reset(Seconds interval) {
            if (interval.count() <= 0) {
                throw std::invalid_argument("Interval must be positive.");
            }
            interval_ = interval;
            last_tick_time = std::chrono::steady_clock::now();
        }
        bool trigger() {
            auto now_tick_time = std::chrono::steady_clock::now();
            if (now_tick_time - last_tick_time >= interval_) {
                return true;
            }
            return false;
        }
        void tick() {
            last_tick_time = std::chrono::steady_clock::now();
        }
    private:
        TimePoint last_tick_time;
        Seconds interval_;
    };

    // 下降检测器
    template<HasComparisonAndAssignment T>
    class DescentDetector final {
    public:
        DescentDetector(T value) : value_(value), count_(0) {}
        bool trigger(T value) {
            if (value_ > value) {
                value_ = value;
                return true;
            }
            value_ = value;
            return false;
        }
    private:
        T value_;
        int count_ = 0;
    };

    // 上升检测器
    template<HasComparisonAndAssignment T>
    class AscentDetector final {
    public:
        AscentDetector(T value) : value_(value), count_(0) {}
        bool trigger(T value) {
            if (value_ < value) {
                value_ = value;
                return true;
            }
            value_ = value;
            return false;
        }
    private:
        T value_;
        int count_;
    };
    
}

//時間頻率控制?