// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

//re-build ???
#pragma once
#include <chrono>
#include <cmath>

namespace LangYa {

struct SineWave final
{
    float Amplitude{ 1.0f };
    float Phase{ 0.0f };
    std::chrono::steady_clock::duration RoundTime{ std::chrono::seconds{1} };
    std::chrono::steady_clock::time_point StartTime{ std::chrono::steady_clock::now() };

    SineWave() noexcept = default;

    SineWave(
            const float amplitude,
            const float phase,
            const std::chrono::steady_clock::duration roundTime,
            const std::chrono::steady_clock::time_point startTime) :
            Amplitude(amplitude),
            Phase(phase),
            RoundTime(roundTime),
            StartTime(startTime)
    {
    }

    SineWave(const float amplitude, const float phase, const std::chrono::steady_clock::duration roundTime) :
            Amplitude(amplitude), Phase(phase), RoundTime(roundTime)
    {
    }

    [[nodiscard]] float Produce(const std::chrono::steady_clock::time_point& now) const
    {
        const float t = static_cast<float>(
                std::chrono::duration_cast<std::chrono::milliseconds>(now - StartTime).count());
        const float round_time = static_cast<float>(
                std::chrono::duration_cast<std::chrono::milliseconds>(RoundTime).count());
        const float angle = t * 2.0f * std::numbers::pi_v<float> / round_time;
        return Amplitude * std::sin(angle + Phase);
    }

    [[nodiscard]] float Produce() const { return Produce(std::chrono::steady_clock::now()); }

    [[nodiscard]] bool GetItem(float& item) noexcept
    {
        item = Produce();
        return true;
    }

    bool Process(const std::chrono::time_point<std::chrono::steady_clock>& argument, float& result)
    {
        result = Produce(argument);
        return true;
    }
};

}