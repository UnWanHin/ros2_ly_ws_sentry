// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <span>
#include <cstdint>
#include <algorithm>
#include <ranges>
#include <span>
#include "crc_checker.hpp"

namespace LangYa {
    class PPSpan final
    {
    public:
        static constexpr std::uint8_t head{ '!' };
        static constexpr std::uint8_t tail{ '\0' };

    private:
        [[nodiscard]] static bool crc_is_correct(const std::span<const std::uint8_t> span) noexcept {
            if (span.size() < 2) return false; // 至少包含头和CRC
            const auto data_part = span.subspan(0, span.size() - 1); // 头+数据部分
            const auto received_crc = span.back(); // 最后一位是CRC
            return CRCChecker::CRC8::calculate(data_part.data(), data_part.size()) == received_crc;
        }

        [[nodiscard]] static bool verify(const std::span<const std::uint8_t> span) noexcept
        {
            const auto span_is_valid = span.size() >= 2; // 最小长度：头+CRC
            const auto head_is_correct = span.front() == head;
            return span_is_valid && head_is_correct && crc_is_correct(span);
            // const auto span_is_not_empty = !span.empty();
            // const auto head_is_correct = span.front() == head;
            // const auto tail_is_correct = span.back() == tail;
            // return span_is_not_empty && head_is_correct && tail_is_correct;
        }

        /// @brief  兼容老接口
        /// @param span 
        /// @param use_crc 
        /// @return 
        [[nodiscard]] static bool legacy_verify(const std::span<const std::uint8_t> span, 
                                                bool use_crc = true) noexcept 
        {
            if(use_crc) {
            return verify(span);
            }
            // 原始尾部校验逻辑（兼容模式）
            return !span.empty() && 
            (span.front() == head) && 
            (span.back() == tail);
        }

        /**
         * 发送数据的时候可能会使用
         */
        static void append_crc(std::span<uint8_t> span) noexcept {
            if(span.size() < 2) return;
            const auto data_part = span.subspan(1, span.size() - 2);
            span.back() = CRCChecker::CRC8::calculate(data_part.data(), data_part.size());
        }

        bool is_last_message_found_in_pong{ false };
	    [[nodiscard]] bool find_span(std::span<const std::uint8_t>& span) const noexcept
        {
            const auto head_index = std::ranges::find(ping, head);
            if (head_index == ping.end()) return false;
            span = full.subspan(head_index - ping.begin(), ping.size());
            return true;
        }

    public:
        std::span<std::uint8_t> ping;
        std::span<std::uint8_t> pong;
        std::span<std::uint8_t> full;

        explicit PPSpan(const std::span<std::uint8_t> fullSpan) :
                ping(fullSpan.data(), fullSpan.size() / 2),
                pong(fullSpan.data() + ping.size(), ping.size()),
                full(fullSpan)
        {
            //if (ping.empty()) throw std::invalid_argument("sizeof fullSpan must be at least 2");
            std::ranges::fill(ping, 0);
        } 

        [[nodiscard]] bool examine(const std::span<std::uint8_t> destination) noexcept
        {
            if (destination.size() < ping.size()) return false;

            // 如果 pong 中有完整的数据包，则直接输出
            if (verify(pong))
            {
                std::ranges::copy(pong, destination.begin());
                if (!is_last_message_found_in_pong)
                {
                    is_last_message_found_in_pong = true;
                    std::ranges::fill(ping, 0);
                }
                return true;
            }

            is_last_message_found_in_pong = false;
            std::span<const std::uint8_t> message_span;
            const auto successful = find_span(message_span) && verify(message_span);
            if (successful) std::ranges::copy(message_span, destination.begin());
            std::ranges::copy(pong, ping.begin()); // 无论成功与否，都利用新数据覆盖 PingSpan
            return successful;
        }
    };

    template<typename TItem>
    struct PPBuffer {
        std::array<std::uint8_t, sizeof(TItem) * 2> Array{};
        PPSpan Span{Array};
    };
}
