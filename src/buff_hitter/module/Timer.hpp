// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <cstddef>

namespace power_rune {
    /// @brief A simple timer class that counts up to a maximum value and resets.
    class Timer
    {
        std::size_t _cur{0};
        std::size_t _max{};

        public:
            Timer(int max) : _max(max) {} 

            void reset2(){
                _cur = _max /2;
            }

            /// @brief Increase the current count and check if it has reached the maximum.
            /// @return 
            bool call() {
                if (_cur + 1>= _max) {
                    reset();
                    return true;
                }

                ++_cur;
                return false;
            }

            void reset() {
                _cur = 0;
            }

            auto& cur() {
                return _cur;
            }

            const auto& cur() const {
                return _cur;
            }

            auto& max() {
                return _max;
            }

            const auto& max() const {
                return _max;
            }
    };
}