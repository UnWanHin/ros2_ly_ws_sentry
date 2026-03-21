// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <auto_aim_common/msg/car.hpp>
#include <vector>

namespace ly_auto_aim::inline detector
{
class CarFinder {
public:
    CarFinder() = default;
    ~CarFinder() = default;

    [[nodiscard]]
    bool FindCar(std::vector<CarDetection> &cars, std::vector<auto_aim_common::msg::Car> &cars_msg) const noexcept {
        if (cars.empty()) return false;
        cars_msg.clear();
        for (const auto &car: cars) {
            auto_aim_common::msg::Car car_msg;
            car_msg.bounding_rect.x = car.bounding_rect.x;
            car_msg.bounding_rect.y = car.bounding_rect.y;
            car_msg.bounding_rect.width = car.bounding_rect.width;
            car_msg.bounding_rect.height = car.bounding_rect.height;
            car_msg.car_id = car.tag_id;
            cars_msg.emplace_back(std::move(car_msg));
        }
        return true;
    }
};
} // namespace ly_auto_aim::inline detector