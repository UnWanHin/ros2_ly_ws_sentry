// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include "armor_detector/corrected_detector.hpp"
#include "car_detector/yolo_detector.hpp"
#include <auto_aim_common/DetectionType.hpp>
#include <future>
#include <Logger/Logger.hpp>

namespace ly_auto_aim::inline detector{
    using namespace LangYa;
    class CarAndArmorDetector final
    {
    public:
        CorrectedDetector armorDetector{};
        YoloDetector carDetector{};

        bool Detect(const cv::Mat& src, std::vector<ArmorObject>& armors, std::vector<CarDetection>& cars)
        {
            if(src.empty()) return false;

            auto armor_future = std::async(std::launch::async, [&]() {
                return armorDetector.Detect(src, armors);
            });
            auto car_future = std::async(std::launch::async, [&]() {
                return carDetector.infer(src);
            });

            if(!armor_future.get()) return false;

            std::vector<YoloDetection> car_detections = car_future.get();
            std::vector<CarDetection> car_results;
            for (const auto& detection : car_detections) {
                CarDetection car;
                car.bounding_rect = cv::Rect2f(detection.rect);
                car.tag_id = detection.class_id;
                car.score = detection.confidence;
                car.center = cv::Point2f(detection.center);
                car_results.emplace_back(car);
                // roslog::info("car id: {}, score: {}", car.tag_id, car.score);
                /// 绘制检测框 
                /// ...
            }

            cars = std::move(car_results);

            return true;
        }
    };
}