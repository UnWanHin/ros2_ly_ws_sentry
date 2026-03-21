// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include "motion_model.hpp"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "TimeStamp/TimeStamp.hpp"
#include "auto_aim_common/PredictionType.hpp"
#include "auto_aim_common/TrackerType.hpp"
#include <rclcpp/rclcpp.hpp> // [ROS 2] 必須包含

namespace ly_auto_aim::inline predictor {
    using tracker::TrackResultPairs;

    // [ROS 2] 聲明全局節點指針，讓外部可以看到
    extern rclcpp::Node::SharedPtr global_predictor_node;

class Predictor{
private:
    std::mutex car_mutex;
    std::map<int, std::unique_ptr<MotionModel>> cars;
    std::map<int, int> detect_count;//detected then set to 0, not detected then add 1, upper than 10 then delete.
    const int MaxMissFrame = 10;
    VectorY world2model(const VectorY& measure);
    Prediction model2world(const VectorX& state, std::function<VectorY(const VectorX&, int)> measureFunc);
public:
    std::function<Predictions(Time::TimeStamp)> predictFunc();
    Predictions predict(Time::TimeStamp timestamp);
    void update(const TrackResultPairs& trackResults, const Time::TimeStamp& timestamp);
    bool Stable() const{return true;};
    
};
std::unique_ptr<Predictor> createPredictor();

} // namespace predictor