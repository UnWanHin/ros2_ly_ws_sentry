#include "predictor/predictor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fmt/format.h> // 必須包含

using namespace ly_auto_aim;

// [ROS 2] 定義全局變量 (對應頭文件中的 extern)
namespace ly_auto_aim::predictor {
    rclcpp::Node::SharedPtr global_predictor_node = nullptr;
}

// [ROS 2] 本地適配 roslog，避免修改下方邏輯代碼
namespace roslog_fmt {
    static rclcpp::Logger get_logger() {
        if (ly_auto_aim::predictor::global_predictor_node) {
            return ly_auto_aim::predictor::global_predictor_node->get_logger();
        }
        return rclcpp::get_logger("predictor");
    }
    template <typename... Args>
    void debug(const char* format_str, const Args&... args) {
        RCLCPP_DEBUG(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void warn(const char* format_str, const Args&... args) {
        RCLCPP_WARN(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void info(const char* format_str, const Args&... args) {
        RCLCPP_INFO(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void error(const char* format_str, const Args&... args) {
        RCLCPP_ERROR(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
}

// 使用宏替換
#define roslog roslog_fmt

auto predictor::createPredictor() -> std::unique_ptr<Predictor> {
    return std::make_unique<predictor::Predictor>();
}

namespace ly_auto_aim:: inline predictor {
    
    std::function<Predictions(Time::TimeStamp)> Predictor::predictFunc()
    {
        return std::function<Predictions(Time::TimeStamp)>([&](Time::TimeStamp timestamp) { return predict(timestamp); });
    }

    Predictions Predictor::predict(Time::TimeStamp timestamp)
    {
        Predictions predictions;
        std::lock_guard<std::mutex> lock(car_mutex);
        for(auto& [carid, car] : cars)
        {
            predictions.push_back(model2world(car->getPredictResult(timestamp),std::function<VectorY(const VectorX&, int)>([&](const VectorX& state, int armorid) {
                return car->measureFromState(state, armorid);
            })));
            predictions.back().id = carid;
            predictions.back().stable = car->armorStable();
            if (!car->Stable()) {
                roslog::debug("Car {} is not yet stable for fire, keep prediction for aim", carid);
            }
        }
        return predictions;
    }

    void Predictor::update(const TrackResultPairs& trackResults, const Time::TimeStamp& timestamp)
    {
        std::map<int, std::vector<std::tuple<VectorY, int, location::Location, bool>>> measures;
        for(const auto& trackResult : trackResults.first)
        {
            VectorY measure;
            XYZ armor_xyz = trackResult.location.xyz_imu;
            measure[0] = armor_xyz.x;
            measure[1] = armor_xyz.y;
            measure[2] = armor_xyz.z;
            measure[3] = trackResult.yaw;
            measures[trackResult.car_id].push_back(std::make_tuple(measure, trackResult.armor_id, trackResult.location, false));
            detect_count[trackResult.car_id] = 0;
        }
        
        for(const auto& trackResult : trackResults.second)
        {
            location::Location edge;
            CXYD temp;
            double leftx = trackResult.bounding_rect.x;
            double rightx = trackResult.bounding_rect.x + trackResult.bounding_rect.width;
            if(measures.find(trackResult.car_id) == measures.end())
                continue;
            for(auto& measure_tuple: measures[trackResult.car_id])
            {
                edge.imu = std::get<2>(measure_tuple).imu;
                temp = std::get<2>(measure_tuple).cxy;
                temp.cx = leftx;
                edge.cxy = temp;
                std::get<0>(measure_tuple)[4] = static_cast<PYD>(edge.pyd_imu).yaw;
                temp.cx = rightx;
                edge.cxy = temp;
                std::get<0>(measure_tuple)[5] = static_cast<PYD>(edge.pyd_imu).yaw;
                double yaw_diff = std::remainder(std::get<0>(measure_tuple)[5] - std::get<0>(measure_tuple)[4], 2 * M_PI)/2.0;
                std::get<0>(measure_tuple)[6] = yaw_diff + std::get<0>(measure_tuple)[4];
                std::get<3>(measure_tuple) = true;
            }
        }
        
        for(auto& [carid, measure_vec] : measures)
        {
            std::lock_guard<std::mutex> lock(car_mutex);
            if(cars.find(carid) == cars.end())
            {
                cars[carid] = std::make_unique<MotionModel>();
                cars[carid]->initMotionModel();
            }
            
            if(measure_vec.size() == 1)
                cars[carid]->setUpdateTotalId(std::get<1>(measure_vec[0]));
            else if(measure_vec.size() == 2)
                cars[carid]->setUpdateTotalId(std::get<1>(measure_vec[0]), std::get<1>(measure_vec[1]));
            else if(measure_vec.size() >= 3)
            {
                roslog::error("Impossible measure_vec size: {}", measure_vec.size());
                cars[carid]->setUpdateTotalId(std::get<1>(measure_vec[0]), std::get<1>(measure_vec[1]));
            }

            for(auto& measure: measure_vec)
                if(std::get<3>(measure))
                    cars[carid]->Update(world2model(std::get<0>(measure)), timestamp, std::get<1>(measure));
        }
        
        std::lock_guard<std::mutex> lock(car_mutex);
        for(auto it = detect_count.begin(); it != detect_count.end();)
        {
            if(it->second > MaxMissFrame)
            {
                cars.erase(it->first);
                it = detect_count.erase(it);
            }
            else
            {
                it->second++;
                ++it;
            }
        }
    }

    Prediction Predictor::model2world(const VectorX& state, std::function<VectorY(const VectorX&, int)> measureFunc)
    {
        Prediction prediction;
        XYZ xyz;
        xyz.x = -state[2];
        xyz.y = -state[0];
        xyz.z = (state[8] + state[9]) / 2.0;
        prediction.center = xyz;
        prediction.vx = -state[3];
        prediction.vy = -state[1];
        prediction.z1 = state[8];
        prediction.z2 = state[9];
        prediction.theta = state[4];
        prediction.omega = state[5];
        prediction.r1 = state[6];
        prediction.r2 = state[7];
        for (int i = 0; i < 4; i++)
        {
            VectorY armormeasure = measureFunc(state, i);
            XYZ xyz_armor;
            double dist = armormeasure[2];
            xyz_armor.z = dist * std::sin(armormeasure[0]);
            xyz_armor.x = -dist * std::cos(armormeasure[0]) * std::sin(armormeasure[1]);
            xyz_armor.y = -dist * std::cos(armormeasure[0]) * std::cos(armormeasure[1]);
            prediction.armors[i].center = xyz_armor;
            prediction.armors[i].yaw = M_PI - armormeasure[3];
            prediction.armors[i].id = i;
            prediction.armors[i].theta = prediction.theta + M_PI / 2 * i;
            double see_angle = std::atan2(state[0],state[2]);
            double armor_angle = std::remainder(prediction.armors[i].theta - M_PI / 2 - see_angle, 2 * M_PI);
            if(armor_angle < M_PI / 3 && armor_angle > -M_PI / 3)
            {
                prediction.armors[i].status = Armor::AVAILABLE;
            }
            else
            {
                // This is not a malformed id; it means the armor face is outside
                // the current camera visible-angle gate.
                roslog::debug("Armor id {} unseen at current view angle: {}", i, armor_angle);
                prediction.armors[i].status = Armor::UNSEEN;
            }
        }
        return prediction;
    }

    VectorY Predictor::world2model(const VectorY& measure)
    {
        VectorY measure_model;
        double armor_x = -measure[1];
        double armor_y = -measure[0];
        double z = measure[2];
        measure_model[2] = std::sqrt(armor_x * armor_x + armor_y * armor_y + z * z);
        measure_model[0] = ceres::atan2(z, ceres::sqrt(armor_x * armor_x + armor_y * armor_y));
        measure_model[1] = ceres::atan2(armor_y, armor_x);

        measure_model[3] = M_PI - measure[3];
        measure_model[4] = -0.5 * M_PI - measure[4];
        measure_model[5] = -0.5 * M_PI - measure[5];
        measure_model[6] = -0.5 * M_PI - measure[6];
        return measure_model;
    }

} // namespace predictor
