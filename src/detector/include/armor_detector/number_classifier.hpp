// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <vector>
#include <string>
#include <stdexcept>
//#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <fmt/printf.h>
#include <fmt/color.h>

#include <auto_aim_common/DetectionType.hpp>
//#include "ArmorCorrector.hpp"

namespace ly_auto_aim::inline detector {

class NumberClassifier {
    private:
        std::shared_ptr<ov::Model> model;
        ov::Core core_;
        ov::CompiledModel compiled_model_;
        ov::InferRequest infer_request_;
    
        int input_size_= 28;
        bool normalize_= true;
        std::vector<std::string> class_names_ = {"1", "2", "3", "4", "5", "0", "7", "6"};// 1-5 + base + outpost + sentry
    
        cv::Mat preprocess(const cv::Mat& image, const std::vector<cv::Point2f> &points);
        std::vector<float> softmax(const float* data, size_t length);
        int postprocess(ov::Tensor& output_tensor);
    
    public:
        // explicit NumberClassifier();
        NumberClassifier() = default;
        ~NumberClassifier() = default;
        [[nodiscard]] bool LoadModel(std::string model_path);
        [[nodiscard]] int Classify(const cv::Mat& image, const std::vector<cv::Point2f>& points);
};

class SVMArmorCorrector final
{
public:
    NumberClassifier Classifier{};

    void Correct(const cv::Mat& frame, std::vector<ArmorObject>& armors)
    {
        for (auto& armor: armors)
        {
            static std::vector<cv::Point2f> temp_points(4);
            temp_points[0] = armor.apex[0];
            temp_points[1] = armor.apex[1];
            temp_points[2] = armor.apex[2];
            temp_points[3] = armor.apex[3];
            // const auto before_type = armor.type;

            const auto mid_type = Classifier.Classify(frame, temp_points);
            armor.type = mid_type;

            // if (mid_type != 0)
            // {
            //     armor.type = mid_type;
            // }else {
            //     armor.type = 6;
            // }
        }
    }
};

/// old model : using classifier.svm
/*

[[nodiscard]] inline cv::Mat GetLUTTable(const float gamma, std::span<std::uint8_t, 256> gammaTableCache)
{
    for (int i = 0; i < 256; ++i)
        gammaTableCache[i] = cv::saturate_cast<std::uint8_t>(
                std::pow(static_cast<float>(i / 255.0), gamma) * 255.0f
        );
    return {1, 256, CV_8UC1, gammaTableCache.data()};
}

class NumberClassifier
{
public:
    static constexpr auto DefaultGamma = 2.5f;

private:
    std::array<std::uint8_t, 256> GammaTableCache{};
    cv::Mat LUTTable{GetLUTTable(DefaultGamma, GammaTableCache)};

    cv::Ptr<cv::ml::SVM> SVMModel;
    cv::HOGDescriptor HOG{
            {32, 32},
            {16, 16},
            {8, 8},
            {8, 8},
            16
    };

public:
    [[nodiscard]] bool LoadModel(const std::string& model_path);

    [[nodiscard]] int Predict(const cv::Mat& frame, const std::vector<cv::Point2f>& corners) const;
};

class SVMArmorCorrector final
{
public:
    NumberClassifier Classifier{};

    void Correct(const cv::Mat& frame, std::vector<ArmorObject>& armors)
    {
        for (auto& armor: armors)
        {
            static std::vector<cv::Point2f> temp_points(4);
            temp_points[0] = armor.apex[0];
            temp_points[1] = armor.apex[1];
            temp_points[2] = armor.apex[2];
            temp_points[3] = armor.apex[3];
            // const auto before_type = armor.type;

            const auto mid_type = Classifier.Predict(frame, temp_points);

            // std::cout << mid_type << std::endl;

            if (mid_type != 0)
            {
                armor.type = mid_type;
            }else {
                armor.type = 6;
            }
        }
    }
};
*/
}
