// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <Eigen/Core>
#include <memory>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>
#include <span>

#include <auto_aim_common/DetectionType.hpp>

namespace ly_auto_aim::inline detector
{
struct OpenVINOArmorDetector final
{
    OpenVINOArmorDetector() = default;
    bool Detect(const cv::Mat& src, std::vector<ArmorObject>& objects);
    bool LoadModel(std::string path);
    static void DrawArmors(cv::Mat& canvas, std::span<ArmorObject> armors);

private:
    int dw, dh;
    float rescale_ratio;

    ov::Core core;
    std::shared_ptr<ov::Model> model;
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request;
    ov::Tensor input_tensor;

    std::string input_name;
    std::string output_name;

    Eigen::Matrix<float, 3, 3> transfrom_matrix;
};
}
