// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "armor_detector/number_classifier.hpp"
//#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>
#include <fmt/format.h>
#include <fmt/color.h>
#include <Logger/Logger.hpp>


namespace ly_auto_aim::inline detector
{

bool NumberClassifier::LoadModel(std::string model_path)
{
    try{
        model = core_.read_model(model_path);
        compiled_model_ = core_.compile_model(model, "CPU");
        infer_request_ = compiled_model_.create_infer_request();
        ov::Shape output_shape = compiled_model_.output().get_shape();

        if (output_shape.back() != class_names_.size()) {
            throw std::invalid_argument("Class count mismatch with model output");
        }
        return true;
    } catch (const std::exception& e) {
        fmt::print(fg(fmt::color::crimson),"NumberClassifier> cannot load model: {}", e.what());
        return false;
    }
}

cv::Mat NumberClassifier::preprocess(const cv::Mat& image, const std::vector<cv::Point2f> &points) {
    if (image.empty()) {
        throw std::runtime_error("Input image is empty");
    }
    cv::Point2f srcPoints[4] = {points[0], points[1], points[2], points[3]};
    float height1 = (srcPoints[3].y - srcPoints[0].y) / 2;
    float height2 = (srcPoints[2].y - srcPoints[1].y) / 2;

    float weight1 = (srcPoints[3].x - srcPoints[0].x) / 2;
    float weight2 = (srcPoints[2].x - srcPoints[1].x) / 2;

    srcPoints[0].x -= weight1/2;
    srcPoints[3].x += weight1/2;
    srcPoints[0].y -= height1/2;
    srcPoints[3].y += height1/2;

    srcPoints[1].x -= weight2/2;
    srcPoints[2].x += weight2/2;
    srcPoints[1].y -= height2/2;
    srcPoints[2].y += height2/2;

    for (int i = 0; i < 4; ++i) {
        srcPoints[i].x = std::max(0.0f, std::min(srcPoints[i].x, static_cast<float>(image.cols)));
        srcPoints[i].y = std::max(0.0f, std::min(srcPoints[i].y, static_cast<float>(image.rows)));
    }

    cv::Point2f dstPoints[4] = {cv::Point2f(0, 0), cv::Point2f(input_size_, 0), cv::Point2f(input_size_, input_size_), cv::Point2f(0, input_size_)};
    cv::Mat M = cv::getAffineTransform(srcPoints, dstPoints);
    cv::Mat affine_img;
    cv::warpAffine(image, affine_img, M, cv::Size(input_size_, input_size_));
    cv::cvtColor(affine_img, affine_img, cv::COLOR_BGR2GRAY);
    if (normalize_) {
        affine_img.convertTo(affine_img, CV_32F, 1.0 / 255.0);
    }
    return cv::dnn::blobFromImage(affine_img); // [1,1,H,W]
}

std::vector<float> NumberClassifier::softmax(const float* data, size_t length) {
    std::vector<float> exp_values(length);
    float max_val = *std::max_element(data, data + length);

    float sum = 0.0f;
    for (size_t i = 0; i < length; ++i) {
        exp_values[i] = std::exp(data[i] - max_val);
        sum += exp_values[i];
    }

    for (auto& val : exp_values) {
        val /= sum;
    }
    return exp_values;
}

int NumberClassifier::postprocess(ov::Tensor& output_tensor) {
    const float* output_data = output_tensor.data<const float>();
    size_t num_classes = output_tensor.get_size();

    auto probabilities = softmax(output_data, num_classes);
    int max_index = std::max_element(probabilities.begin(), probabilities.end()) - probabilities.begin();
    if (max_index < 0 || max_index >= class_names_.size()) {
        throw std::runtime_error("Invalid class index: " + std::to_string(max_index));
    }
    if (probabilities[max_index] < 0.65) {
        return 0;
    }

    return std::stoi(class_names_[max_index]);
}

int NumberClassifier::Classify(const cv::Mat& image,const std::vector<cv::Point2f> &points) {
    cv::Mat input_blob = preprocess(image, points);
    ov::Tensor input_tensor(
        ov::element::f32,
        compiled_model_.input().get_shape(),
        input_blob.ptr<float>()
    );
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.infer();

    ov::Tensor output_tensor = infer_request_.get_output_tensor();
    return postprocess(output_tensor);
}

}
