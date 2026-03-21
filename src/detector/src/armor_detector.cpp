// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "armor_detector/armor_detector.hpp"
#include <algorithm>
//#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <fmt/color.h>

#pragma region parameter

constexpr int InputWidth = 416;
constexpr int InputHeight = InputWidth;
constexpr int ClassCount = 1;
constexpr int ColorCount = 2;
constexpr int TopK = 128;
constexpr float NMSThresh = 0.3f;
constexpr float BBOXConfidenceThresh = 0.5f;
constexpr float MergeConfidenceThresh = 0.15f;
constexpr float MergeMinIOU = 0.9f;

#pragma endregion parameter

#pragma region detect
namespace ly_auto_aim::inline detector {
struct GridAndStride {
    int grid0;
    int grid1;
    int stride;
};

int FindIndexOfMaxNumber(const float* ptr, const int len) {
    const std::span span{ptr, static_cast<std::size_t>(len)};
    return static_cast<int>(std::ranges::max_element(span) - span.begin());
}

inline cv::Mat ScaledResize(const cv::Mat& img, Eigen::Matrix<float, 3, 3>& transformMatrix) {
    const float ratio = std::min<float>(
            static_cast<float>(InputWidth) / (static_cast<float>(img.cols) * 1.0f),
            static_cast<float>(InputHeight) / (static_cast<float>(img.rows) * 1.0f));

    const int width_without_padding = static_cast<int>(static_cast<float>(img.cols) * ratio);
    const int height_without_padding = static_cast<int>(static_cast<float>(img.rows) * ratio);
    const int delta_width = (InputWidth - width_without_padding) / 2;
    const int delta_height = (InputHeight - height_without_padding) / 2;

    transformMatrix << 1.0f / ratio, 0, -static_cast<float>(delta_width) / ratio,
            0, 1.0f / ratio, -static_cast<float>(delta_height) / ratio,
            0, 0, 1;

    cv::Mat re{};
    resize(img, re, cv::Size(width_without_padding, height_without_padding));
    cv::Mat out{};
    copyMakeBorder(
            re, out,
            delta_height, delta_height, delta_width, delta_width,
            cv::BORDER_CONSTANT
    );
    return out;
}

void GenerateGridsFromStride(const int width, const int height,
                             const std::vector<int>& strides, std::vector<GridAndStride>& grids) {
    for (const auto stride : strides) {
        const int num_grid_width = width / stride;
        const int num_grid_height = height / stride;

        for (int g1 = 0; g1 < num_grid_height; g1++) {
            for (int g0 = 0; g0 < num_grid_width; g0++) { grids.emplace_back(g0, g1, stride); }
        }
    }
}

/**
 * @brief Generate Proposal
 * @param grids Grid strides
 * @param feat_ptr Original predition result.
 * @param prob_threshold Confidence Threshold.
 * @param objects Objects proposed.
 */
void GenerateYoloXProposals(const std::vector<GridAndStride>& grids, const float* feat_ptr,
                            const Eigen::Matrix<float, 3, 3>& transform_matrix, const float prob_threshold,
                            std::vector<ArmorObject>& objects) {
    const int num_anchors = grids.size();
    //Travel all the anchors
    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
        const int grid0 = grids[anchor_idx].grid0;
        const int grid1 = grids[anchor_idx].grid1;
        const int stride = grids[anchor_idx].stride;
        const int basic_pos = anchor_idx * (9 + (ColorCount) + ClassCount);

        float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
        float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
        float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
        float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
        float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
        float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
        float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
        float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;

        const float box_objectness = (feat_ptr[basic_pos + 8]);
        const int box_color = FindIndexOfMaxNumber(feat_ptr + basic_pos + 9, ColorCount);
        const int box_class = FindIndexOfMaxNumber(feat_ptr + basic_pos + 9 + ColorCount, ClassCount);
        const float box_prob = box_objectness;
        if (box_prob >= prob_threshold) {
            ArmorObject obj;

            Eigen::Matrix<float, 3, 4> apex_norm;
            Eigen::Matrix<float, 3, 4> apex_dst;

            apex_norm << x_1, x_2, x_3, x_4,
                    y_1, y_2, y_3, y_4,
                    1, 1, 1, 1;

            apex_dst = transform_matrix * apex_norm;

            for (int i = 0; i < 4; i++) {
                obj.apex[i] = cv::Point2f(apex_dst(0, i), apex_dst(1, i));
                obj.points.push_back(obj.apex[i]);
            }

            std::vector tmp(obj.apex, obj.apex + 4);
            obj.Rectangular = boundingRect(tmp);
            obj.type = box_class;
            obj.color = box_color;
            obj.prob = box_prob;

            objects.push_back(obj);
        }
    }
}

float IntersectionArea(const ArmorObject& a, const ArmorObject& b) {
    return (a.Rectangular & b.Rectangular).area();
}

void qsort_descent_inplace(std::vector<ArmorObject>& face_objects, int left, int right) {
    int i = left;
    int j = right;
    const float p = face_objects[(left + right) / 2].prob;

    while (i <= j) {
        while (face_objects[i].prob > p)
            i++;

        while (face_objects[j].prob < p)
            j--;

        if (i <= j) {
            // swap
            std::swap(face_objects[i], face_objects[j]);
            i++;
            j--;
        }
    }
    if (left < j) qsort_descent_inplace(face_objects, left, j);
    if (i < right) qsort_descent_inplace(face_objects, i, right);
}

void qsort_descent_inplace(std::vector<ArmorObject>& objects) {
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

void nms_sorted_bboxes(std::vector<ArmorObject>& face_objects, std::vector<int>& picked, float nms_threshold) {
    picked.clear();
    const int n = face_objects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++) { areas[i] = face_objects[i].Rectangular.area(); }

    for (int i = 0; i < n; i++) {
        ArmorObject& a = face_objects[i];
        int keep = 1;
        for (int j : picked) {
            ArmorObject& b = face_objects[j];
            const float inter_area = IntersectionArea(a, b);
            const float union_area = areas[i] + areas[j] - inter_area;
            const float iou = inter_area / union_area;
            if (iou > nms_threshold || isnan(iou)) {
                keep = 0;
                if (iou > MergeMinIOU && abs(a.prob - b.prob) < MergeConfidenceThresh
                    && a.type == b.type && a.color == b.color) {
                    for (const auto& i : a.apex)
                        b.points.push_back(i);
                }
                // cout<<b.pts_x.size()<<endl;
            }
        }
        if (keep)
            picked.push_back(i);
    }
}

void DecodeOutputs(const float* prob, std::vector<ArmorObject>& objects,
                   const Eigen::Matrix<float, 3, 3>& transform_matrix) {
    std::vector<ArmorObject> proposals;
    const std::vector strides = {8, 16, 32};
    std::vector<GridAndStride> grid_strides;

    GenerateGridsFromStride(InputWidth, InputHeight, strides, grid_strides);
    GenerateYoloXProposals(grid_strides, prob, transform_matrix, BBOXConfidenceThresh, proposals);
    qsort_descent_inplace(proposals);
    if (proposals.size() >= TopK)
        proposals.resize(TopK);
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, NMSThresh);
    const int count = picked.size();
    objects.resize(count);
    for (int i = 0; i < count; i++) { objects[i] = proposals[picked[i]]; }
}

float GetAreaOfTriangle(const cv::Point2f points[3]) {
    const auto& p0 = points[0];
    const auto& p1 = points[1];
    const auto& p2 = points[2];
    const auto delta_p01 = p0 - p1;
    const auto delta_p12 = p1 - p2;
    const auto delta_p20 = p2 - p0;
    const auto a = std::sqrt(delta_p01.x * delta_p01.x + delta_p01.y * delta_p01.y);
    const auto b = std::sqrt(delta_p12.x * delta_p12.x + delta_p12.y * delta_p12.y);
    const auto c = std::sqrt(delta_p20.x * delta_p20.x + delta_p20.y * delta_p20.y);
    const auto p = (a + b + c) / 2.0f;
    return std::sqrt(p * (p - a) * (p - b) * (p - c));
}

float GetAreaOfTetragon(cv::Point2f points[4]) { return GetAreaOfTriangle(points) + GetAreaOfTriangle(points + 1); }

bool OpenVINOArmorDetector::LoadModel(std::string path) {
//    spdlog::info("ArmorDetector> loading model from path({})", path)
    fmt::print("ArmorDetector> loading model from path({})", path);
    const std::string device_name{"CPU"};

#ifdef _LINUX
    this->core.set_property(device_name, ov::enable_profiling(false));
#endif

    try { model = core.read_model(path); }
    catch (const std::exception& ex) {
//        spdlog::error("ArmorDetector> cannot load model: {}", ex.what());
        fmt::print(fg(fmt::color::crimson) | fmt::emphasis::bold, "ArmorDetector> cannot load model: {}", ex.what());
        return false;
    }
    catch (...) {
//        spdlog::error("ArmorDetector> cannot load model: unknown error");
        fmt::print(fg(fmt::color::crimson) | fmt::emphasis::bold, "ArmorDetector> cannot load model: unknown error");
        return false;
    }

    ov::preprocess::PrePostProcessor ppp(model);
    ppp.input().tensor().set_element_type(ov::element::f32);
    ppp.output().tensor().set_element_type(ov::element::f32);
    ppp.build();
    compiled_model = core.compile_model(
            model, device_name,
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
    );
    infer_request = compiled_model.create_infer_request();
    return true;
}

bool OpenVINOArmorDetector::Detect(const cv::Mat& src, std::vector<ArmorObject>& objects) {
    if (src.empty()) return false;
    const cv::Mat pr_img = ScaledResize(src, transfrom_matrix);
    cv::Mat pre;
    cv::Mat pre_split[3];
    pr_img.convertTo(pre, CV_32F);
    split(pre, pre_split);
    input_tensor = infer_request.get_input_tensor(0);
    infer_request.set_input_tensor(input_tensor);
    float* tensor_data = input_tensor.data<float_t>();
    constexpr auto img_offset = InputHeight * InputWidth;
    for (const auto& mat : pre_split) {
        memcpy(tensor_data, mat.data, sizeof(float) * InputHeight * InputWidth);
        tensor_data += img_offset;
    }
    infer_request.infer();
    ov::Tensor output_tensor = infer_request.get_output_tensor();
    const float* output = output_tensor.data<float_t>();
    DecodeOutputs(output, objects, transfrom_matrix);
    for (auto& [rect, cls, color, prob, pts, area, apex] : objects) {
        if (pts.size() >= 8) {
            const auto count = pts.size();
            cv::Point2f pts_final[4];
            for (int i = 0; i < static_cast<int>(count); i++) { pts_final[i % 4] += pts[i]; }

            for (auto& i : pts_final) {
                i.x = i.x / (static_cast<float>(count) / 4.0f);
                i.y = i.y / (static_cast<float>(count) / 4.0f);
            }

            apex[0] = pts_final[0];
            apex[1] = pts_final[1];
            apex[2] = pts_final[2];
            apex[3] = pts_final[3];
        }
        area = static_cast<int>(GetAreaOfTetragon(apex));
    }

    return !objects.empty();
}

const char* GetColorText(const int id) {
    if (id > 1) return "color";
    constexpr const char* ColorTextTable[] = {"Blue", "Red"};
    return ColorTextTable[id];
}

const char* GetTypeText(const int id) {
    if (id > 7) return "type";
    static constexpr const char* const TypeTextTable[] = {"Sentry", "1", "2", "3", "4", "5", "OutPost", "Base"};
    return TypeTextTable[id];
}


void OpenVINOArmorDetector::DrawArmors(cv::Mat& canvas, const std::span<ArmorObject> armors) {
    for (const auto& [rect, cls, color, prob, pts, area, apex] : armors) {
        for (int i = 0; i < 4; i++) { line(canvas, apex[i % 4], apex[(i + 1) % 4], cv::Scalar(0, 0, 255), 2); }
        std::stringstream stream;
        stream << GetColorText(color) << " " << GetTypeText(cls) << " : " << prob * 100;
        putText(canvas, stream.str(), apex[0],
                cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 0));
    }
}
}

#pragma endregion detect
