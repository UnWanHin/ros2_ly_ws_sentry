#include "BuffDetector.hpp"

//xty::



#include <array>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <cstdlib>

namespace {
bool isHighGuiDisplayAvailable() {
    const char* disable_imshow = std::getenv("DISABLE_OPENCV_IMSHOW");
    if (disable_imshow != nullptr && disable_imshow[0] != '\0' && disable_imshow[0] != '0') {
        return false;
    }

    const char* display = std::getenv("DISPLAY");
    const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
    return (display != nullptr && display[0] != '\0') ||
           (wayland_display != nullptr && wayland_display[0] != '\0');
}
} // namespace

namespace power_rune {
// std::mutex MUTEX; //
/**
 * @brief Construct a new BuffDetector::BuffDetector object
 * @param[in] armor         装甲板
 * @param[in] center        中心 R
 */
// BuffDetector::createBuffDetector(const std::string &buff_model_path) {
//     return BuffDetector::buffDetector;
// }


BuffDetector :: BuffDetector(const std::string& red_buff_model_path, const std::string& blue_buff_model_path){
    m_blue_buff_model_path = blue_buff_model_path;            
    blue_core = ov::Core();
    blue_compiled_model = blue_core.compile_model(m_blue_buff_model_path, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    blue_request = blue_compiled_model.create_infer_request();

    m_red_buff_model_path = red_buff_model_path;            
    red_core = ov::Core();
    red_compiled_model = red_core.compile_model(m_red_buff_model_path, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    red_request = red_compiled_model.create_infer_request();
};

/*
 * @brief 检测箭头，装甲板和中心。如果所有检测均成功，则返回 true，否则返回 false。
 * @param[in] Frame        从相机传来的一帧图像，包括图像本身和其时间戳
 * @return true
 * @return false
 */
bool BuffDetector::buffDetect(const cv::Mat& frame, int enemy_color) {
    globalImage_3 = frame.clone(); //xjj
    m_imageShow = frame.clone();
    m_enemy_color = enemy_color;

    //xty::



    m_has_valid_armor = false;
    m_armors.clear();
    m_has_raw_detect = false;
    m_last_boxes.clear();
    m_last_keypoints.clear();
    m_last_class_ids.clear();
    m_last_confidences.clear();
    m_last_confidence = 0.0f;
    std::cout<<m_enemy_color<<"5555555555555"<<std::endl;

    std::cout<<"开始detectArmor"<<std::endl;

    if (findBuffArmor(m_armor) == false) {
        std::cout<<"裝甲板fail"<<std::endl;
        return false;
    }

    //xty::



    m_has_valid_armor = true;
    return true;
    //enemy_color=param::Param param()[config.json.enemy_color];
}

void BuffDetector::drawTargetPoint(const cv::Point2f& point) {
    cv::circle(m_imageShow, point, 4, cv::Scalar (0, 205, 0), 6);
}


//xty::



void BuffDetector::drawDebugOverlay(cv::Mat& image, bool print_coords) const {
    if (image.empty()) {
        return;
    }
    //xty::
    //调试这里关掉文字可以压缩
    //print_coords=false;

    if (m_has_raw_detect) {
        for (size_t i = 0; i < m_last_boxes.size(); ++i) {
            const cv::Rect& box = m_last_boxes[i];
            float score = m_last_confidences[i];
            cv::Scalar color(0, 255, 0); // Green for detection

            // Draw bounding box
            cv::rectangle(image, box, color, 2);
            std::string label = "fan " + std::to_string(score).substr(0, 4);
            cv::putText(image, label, box.tl() + cv::Point(0, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

            // Draw and connect keypoints for each armor
            if (i < m_armors.size()) {
                const auto& a = m_armors[i];
                std::array<cv::Point2f, 6> pts = {a.m_0p, a.m_1p, a.m_2p, a.m_3p, a.m_4p, a.m_5p};
                std::array<std::string, 6> names = {"R", "C", "U", "Rgt", "D", "L"};
                
                // Draw lines connecting keypoints (Fan shape and R-C line)
                // C to 4 outer points
                cv::line(image, pts[1], pts[2], cv::Scalar(255, 255, 0), 1);
                cv::line(image, pts[1], pts[3], cv::Scalar(255, 255, 0), 1);
                cv::line(image, pts[1], pts[4], cv::Scalar(255, 255, 0), 1);
                cv::line(image, pts[1], pts[5], cv::Scalar(255, 255, 0), 1);
                // Outer points connections
                cv::line(image, pts[2], pts[3], cv::Scalar(0, 255, 255), 2);
                cv::line(image, pts[3], pts[4], cv::Scalar(0, 255, 255), 2);
                cv::line(image, pts[4], pts[5], cv::Scalar(0, 255, 255), 2);
                cv::line(image, pts[5], pts[2], cv::Scalar(0, 255, 255), 2);
                // R to C line
                cv::line(image, pts[0], pts[1], cv::Scalar(255, 0, 255), 2);

                for (size_t j = 0; j < pts.size(); ++j) {
                    cv::circle(image, pts[j], 4, cv::Scalar(0, 0, 255), -1);
                    if (print_coords) {
                        std::ostringstream oss;
                        oss << names[j] << "(" << std::fixed << std::setprecision(1) << pts[j].x << "," << pts[j].y << ")";
                        cv::putText(image, oss.str(), pts[j] + cv::Point2f(6.0f, -6.0f), cv::FONT_HERSHEY_SIMPLEX,
                                    0.35, cv::Scalar(255, 255, 255), 1);
                    }
                }
            }
        }
    }

    if (print_coords && m_has_valid_armor) {
        std::cout << "[BUFF] Detected counts: " << m_armors.size() << std::endl;
        for (size_t i = 0; i < m_armors.size(); ++i) {
            const auto& a = m_armors[i];
            std::cout << "[BUFF][" << i << "][BOX] x=" << a.m_box_x << ", y=" << a.m_box_y << std::endl;
        }
    }
}

// 前向声明：将 letterbox 后的坐标映射回原始图像并设置到 Armor
static bool change_scale(const cv::Mat& globalImage, std::vector<cv::Rect>& boxes, std::vector<cv::Point2f>& keypoints, Armor& armor, const float MODEL_IMG_SIZE);

/**
 * @brief 寻找符合装甲板要求的边框灯条，并将其存入一个向量中。成功返回 true，否则返回 false。
 * @param[in] armor         裝甲板
 * @return true
 * @return false
 */
bool BuffDetector::findBuffArmor(Armor& armor) {
    cv::Mat image2show_detect = globalImage_3.clone();  //畫點或框調試用
    // 引入openvino推理
    // 编译模型
    // core = ov::Core
    // Core();
    // compiled_model = core.compile_model(OV_MODEL_PATH, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));
    // request = compiled_model.create_infer_request();
    // 图像预处理
    letterbox_img = letterbox_image(globalImage_3, cv::Size(416, 416)); //dnn--yolo格式
    letterbox_img.convertTo(input_image, CV_32F, 1.0 / 255.0);
    input_image=input_image;//转换后的。
    //创建推理。
    blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(MODEL_IMG_SIZE, MODEL_IMG_SIZE), cv::Scalar(), true);
    static bool s_highgui_available = isHighGuiDisplayAvailable();
    if (s_highgui_available) {
        try {
            cv::imshow("letterbox_img", letterbox_img);
            cv::waitKey(1);
        } catch (const cv::Exception& e) {
            std::cerr << "[BUFF] HighGUI unavailable, disable imshow. reason: " << e.what() << std::endl;
            s_highgui_available = false;
        } catch (const std::exception& e) {
            std::cerr << "[BUFF] Disable imshow due to exception: " << e.what() << std::endl;
            s_highgui_available = false;
        } catch (...) {
            std::cerr << "[BUFF] Disable imshow due to unknown exception." << std::endl;
            s_highgui_available = false;
        }
    }
    /*cv::dnn::blobFromImage(
    letterbox_img,          // 输入图像（经过letterbox预处理的图像）
    1.0 / 255.0,            // 缩放因子（将像素值从[0,255]归一化到[0,1]）
    cv::Size(MODEL_IMG_SIZE, MODEL_IMG_SIZE),  // 目标尺寸（模型要求的输入宽高）
    cv::Scalar(),           // 均值减法（此处为空，表示不做均值去除）
    true                    // 是否交换通道（OpenCV默认图像为BGR，神经网络常需RGB，true表示交换）
)
*/

    if (m_enemy_color == 0) //I am 0->red, hit red buff
    {
        request = red_request;
        compiled_model = red_compiled_model;
    }
    else if (m_enemy_color == 1) //I am 1->blue
    {
        request = blue_request;
        compiled_model = blue_compiled_model;
    }

    // 设置输入张量
    auto input_port = compiled_model.input();//bin
    //推理：
    ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));
    request.set_input_tensor(input_tensor);
    // 推理
    auto start_time = std::chrono::high_resolution_clock::now();
    request.infer(); //推理
    auto end_time = std::chrono::high_resolution_clock::now();
    auto inference_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout<<"inference_time:"<<inference_time<<std::endl;
    // 获取输出张量
    ov::Tensor output_tensor = request.get_output_tensor();
    const float* output_data = output_tensor.data<float>();
    const std::size_t det_stride = 24;
    const std::size_t det_count = output_tensor.get_size() / det_stride;

    struct Candidate {
        float confidence = 0.0f;
        int class_id = -1;
        cv::Rect box;
        std::vector<cv::Point2f> keypoints;
        Armor armor;
    };

    std::vector<Candidate> candidates;
    candidates.reserve(det_count);

    for (std::size_t di = 0; di < det_count; ++di) {
        const float* det = output_data + di * det_stride;
        float confidence = det[4];
        if (confidence < 0.8f) {
            continue;
        }

        float box_x_min = det[0];
        float box_y_min = det[1];
        float box_x_max = det[2];
        float box_y_max = det[3];
        if (box_x_min <= 0 || box_y_min <= 0 || box_x_max >= MODEL_IMG_SIZE || box_y_max >= MODEL_IMG_SIZE) {
            continue;
        }

        Candidate c;
        c.confidence = confidence;
        c.class_id = static_cast<int>(det[5]);
        c.box = cv::Rect(box_x_min, box_y_min, box_x_max - box_x_min, box_y_max - box_y_min);
        c.keypoints.reserve(num_keypoints);
        for (int i = 0; i < num_keypoints; ++i) {
            float keypoint_x = det[6 + i * 3];
            float keypoint_y = det[7 + i * 3];
            c.keypoints.emplace_back(keypoint_x, keypoint_y);
        }

        std::vector<cv::Rect> one_box{c.box};
        std::vector<cv::Point2f> one_kp = c.keypoints;
        if (!change_scale(globalImage_3, one_box, one_kp, c.armor, MODEL_IMG_SIZE)) {
            continue;
        }
        c.box = one_box[0];
        c.keypoints = one_kp;
        candidates.emplace_back(std::move(c));
    }

    if (candidates.empty()) {
        return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.confidence > b.confidence;
    });

    if (candidates.size() > 2) {
        candidates.resize(2);
    }

    m_armors.clear();
    m_last_boxes.clear();
    m_last_keypoints.clear();
    m_last_class_ids.clear();
    m_last_confidences.clear();

    for (const auto& c : candidates) {
        m_armors.push_back(c.armor);
        m_last_boxes.push_back(c.box);
        m_last_class_ids.push_back(c.class_id);
        m_last_confidences.push_back(c.confidence);
        m_last_keypoints.insert(m_last_keypoints.end(), c.keypoints.begin(), c.keypoints.end());
    }
    m_last_confidence = m_last_confidences.front();
    m_has_raw_detect = true;

    armor = m_armors.front();

    // 绘制检测框和关键点（用于本地调试窗口）
    draw_boxes_keypoints(image2show_detect, m_last_boxes, m_last_confidence, m_last_class_ids, m_last_keypoints);
    cv::resize(image2show_detect, image2show_detect, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    // cv::imshow("Detection Result", image2show_detect); 
    // cv::waitKey(1);
    return true;
}

int BuffDetector::getDetectedArmorCount() const {
    return static_cast<int>(m_armors.size());
}

std::vector<cv::Point2f> BuffDetector::getCameraPointsByIndex(std::size_t index) const {
    if (m_armors.empty()) {
        return {};
    }

    if (index >= m_armors.size()) {
        index = 0;
    }

    const auto& a = m_armors[index];
    return {a.m_0p, a.m_1p, a.m_2p, a.m_3p, a.m_4p, a.m_5p};
}


void Armor::setRoiArmorBox(double x, double y, double width, double height) {
    m_box_x = x;
    m_box_y = y;
    m_box_width = width;
    m_box_height = height;
}

void Armor::setRoiArmor(std::vector<cv::Point2f> keypoints) {
    m_0p.x = keypoints[0].x;
    m_0p.y = keypoints[0].y;
    m_1p.x = keypoints[1].x;
    m_1p.y = keypoints[1].y;
    m_2p.x = keypoints[2].x;
    m_2p.y = keypoints[2].y;
    m_3p.x = keypoints[3].x;
    m_3p.y = keypoints[3].y;
    m_4p.x = keypoints[4].x;
    m_4p.y = keypoints[4].y;
    m_5p.x = keypoints[5].x;
    m_5p.y = keypoints[5].y;
    // m_6p.x = keypoints[6].x;
    // m_6p.y = keypoints[6].y;
}

// 定义 Letterbox 缩放函数
cv::Mat BuffDetector::letterbox_image(const cv::Mat& image, const cv::Size& new_shape) {
    int ih = image.rows;
    int iw = image.cols;
    std::cout<<"letterbox_height & width:"<<ih<<", "<<iw<<std::endl;

    int w = new_shape.width;
    int h = new_shape.height;

    float scale = std::min(static_cast<float>(w) / iw, static_cast<float>(h) / ih);
    int nw = static_cast<int>(round(iw * scale));
    int nh = static_cast<int>(round(ih * scale));

    cv::Mat resized_image;
    cv::resize(image, resized_image, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    cv::Mat letterbox_img = cv::Mat::zeros(h, w, image.type());
    int top = (h - nh) / 2;
    int left = (w - nw) / 2;
    resized_image.copyTo(letterbox_img(cv::Rect(left, top, nw, nh)));

    return letterbox_img;
}

// 绘制检测框和关键点
void BuffDetector::draw_boxes_keypoints(cv::Mat& image, const std::vector<cv::Rect>& boxes, const float confidence, const std::vector<int>& class_ids, const std::vector<cv::Point2f>& keypoints) const {
    for (size_t i = 0; i < boxes.size(); ++i) {
        // 获取检测框信息
        const cv::Rect& box = boxes[i];
        int class_id = class_ids[i];
        float score = confidence;

        // 绘制检测框
        cv::rectangle(image, box.tl(), box.br(), cv::Scalar(0, 255, 0), 2);

        // 添加类别和置信度标签
        std::string label = "fan " + std::to_string(score).substr(0, 4);
        putText(image, label, box.tl() + cv::Point(0, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        // 绘制关键点
        for (const auto& kp : keypoints) {
            circle(image, kp, 3, cv::Scalar(239, 4, 25), 5, cv::FILLED);  // 用红色圆圈绘制关键点
        }
    }
}

// 将坐标从模型输入（letterbox 后）映射回原始图像并设置到 Armor
static bool change_scale(const cv::Mat& globalImage, std::vector<cv::Rect>& boxes, std::vector<cv::Point2f>& keypoints, power_rune::Armor& armor, const float MODEL_IMG_SIZE) {
    if (boxes.empty()) {
        std::cout << "change_scale: no boxes provided" << std::endl;
        return false;
    }

    float original_width = static_cast<float>(globalImage.cols);
    float original_height = static_cast<float>(globalImage.rows);
    std::cout << "orig W H: " << original_width << " " << original_height << " model_size: " << MODEL_IMG_SIZE << std::endl;

    float rate = std::min(MODEL_IMG_SIZE / original_width, MODEL_IMG_SIZE / original_height);
    float dx = (MODEL_IMG_SIZE - original_width * rate) / 2.0f;
    float dy = (MODEL_IMG_SIZE - original_height * rate) / 2.0f;

    // 调整检测框坐标
    for (auto& box : boxes) {
        box.x = std::max((box.x - dx) / rate, 0.0f);
        box.y = std::max((box.y - dy) / rate, 0.0f);
        box.width /= rate;
        box.height /= rate;
        std::cout << "转化后 box: " << box.x << ", " << box.y << ", " << box.width << ", " << box.height << std::endl;
        if (box.x < 0 || box.y < 0) {
            std::cout << "change_scale: box out of range" << std::endl;
            return false;
        }
    }

    armor.setRoiArmorBox(boxes[0].x, boxes[0].y, boxes[0].width, boxes[0].height);

    // 调整关键点坐标
    for (auto& kp : keypoints) {
        kp.x = std::max((kp.x - dx) / rate, 0.0f);
        kp.y = std::max((kp.y - dy) / rate, 0.0f);
        if (kp.x <= 0 || kp.y <= 0 || kp.x >= original_width || kp.y >= original_height) {
            std::cout << "change_scale: keypoint out of range" << std::endl;
            return false;
        }
        std::cout << "转化后 keypoint: " << kp.x << ", " << kp.y << std::endl;
    }

    armor.setRoiArmor(keypoints);
    return true;
}

}  // namespace power_rune
