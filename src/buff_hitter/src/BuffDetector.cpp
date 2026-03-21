// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "BuffDetector.hpp"
#include <filesystem>
#include <stdexcept>

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
BuffDetector::BuffDetector(const std::string& buff_model_path){
    m_buff_model_path = buff_model_path;            
    if (!std::filesystem::exists(m_buff_model_path)) {
        throw std::runtime_error("BuffDetector model file not found: " + m_buff_model_path);
    }
    core = ov::Core();
    compiled_model = core.compile_model(m_buff_model_path, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    request = compiled_model.create_infer_request();
}

/**
 * @brief 检测箭头，装甲板和中心。如果所有检测均成功，则返回 true，否则返回 false。
 * @param[in] Frame        从相机传来的一帧图像，包括图像本身和其时间戳
 * @return true
 * @return false
 */
bool BuffDetector::buffDetect(const cv::Mat& frame) {
    globalImage_3 = frame.clone(); //xjj
    m_imageShow = frame.clone();

    std::cout<<"开始detectArmor"<<std::endl;

    if (findBuffArmor(m_armor) == false) {
        std::cout<<"裝甲板fail"<<std::endl;
        return false;
    }
    return true;
}


void BuffDetector::drawTargetPoint(const cv::Point2f& point) {
    cv::circle(m_imageShow, point, 4, cv::Scalar (0, 205, 0), 6);
}


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
    // core = ov::Core();
    // compiled_model = core.compile_model(OV_MODEL_PATH, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));
    // request = compiled_model.create_infer_request();
    // 图像预处理
    letterbox_img = letterbox_image(globalImage_3, cv::Size(416, 416)); 
    letterbox_img.convertTo(input_image, CV_32F, 1.0 / 255.0);
    blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(MODEL_IMG_SIZE, MODEL_IMG_SIZE), cv::Scalar(), true);
    // cv::imshow("letterbox_img", letterbox_img);
    // 设置输入张量
    auto input_port = compiled_model.input();
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
    // 提取置信度最高的检测结果
    std::vector<float> highest_result(output_data, output_data + 24);  // 假设每个检测结果有21个值
    // 解析检测框和关键点
    // int num_keypoints = 6;  // 关键点数量
    std::vector<int> class_ids;
    float confidence = highest_result[4];
    if(confidence < 0.8) {
        // cv::resize(image2show_detect, image2show_detect, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
        // cv::imshow("Detection Result", image2show_detect); 
        // cv::resize(m_imageShow, m_imageShow, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
        // cv::imshow("visualized", m_imageShow); 
        // cv::waitKey(1);
        return false;
    }
    std::vector<cv::Rect> boxes;
    std::vector<cv::Point2f> keypoints;
    
    // bool is_out = false;

    // 提取目标框坐标、置信度、类别ID和关键点信息
    float box_x_min = highest_result[0];
    float box_y_min = highest_result[1];
    float box_x_max = highest_result[2];
    float box_y_max = highest_result[3];
    if(box_x_min <= 0 || box_y_min <= 0 || box_x_max >= MODEL_IMG_SIZE || box_y_max >= MODEL_IMG_SIZE) {
        is_out = true;
        std::cout<<"out of image range (416)!!!!"<<std::endl;
        return false;
    }

    int class_id = static_cast<int>(highest_result[5]);

    for (int i = 0; i < num_keypoints; ++i) {
        float keypoint_x = highest_result[6 + i * 3];
        float keypoint_y = highest_result[7 + i * 3];
        float keypoint_score = highest_result[8 + i * 3];
        keypoints.push_back(cv::Point2f(keypoint_x, keypoint_y));
    }
    // 保存到对应的变量中
    class_ids.push_back(class_id);
    boxes.push_back(cv::Rect(box_x_min, box_y_min, box_x_max - box_x_min, box_y_max - box_y_min));
    std::cout<<"keybox:"<<box_x_min<<", "<<box_y_min<<", "<<box_x_max - box_x_min<<", "<<box_y_max - box_y_min<<std::endl;
    std::cout<<"keypoints:"<<std::endl;
    for(const auto& keypoint : keypoints){
        std::cout<<keypoint.x<<", "<<keypoint.y<<std::endl;
        cv::circle(letterbox_img, cv::Point2f(keypoint.x, keypoint.y), 2, (255,255,255), 2);
    }
    // cv::imshow("drawkeypoint", letterbox_img);

    // 坐标转换：从缩放后的图像尺寸转换回原始图像尺寸
    original_width = static_cast<float>(globalImage_3.cols);
    original_height = static_cast<float>(globalImage_3.rows);
    //letterbox_width = letterbox_height = MODEL_IMG_SIZE = 416
    std::cout<<original_width<<original_height<<MODEL_IMG_SIZE<<MODEL_IMG_SIZE<<std::endl;
    float rate = std::min(MODEL_IMG_SIZE / original_width, MODEL_IMG_SIZE / original_height);
    float dx = (MODEL_IMG_SIZE - original_width * rate) / 2.0f;
    float dy = (MODEL_IMG_SIZE - original_height * rate) / 2.0f;
    // 调整检测框坐标
    for (auto& box : boxes) {
        box.x = std::max((box.x - dx) / rate, 0.0f);
        box.y = std::max((box.y - dy) / rate, 0.0f);
        box.width /= rate;
        box.height /= rate;
        std::cout<<"經轉換后的keybox:"<<box.x<<", "<<box.y<<", "<<box.width<<", "<<box.height<<", 面积："<<box.width*box.height<<std::endl;
        if(box.x < 0 || box.y < 0 ){
            is_out = true;
            std::cout<<"out of image range!!!!"<<std::endl;
            return false;}
    }
    std::cout<<"boxes' size:"<<boxes.size()<<std::endl;
    armor.setRoiArmorBox(boxes[0].x, boxes[0].y, boxes[0].width, boxes[0].height);
    
    // 调整关键点坐标
    for (auto& kp : keypoints) {
        kp.x = std::max((kp.x - dx) / rate, 0.0f);
        kp.y = std::max((kp.y - dy) / rate, 0.0f);
        if(kp.x <= 0 || kp.y <= 0 || kp.x >= original_width || kp.y >= original_height) {
            is_out = true;
            std::cout<<"out of image range!!!!"<<std::endl;
            return false;
        }
        std::cout<<"經轉換后的keypoint:"<<kp.x<<", "<<kp.y<<std::endl;
    }
    armor.setRoiArmor(keypoints);

    // 绘制检测框和关键点
    draw_boxes_keypoints(image2show_detect, boxes, confidence, class_ids, keypoints);
    cv::resize(image2show_detect, image2show_detect, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    // cv::imshow("Detection Result", image2show_detect); 
    // cv::waitKey(1);
    return true;
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
void BuffDetector::draw_boxes_keypoints(cv::Mat& image, const std::vector<cv::Rect>& boxes, const float confidence, const std::vector<int>& class_ids, const std::vector<cv::Point2f>& keypoints) {
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

}  // namespace power_rune
