// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace power_rune {

/**
 * @brief 装甲板
 */
struct Armor {
    Armor() = default;
    void setRoiArmorBox(double x, double y, double width, double height);
    void setRoiArmor(std::vector<cv::Point2f> keypoints);

    cv::Point2f m_0p;     // R标
    cv::Point2f m_1p;     // 裝甲板正中心
    cv::Point2f m_2p;     // 最遠點
    cv::Point2f m_3p;     // 右點
    cv::Point2f m_4p;     // 下點
    cv::Point2f m_5p;     // 左點

    double m_box_x;
    double m_box_y;
    double m_box_width;
    double m_box_height;
};


/**
 * @brief 检测类，负责对图像的处理和目标的检测，得到所有特征点的像素坐标，以及点亮的装甲板数目。
 */
class BuffDetector {
    public:
        explicit BuffDetector(const std::string& buff_model_path);
        bool buffDetect(const cv::Mat& frame);
        void drawTargetPoint(const cv::Point2f& point);
        inline void visualize() { 
            // cv::resize(m_imageShow, m_imageShow, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
            // cv::imshow("visualized", m_imageShow); 
        }
        bool findBuffArmor(Armor& armor);

        /**
         * @brief
         * 得到像素坐标系特征点，分别为装甲板内灯条的 中心R, 裝甲板中心, 上, 右, 下, 左。
         * @return std::vector<cv::Point2f>
         */
        inline std::vector<cv::Point2f> getCameraPoints() {//DONE 5.5
            // return {m_armor.m_0p,                   m_armor.m_2p,
            //         m_armor.m_3p, m_armor.m_4p, m_armor.m_5p,
            //         m_armor.m_6p};
            return {m_armor.m_0p, m_armor.m_1p, m_armor.m_2p,
                    m_armor.m_3p, m_armor.m_4p, m_armor.m_5p};
        }

    private:
        // OpenVINO 模型
        std::string m_buff_model_path;
        ov::Core core;
        ov::CompiledModel compiled_model;
        ov::InferRequest request;
        cv::Mat globalImage_3;   // 全局的3通道原圖
        cv::Mat letterbox_img;
        cv::Mat input_image;
        cv::Mat blob;

        int num_keypoints = 6;  // 关键点数量
        // std::vector<int> class_ids;
        // float confidence;
        // std::vector<cv::Rect> boxes;
        // std::vector<cv::Point2f> keypoints;
        bool is_out = false;

        const double MODEL_IMG_SIZE = 416; 
        float original_width;
        float original_height;

        cv::Mat m_imageShow;     // 可视化图片

        Armor m_armor;           // 装甲板
        std::chrono::steady_clock::time_point m_startTime;  // 检测开始的时间戳
        std::chrono::steady_clock::time_point m_frameTime;  // 当前帧的时间戳
        
        cv::Mat letterbox_image(const cv::Mat& image, const cv::Size& new_shape); //Letterbox 缩放函数
        void draw_boxes_keypoints(cv::Mat& image, const std::vector<cv::Rect>& boxes, const float confidence, const std::vector<int>& class_ids, const std::vector<cv::Point2f>& keypoints); // 绘制检测框和关键点
};

}  // namespace power_rune
