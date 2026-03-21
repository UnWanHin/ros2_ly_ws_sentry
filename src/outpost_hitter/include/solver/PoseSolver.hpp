// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

// [ROS 2] 核心頭文件
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <auto_aim_common/DetectionType.hpp>
#include <vector>
#include <Logger/Logger.hpp>
#include <detector/BBoxes.h>

// [關鍵] 包含此頭文件以識別 ArmorPoses
#include "solver.hpp"
#include "detector/detector.hpp"

using namespace cv;
using namespace std;
using namespace DETECTOR;
using namespace ly_auto_aim;

namespace SOLVER
{
    // [全局節點聲明]
    // 用於在不改變 PoseSolver 構造函數參數的情況下傳遞節點指針
    // 記得在 PoseSolver.cpp 中定義它，在 main 函數中賦值
    extern rclcpp::Node::SharedPtr global_solver_node;

    class PoseSolver
    {
    public:
        // [ROS 2 適配] 保持無參構造函數
        explicit PoseSolver()
        {
            // 1. 獲取全局節點
            if (!global_solver_node) {
                throw std::runtime_error("SOLVER::global_solver_node is not initialized! Please set it in main().");
            }
            auto node = global_solver_node;

            // ROS 1: ros::NodeHandle nh("solver_config");
            // ROS 2: 參數名通常為 "命名空間.參數名"
            std::string ns_prefix = "solver_config.";

            // -----------------------------------------------------------
            // 2. 讀取相機內參 (Camera Intrinsic)
            // -----------------------------------------------------------
            std::string intrinsic_name = ns_prefix + "camera_intrinsic_matrix";
            std::vector<double> intrinsic_flat;

            // 必須先聲明參數
            if (!node->has_parameter(intrinsic_name)) {
                node->declare_parameter(intrinsic_name, std::vector<double>({}));
            }

            // 獲取參數
            if (!node->get_parameter(intrinsic_name, intrinsic_flat) || intrinsic_flat.size() != 9) {
                RCLCPP_ERROR(node->get_logger(), "Invalid camera_intrinsic_matrix format: %s", intrinsic_name.c_str());
                throw std::runtime_error("Parameter loading failed: camera_intrinsic_matrix");
            }

            intrinsicMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    intrinsicMatrix.at<double>(i, j) = intrinsic_flat[i * 3 + j];
                }
            }

            fx_ = intrinsicMatrix.at<double>(0, 0);
            fy_ = intrinsicMatrix.at<double>(1, 1);
            u0_ = intrinsicMatrix.at<double>(0, 2);
            v0_ = intrinsicMatrix.at<double>(1, 2);

            // -----------------------------------------------------------
            // 3. 讀取畸變係數 (Distortion Coefficients)
            // -----------------------------------------------------------
            std::string distortion_name = ns_prefix + "camera_distortion_coefficients";
            std::vector<double> distortion_flat;

            if (!node->has_parameter(distortion_name)) {
                node->declare_parameter(distortion_name, std::vector<double>({}));
            }

            if (!node->get_parameter(distortion_name, distortion_flat) || distortion_flat.size() != 5) {
                RCLCPP_ERROR(node->get_logger(), "Invalid camera_distortion_coefficients format: %s", distortion_name.c_str());
                throw std::runtime_error("Parameter loading failed: camera_distortion_coefficients");
            }

            distortionCoefficients = cv::Mat(1, 5, CV_64F, distortion_flat.data()).clone();
            
            RCLCPP_INFO(node->get_logger(), "PoseSolver parameters loaded successfully.");
        }

        ~PoseSolver() = default;

        ArmorPoses solveArmorPoses(const ly_auto_aim::detector::Detections &, const float &, const float &);

        void setCameraMatrix(double fx, double fy, double u0, double v0, double k1, double k2, double p1, double p2, double k3)
        {
            // 相机内参
            intrinsicMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
            intrinsicMatrix.ptr<double>(0)[0] = fx;
            intrinsicMatrix.ptr<double>(0)[2] = u0;
            intrinsicMatrix.ptr<double>(1)[1] = fy;
            intrinsicMatrix.ptr<double>(1)[2] = v0;
            intrinsicMatrix.ptr<double>(2)[2] = 1.0f;

            // 畸变系数
            distortionCoefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
            distortionCoefficients.ptr<double>(0)[0] = k1;
            distortionCoefficients.ptr<double>(1)[0] = k2;
            distortionCoefficients.ptr<double>(2)[0] = p1;
            distortionCoefficients.ptr<double>(3)[0] = p2;
            distortionCoefficients.ptr<double>(4)[0] = k3;

            this->fx_ = fx;
            this->fy_ = fy;
            this->u0_ = u0;
            this->v0_ = v0;
        }

    private:
        //////////////////////////////////////////////////
        // 相机参数：通过标定获取
        cv::Mat intrinsicMatrix;
        cv::Mat distortionCoefficients;
        double fx_, fy_, u0_, v0_;

        cv::Mat rvec; // 旋转向量 OpenCV
        cv::Mat tvec; // 平移向量 OpenCV

        //////////////////////////////////////////////////
        // 装甲板的物理信息，用于PNP解算
        float length_of_small = 0.0675f;
        float height_of_small = 0.0275f;
        float length_of_big = 0.1125f;
        float height_of_big = 0.0275f;

        // 小装甲板3d坐标
        vector<Point3f> points_small_3d = {Point3f(-length_of_small, -height_of_small, 0.f),
                                           Point3f(length_of_small, -height_of_small, 0.f),
                                           Point3f(length_of_small, height_of_small, 0.f),
                                           Point3f(-length_of_small, height_of_small, 0.f)};

        //  大装甲板3d坐标
        vector<Point3f> points_large_3d = {Point3f(-length_of_big, -height_of_big, 0.f),
                                           Point3f(length_of_big, -height_of_big, 0.f),
                                           Point3f(length_of_big, height_of_big, 0.f),
                                           Point3f(-length_of_big, height_of_big, 0.f)};
        //////////////////////////////////////////////////

        void point2Angle(const cv::Point2f &, float &, float &);
    };
} // namespace SOLVER