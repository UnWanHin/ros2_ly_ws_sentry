// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <opencv2/opencv.hpp>

namespace ly_auto_aim::inline detector
{

/// @brief 传统视觉中的装甲板
struct ArmorBlob
{
    ArmorBlob() : corners(4)
    {
    }

    double confidence;
    cv::Rect rect;
    std::vector<cv::Point2f> corners;
    int _class;
    double angle;
    double x, y, z;
    bool is_big_armor;

    [[nodiscard]] double GetSquareDistance() const
    {
        return x * x + y * y + z * z;
    }

    friend bool operator<(const ArmorBlob& left, const ArmorBlob& right)
    {
        return left.GetSquareDistance() < right.GetSquareDistance();
    }
};

using ArmorBlobs = std::vector<ArmorBlob>;

// /// @brief 机器学习中的装甲板
// struct ArmorObject
// {
//     enum ColorEntry {
//         Blue = 0,
//         Red = 1,
//         Gray = 2,
//         Others
//     };

//     cv::Rect2f Rectangular;
//     int type;
//     int color;
//     float prob;
//     std::vector<cv::Point2f> points;
//     int area;
//     cv::Point2f apex[4];

//     ColorEntry ActualColor() const {
//         switch (color / 2) {
//             case 0:
//                 return Blue;
//             case 1:
//                 return Red;
//             case 2:
//                 return Gray;
//             default:
//                 return Others;
//         }
//     }

//     bool IsLarge() const {
//         return color % 2 == 1;
//     }
// };

/// @brief 机器学习中的装甲板
/// @brief 新模型不再直接推断type，而只是一个四点模型
struct ArmorObject{
    enum ColorEntry {
        Blue = 0,
        Red = 1
    };

    cv::Rect2f Rectangular;
    int type;
    int color;
    float prob;
    std::vector<cv::Point2f> points;
    int area;
    cv::Point2f apex[4];


    ColorEntry ActualColor() const {
        if(color == 0){
            return Blue;
        } 
        return Red;
    }
    
    bool IsLarge() const 
    {
        return type == 1 || type == 0;
    }
};

struct Detection
{
    cv::Rect2f bounding_rect;
    cv::Point2f center;
    int tag_id;
    float score;
    std::vector<cv::Point2f> corners;
};
using Detections = std::vector<Detection>;   /// 可能还需要知道一个center和isGray(先不管了后面记得！不管了！！)

/// @brief 车辆检测中的装甲板
struct CarDetection
{
    cv::Rect2f bounding_rect;
    cv::Point2f center;
    int tag_id;// will be used for recognize Car's Model
    //but now it's not used
    float score;
};
using CarDetections = std::vector<CarDetection>;
/// @brief 机器学习中的类型--地面作战单位
enum class ArmorType : std::uint8_t
{
    Base   = 0,
    Hero      = Base + 1,
    Engineer  = Hero + 1,
    Infantry1 = Engineer + 1,
    Infantry2 = Infantry1 + 1,
    Infantry3 = Infantry2 + 1,
    Sentry     = Infantry3 + 1,
    Outpost    = Sentry + 1,
    UnKnown    = Outpost + 1
};

}
