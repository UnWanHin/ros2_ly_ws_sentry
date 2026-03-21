// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace DETECTOR
{
    class ArmorBBox
    {
    public:
        ArmorBBox() : center(0, 0), rect(0, 0, 0, 0), area(0.0f), confidence(0.0f), color_id(0), tag_id(0)
        {
            for (int i = 0; i < 4; i++)
            {
                corners[i] = cv::Point2f(0, 0); // 初始化 corners 数组
            }
            points.clear(); // 清空 points 向量
        }

        cv::Point2f corners[4];          // 用于pnp
        std::vector<cv::Point2f> points; // 用于平均
        cv::Point2f center;
        cv::Rect rect;
        float area;
        float confidence;
        int color_id;
        int tag_id;
    };

    typedef std::vector<ArmorBBox> ArmorBBoxes;

}