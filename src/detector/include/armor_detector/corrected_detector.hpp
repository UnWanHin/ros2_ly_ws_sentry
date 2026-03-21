// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include "armor_filter.hpp"
#include "armor_detector.hpp"
#include "number_classifier.hpp"


namespace ly_auto_aim::inline detector{

class CorrectedDetector final
{

public:
    OpenVINOArmorDetector Detector{};
    SVMArmorCorrector Corrector{};

    bool Detect(const cv::Mat& src, std::vector<ArmorObject>& objects)
    {
        if (!Detector.Detect(src, objects)) return false;
        Corrector.Correct(src, objects);
        return true;

        /*
        const auto width = src.cols;
        const auto height = src.rows;
        const cv::Rect2i roi(width / 4, height / 4, width / 2, height / 2);
        if (!Detector.Detect(src(roi), objects))
        {
            if (!Detector.Detect(src, objects)) return false;
        }
        else
        {
            const auto tl = roi.tl();
            // roi 的后处理
            for (auto& armor : objects)
            {
                for (auto& point : armor.apex)
                {
                    point.x += tl.x;
                    point.y += tl.y;
                }

                for (auto& point : armor.points)
                {
                    point.x += tl.x;
                    point.y += tl.y;
                }
            }
        }

        Corrector.Correct(src, objects);
        return true;
        */
    }
};
}
