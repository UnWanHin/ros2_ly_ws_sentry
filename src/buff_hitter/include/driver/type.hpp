// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <iostream>
#include <string>
#include <memory>
#include <functional>
#include <opencv2/opencv.hpp>
#include "TimeStamp/TimeStamp.hpp"

//前向声明
namespace controller
{
    struct ControlResult;
}

namespace driver
{
    struct SerialConfig
    {
        std::string portName;
        int baudRate;
    };

    struct CameraConfig
    {
        enum LightSource
        {
            GX_AWB_LAMP_HOUSE_ADAPTIVE = 0, // Auto adaptive mode
            GX_AWB_LAMP_HOUSE_FLUORESCENCE = 1, // Fluorescent lamp mode
            GX_AWB_LAMP_HOUSE_INCANDESCENT = 2, // Incandescent lamp mode
            GX_AWB_LAMP_HOUSE_U30 = 3, //3000K lamp mode
            GX_AWB_LAMP_HOUSE_D50 = 4, //5000K lamp mode
            GX_AWB_LAMP_HOUSE_D65 = 5, //6500K lamp mode
            GX_AWB_LAMP_HOUSE_D75 = 6 //7000K lamp mode
        };
        std::string cameraSN;
        bool autoExposure = false;
        bool autoGain = false;
        bool autoWhiteBalance = false;
        double exposureTime = 5000;
        double autoExposureTimeMin = 3000;
        double autoExposureTimeMax = 8000;
        double gain = 12;
        double autoGainMin = 6;
        double autoGainMax = 18;
        int64_t grayValueMin = 100;
        int64_t grayValueMax = 200;
        float balanceRatioRed = 1.0;
        float balanceRatioBlue = 1.0;
        float balanceRatioGreen = 1.0;
        bool triggerSourceLine2 = true;
        LightSource lightSource = GX_AWB_LAMP_HOUSE_ADAPTIVE;
    };

    struct RawSerialData
    {
        int8_t start_flag;
        float pitch_now;
        float yaw_now;
        float roll_now;
        float actual_bullet_speed;
        uint8_t available_shoot_number;
        uint8_t aim_request;
        uint8_t mode_want;
        uint8_t number_want;
        uint8_t enemy_color; // 0:red 1:blue
        uint16_t crc16;
    }__attribute__((packed));

    struct ParsedSerialData
    {
        int8_t start_flag;
        float pitch_now;
        float yaw_now;
        float roll_now;
        float actual_bullet_speed;
        uint8_t available_shoot_number;
        uint8_t aim_request;
        uint8_t mode_want;
        uint8_t number_want;
        uint8_t enemy_color; // 0:red 1:blue

        Time::TimeStamp timestamp;

        ParsedSerialData(const RawSerialData& x) : start_flag(x.start_flag), pitch_now(x.pitch_now), yaw_now(x.yaw_now), roll_now(x.roll_now), 
            actual_bullet_speed(x.actual_bullet_speed), aim_request(x.aim_request), mode_want(x.mode_want), available_shoot_number(x.available_shoot_number),
            number_want(x.number_want), enemy_color(x.enemy_color), timestamp(Time::TimeStamp()) {};
        ParsedSerialData() {};
    };

    struct RawSerialWriteData
    {
        int8_t start_flag;
        uint8_t detect_number;
        uint8_t shoot_flag;
        float pitch_setpoint;
        float yaw_setpoint;
        uint16_t crc16;
        RawSerialWriteData(const controller::ControlResult& x); 
    }__attribute__((packed));

    struct TimeImageData
    {
        cv::Mat image;
        Time::TimeStamp timestamp;
    };


}