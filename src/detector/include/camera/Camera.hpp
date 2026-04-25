// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <opencv2/opencv.hpp>
#include <memory>

#include <camera/GxAPI.h>
#include <camera/DxImageProc.h>

#include <RosTools/RosTools.hpp>

namespace LangYa {

    void ConvertRaw8ToRGB24(const auto& raw8Image, cv::Mat& rgbImage) /*noexcept*/
    {
        rgbImage.create(raw8Image.nHeight, raw8Image.nWidth, CV_8UC3);
        (void)DxRaw8toRGB24Ex(
                raw8Image.pImgBuf,
                rgbImage.data,
                raw8Image.nWidth,
                raw8Image.nHeight,
                RAW2RGB_NEIGHBOUR,
                BAYERBG,
                false,
                DX_ORDER_RGB
        ); //TODO 检查这里的错误
    }

    [[nodiscard]] std::string GetLibError() noexcept
    {
        GX_STATUS error_pre{};
        std::size_t error_size{};
        if (GXGetLastError(&error_pre, nullptr, &error_size) != GX_STATUS_SUCCESS)
        {
            roslog::error("GetLibError: cannot get length of last error");
            return "";
        }

//        std::string buffer('\0', error_size);
        std::string buffer(error_size, '\0');
        GX_STATUS error_after{};
        if (GXGetLastError(&error_after, buffer.data(), &error_size) != GX_STATUS_SUCCESS)
        {
            roslog::error("GetLibError: cannot get content of last error");
            return "";
        }
        if (error_after != error_pre)
            roslog::warn("GetLibError: error code not match ({} != {})", error_pre, error_after);
        return buffer;
    }

    template <typename TValue>
    using GxGetFuncPtr = GX_STATUS(*)(GX_DEV_HANDLE, GX_FEATURE_ID_CMD, TValue*);

    template <typename TValue>
    using GxSetFuncPtr = GX_STATUS(*)(GX_DEV_HANDLE, GX_FEATURE_ID_CMD, TValue);

    template <
            typename TValue,
            GxGetFuncPtr<TValue> TGet,
            GxSetFuncPtr<TValue> TSet,
            GX_FEATURE_ID_CMD TFeature>
    struct DeviceParameter
    {
        TValue Value{};

        bool Get(const GX_DEV_HANDLE device) noexcept
        {
            if (TGet(device, TFeature, &Value) == GX_STATUS_SUCCESS) return true;
            roslog::error("DeviceParameter::Get: cannot get feature({}): {}", TFeature, GetLibError());
            return false;
        }

        bool Set(const GX_DEV_HANDLE device) const noexcept
        {
            if (TSet(device, TFeature, Value) == GX_STATUS_SUCCESS) return true;
            roslog::error("DeviceParameter::Set: cannot set feature({}): {}", TFeature, GetLibError());
            return false;
        }

        static bool GetIsImplemented(const GX_DEV_HANDLE device, bool& isImplemented) noexcept
        {
            if (GXIsImplemented(device, TFeature, &isImplemented) == GX_STATUS_SUCCESS) return true;
            roslog::error("DeviceParameter::GetIsImplemented: cannot check feature({}) is implemented: {}", TFeature, GetLibError());
            return false;
        }
    };

    template <GX_FEATURE_ID_CMD TFeature>
    using DeviceIntParameter = DeviceParameter<std::int64_t, GXGetInt, GXSetInt, TFeature>;

    template <GX_FEATURE_ID_CMD TFeature>
    using DeviceFloatParameter = DeviceParameter<double, GXGetFloat, GXSetFloat, TFeature>;

    template <GX_FEATURE_ID_CMD TFeature>
    using DeviceBoolParameter = DeviceParameter<bool, GXGetBool, GXSetBool, TFeature>;

    template <GX_FEATURE_ID_CMD TFeature, typename TEnum>
    struct DeviceEnumParameter
    {
        TEnum Value;

        bool Get(const GX_DEV_HANDLE device) noexcept
        {
            std::int64_t value{};
            if (GXGetEnum(device, TFeature, &value) == GX_STATUS_SUCCESS)
            {
                Value = static_cast<TEnum>(value);
                return true;
            }
            roslog::error("无法获取相机参数({}): {}", TFeature, GetLibError());
            return false;
        }

        bool Set(const GX_DEV_HANDLE device) noexcept
        {
            if (const auto value = static_cast<std::int64_t>(Value);
                    GXSetEnum(device, TFeature, value) == GX_STATUS_SUCCESS)
                return true;
            roslog::error("无法设置相机参数({}): {}", TFeature, GetLibError());
            return false;
        }

        static bool GetIsImplemented(const GX_DEV_HANDLE device, bool& isImplemented) noexcept
        {
            if (GXIsImplemented(device, TFeature, &isImplemented) == GX_STATUS_SUCCESS) return true;
            roslog::error("无法获取相机参数({})是否实现：{}", TFeature, GetLibError());
            return false;
        }
    };

    namespace DeviceParameters
    {
        using PayLoadSize = DeviceIntParameter<GX_INT_PAYLOAD_SIZE>;

        using Gain = DeviceFloatParameter<GX_FLOAT_GAIN>;
        using AutoGainMax = DeviceFloatParameter<GX_FLOAT_AUTO_GAIN_MAX>;
        using AutoGainMin = DeviceFloatParameter<GX_FLOAT_AUTO_GAIN_MIN>;
        using AutoGain = DeviceEnumParameter<GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_ENTRY>;

        using ExposureTime = DeviceFloatParameter<GX_FLOAT_EXPOSURE_TIME>;
        using AutoExposureTimeMax = DeviceFloatParameter<GX_FLOAT_AUTO_EXPOSURE_TIME_MAX>;
        using AutoExposureTimeMin = DeviceFloatParameter<GX_FLOAT_AUTO_EXPOSURE_TIME_MIN>;
        using AutoExposure = DeviceEnumParameter<GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_ENTRY>;

        using BalanceRatioSelector = DeviceEnumParameter<GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_ENTRY>;
        using BalanceRatio = DeviceFloatParameter<GX_FLOAT_BALANCE_RATIO>;
        using AutoBalance = DeviceEnumParameter<GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_ENTRY>;
    }

    struct GxCameraConfigureParameter
    {
        DeviceParameters::Gain Gain{ 12 };
        DeviceParameters::AutoGainMax AutoGainMax{ 12 };
        DeviceParameters::AutoGainMin AutoGainMin{ 2 };
        DeviceParameters::AutoGain AutoGain{ GX_GAIN_AUTO_OFF };

        DeviceParameters::ExposureTime ExposureTime{ 4000 };
        DeviceParameters::AutoExposureTimeMax AutoExposureTimeMax{ 10000 };
        DeviceParameters::AutoExposureTimeMin AutoExposureTimeMin{ 1000 };
        DeviceParameters::AutoExposure AutoExposure{ GX_EXPOSURE_AUTO_OFF };

        DeviceParameters::BalanceRatio RedBalanceRatio{ 1.00f };
        DeviceParameters::BalanceRatio GreenBalanceRatio{ 1.00f };
        DeviceParameters::BalanceRatio BlueBalanceRatio{ 1.00f };

        bool Apply(const GX_DEV_HANDLE device)
        {
            if (!AutoGain.Set(device)) return false;
            if (AutoGain.Value != GX_GAIN_AUTO_OFF)
            {
                if (!AutoGainMax.Set(device)) return false;
                if (!AutoGainMin.Set(device)) return false;
            }
            else if (!Gain.Set(device)) return false;

            if (!AutoExposure.Set(device)) return false;
            if (AutoExposure.Value != GX_EXPOSURE_AUTO_OFF)
            {
                if (!AutoExposureTimeMax.Set(device)) return false;
                if (!AutoExposureTimeMin.Set(device)) return false;
            }
            else { if (!ExposureTime.Set(device)) return false; }

            DeviceParameters::AutoBalance auto_balance{};
            auto_balance.Value = GX_BALANCE_WHITE_AUTO_OFF;
            if (!auto_balance.Set(device)) return false;

            DeviceParameters::BalanceRatioSelector selector{};
            selector.Value = GX_BALANCE_RATIO_SELECTOR_RED;
            if (!selector.Set(device)) return false;
            if (!RedBalanceRatio.Set(device)) return false;

            selector.Value = GX_BALANCE_RATIO_SELECTOR_GREEN;
            if (!selector.Set(device)) return false;
            if (!GreenBalanceRatio.Set(device)) return false;

            selector.Value = GX_BALANCE_RATIO_SELECTOR_BLUE;
            if (!selector.Set(device)) return false;
            if (!BlueBalanceRatio.Set(device)) return false;
            return true;
        }
    };

    enum class CaptureMode {
        CameraDevice,  // 工业相机模式
        VideoFile      // 视频文件模式
    };

    class Video{
        cv::VideoCapture VideoCap;
        std::string VideoPath;
        double FPS;
        int delay;

        static double calculate_real_fps(const cv::VideoCapture& cap) {
            /// TODO 通过时间戳处理 下面是乱写的
            double fps = 0.0;
            double frame_count = cap.get(cv::CAP_PROP_FRAME_COUNT);
            double duration = cap.get(cv::CAP_PROP_POS_MSEC);
            if (frame_count > 0) {
                fps = frame_count / (duration / 1000.0);
            }
            return fps;
        }

        public:
        Video(const std::string& path) : VideoPath(path)
        {
            VideoCap.open(VideoPath);
            if (!VideoCap.isOpened()) {
                roslog::error("Video::Video: cannot open video file: {}", VideoPath);
                throw std::runtime_error("Video::Video: cannot open video file");
            }
            FPS = VideoCap.get(cv::CAP_PROP_FPS);
            if(FPS <= 0) {
                roslog::warn("Video::Video: FPS is not set, calculating real FPS");
                FPS = calculate_real_fps(VideoCap);
            }
            delay = static_cast<int>(1000.0 / FPS);
        }

        ~Video() {
            if (VideoCap.isOpened()) {
                VideoCap.release();
            }
        }

        void set_speed(double speed_factor = 1.0) {
            if (speed_factor <= 0) {
                roslog::error("Video::set_speed: speed factor must be positive");
                return;
            }
            int original_delay = static_cast<int>(1000.0 / FPS);
            int new_delay = std::max(1, static_cast<int>(original_delay / speed_factor));
            delay = new_delay;
        }

        bool read(cv::Mat& image) {
            if (!VideoCap.read(image)) {
                roslog::error("Video::read: cannot read video frame");
                return false;
            }
            return true;
        }

    };

    class Camera
    {
        std::unique_ptr<GX_DEV_HANDLE> DeviceHandle{};
        GxCameraConfigureParameter Parameters{};
        std::vector<std::uint8_t> ImageBuffer{};
        CaptureMode Mode = CaptureMode::CameraDevice; // 默认模式
        cv::VideoCapture VideoCap; // 新增VideoCapture对象

        PGX_FRAME_BUFFER DeviceFrameBuffers[5]; 

        struct GxLib
        {
            GX_STATUS Status;
            GxLib() noexcept
            {
                Status = GXInitLib();
                switch (Status)
                {
                    case GX_STATUS_NOT_FOUND_TL:
                        roslog::error("Camera::GxLib::GxLib: cannot find gxlib");
                        break;

                    case GX_STATUS_SUCCESS:
                        roslog::info("Camera::GxLib::GxLib: initialized successfully");
                        break;

                    default:
                        roslog::warn("Camera::GxLib::GxLib: unexpected status({})", Status);
                        break;
                }
            }

            [[nodiscard]] bool IsInitialized() const noexcept
            {
                return Status == GX_STATUS_SUCCESS;
            }

            ~GxLib() noexcept
            {
                GXCloseLib(); //真的会有人去检查这个状态吗？反正关了就完事了
            }
        };

        inline static std::unique_ptr<GxLib> LibPtr{};

    public:
        Camera() noexcept = default;
        ~Camera() noexcept
        {
            if(Mode == CaptureMode::VideoFile && VideoCap.isOpened())
            {
                VideoCap.release();
                return;
            }

            if (DeviceHandle != nullptr)
            {
                auto& handle = *DeviceHandle;
                GXStreamOff(handle);
                GXCloseDevice(handle);
                DeviceHandle.reset();
            }
            if (LibPtr != nullptr) LibPtr.reset();
        }

        bool Initialize(const std::string& source = "", const std::string& sn = "", int device_index = 1) noexcept try
        {
            if(!source.empty()) {
                Mode = CaptureMode::VideoFile;
                VideoCap.open(source);
                if (!VideoCap.isOpened()) {
                    roslog::error("Camera::Initialize: cannot open video file: {}", source);
                    return false;
                }
                return true;
            }
            else {
                Mode = CaptureMode::CameraDevice;
            }
            if (LibPtr == nullptr) LibPtr = std::make_unique<GxLib>();
            if (!LibPtr->IsInitialized()) return false;
            auto handle_ptr = std::make_unique<GX_DEV_HANDLE>();
            auto& handle = *handle_ptr;

            std::uint32_t device_count;
            static constexpr auto timeout_ms = 1000;
            if (GXUpdateDeviceList(&device_count, timeout_ms) != GX_STATUS_SUCCESS)
            {
                roslog::error("Camera::AcqureDevice: cannot update device list: {}", GetLibError());
                return false;
            }

            if (device_count == 0)
            {
                roslog::error("Camera::AcquireDevice: cannot find any device");
                return false;
            }

		    if (sn.empty()) {

            // open by index
            if (GXOpenDeviceByIndex(device_index, &handle) != GX_STATUS_SUCCESS)
            {
                roslog::error("Camera::AcquireDevice: cannot open device({}): {}", device_index, GetLibError());
                return false;
            }
	    }
	    else {
		   std::string sn_copy = sn;
		   GX_OPEN_PARAM op{(char*)sn_copy.c_str(), GX_OPEN_SN, GX_ACCESS_EXCLUSIVE};
		   if (GXOpenDevice(&op, &handle) != GX_STATUS_SUCCESS) {
	    	       roslog::error("Camera::AcquireDevice: cannot open device({}): {}", sn_copy, GetLibError());
                	return false;
		   }	
	    }
	    

            if (!Parameters.Apply(handle))
            {
                roslog::error("Camera::AcquireDevice: cannot apply parameters: {}", GetLibError());
                return false;
            }

            if (GXStreamOn(handle) != GX_STATUS_SUCCESS)
            {
                roslog::error("Camera::AcquireDevice: cannot start capture: {}", GetLibError());
                return false;
            }

            DeviceHandle = std::move(handle_ptr);
            return true;
        }
        catch (const std::exception& ex)
        {
            roslog::error("Camera::Initialize: {}", ex.what());
            return false;
        }

        bool IsInitialized() const noexcept
        {
            if (Mode == CaptureMode::VideoFile) {
                return VideoCap.isOpened();
            }
            return DeviceHandle != nullptr;
        }

        // this function do not check if this instance is initialized
        bool GetImage(cv::Mat& image) noexcept
        {
            if(Mode == CaptureMode::VideoFile)
            {
                return VideoCap.read(image);
            }

            auto& deviceHandle = *DeviceHandle;

            std::uint32_t frame_count{};
            const auto borrow_result = GXDQAllBufs(deviceHandle, DeviceFrameBuffers, 5, &frame_count, 1000);
            
            if (borrow_result != GX_STATUS_SUCCESS)
            {
                roslog::error("Camera::GetImage: cannot borrow buffer: {}", GetLibError());
                return false;
            }

            if (frame_count == 0){
                roslog::warn("Camera::GetImage: no frame captured, this is impossible if the camera is ok");
                const auto return_result = GXQAllBufs(deviceHandle);
                if (return_result != GX_STATUS_SUCCESS)
                    roslog::error("Camera::GetImage: cannot return buffer: {}", GetLibError());
                return false;
            }

            for (int i = static_cast<int>(frame_count) - 1; i >= 0; --i) {
                const auto& buffer = DeviceFrameBuffers[i];
                if (buffer == nullptr || buffer->nStatus != GX_FRAME_STATUS_SUCCESS){
                    roslog::error("Camera::GetImage: the image at {} is failed to capture", i);
                    continue;
                }

                auto& frame = *buffer;
    
                image.create(frame.nHeight, frame.nWidth, CV_8UC3);
                ConvertRaw8ToRGB24(frame, image);
                const auto return_result = GXQAllBufs(deviceHandle);
                if (return_result != GX_STATUS_SUCCESS)
                    roslog::error("Camera::GetImage: cannot return buffer: {}", GetLibError());

                return true;
            }

            roslog::warn("Camera::GetImage: there is no valid image in camera buffer");

            const auto return_result = GXQAllBufs(deviceHandle);
            if (return_result != GX_STATUS_SUCCESS)
                roslog::error("Camera::GetImage: cannot return buffer: {}", GetLibError());

            return false;
        }

        GxCameraConfigureParameter& Configure() noexcept
        {
            return Parameters;
        }
    };

}
