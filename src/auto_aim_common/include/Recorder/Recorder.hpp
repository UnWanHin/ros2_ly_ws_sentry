// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>

namespace recording {

class VideoRecorder {
public:
    // 单例访问接口
    static VideoRecorder& instance() {
        static VideoRecorder instance;
        return instance;
    }

    // 启动录制系统
    void start(const std::string& output_dir = "../record") {
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_running_) {
            log("Recorder already running");
            return;
        }

        // 创建带时间戳的文件名
        auto timestamp = std::chrono::system_clock::now();
        std::time_t time = std::chrono::system_clock::to_time_t(timestamp);
        char buffer[32];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&time));
        
        // 确保输出目录存在
        output_path_ = std::filesystem::path(output_dir) / "videos";
        std::filesystem::create_directories(output_path_);
        
        // 构建完整文件路径
        video_path_ = (output_path_ / buffer).string() + ".avi";
        
        // 初始化录制状态
        is_running_ = true;
        frame_queue_ = {};
        worker_thread_ = std::thread(&VideoRecorder::processing_loop, this);
        
        log("Recording started: " + video_path_);
    }

    // 添加视频帧
    void add_frame(const cv::Mat& frame) {
        if (!is_running_ || frame.empty()) return;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (frame_queue_.size() >= MAX_QUEUE_SIZE) {
                log("Frame dropped, queue full: " + std::to_string(frame_queue_.size()));
                return;
            }
            frame_queue_.push(frame.clone()); // 深拷贝避免数据竞争
        }
        
        condition_.notify_one();
    }

    // 停止录制
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!is_running_) return;
            is_running_ = false;
        }

        condition_.notify_all();
        
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }

        if (video_writer_.isOpened()) {
            video_writer_.release();
            log("Video saved: " + video_path_);
        }
    }

private:
    // 核心处理循环
    void processing_loop() {
        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            condition_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return !is_running_ || !frame_queue_.empty();
            });

            if (!is_running_ && frame_queue_.empty()) break;

            process_frames();
        }
        process_frames(); // 处理剩余帧
    }

    // 帧处理逻辑
    void process_frames() {
        std::queue<cv::Mat> temp_queue;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            std::swap(temp_queue, frame_queue_);
        }

        while (!temp_queue.empty()) {
            auto frame = temp_queue.front();
            temp_queue.pop();

            if (!video_writer_.isOpened()) {
                initialize_writer(frame);
            }

            if (video_writer_.isOpened()) {
                video_writer_.write(frame);
            }
        }
    }

    // 初始化视频写入器
    void initialize_writer(const cv::Mat& frame) {
        const int codec = cv::VideoWriter::fourcc('X','V','I','D'); // 推荐编码器
        const double fps = 30.0;
        
        if (frame.cols < 640 || frame.rows < 480) {
            log("Invalid frame size: " 
                + std::to_string(frame.cols) + "x" 
                + std::to_string(frame.rows));
            return;
        }

        video_writer_.open(video_path_, codec, fps, frame.size());
        if (!video_writer_.isOpened()) {
            log("Failed to initialize video writer for: " + video_path_);
        }
    }

    // 日志输出
    void log(const std::string& message) {
        std::cout << "[Recorder] " << message << std::endl;
    }

    // 成员变量
    std::string video_path_;
    std::filesystem::path output_path_;
    
    cv::VideoWriter video_writer_;
    std::queue<cv::Mat> frame_queue_;
    
    std::mutex mutex_;
    std::mutex queue_mutex_;
    std::condition_variable condition_;
    
    std::thread worker_thread_;
    std::atomic<bool> is_running_{false};
    
    static constexpr size_t MAX_QUEUE_SIZE = 100; // 队列容量
};

} // namespace recording