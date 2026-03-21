// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

// Create by Qmian 2025/3/17
#pragma once
#include <opencv2/opencv.hpp>
#include "httplib.h"
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <iostream>
#include <atomic>

class VideoStreamer {
public:
    VideoStreamer() = delete; // 禁止实例化

    // 初始化服务器（需要在程序启动时调用一次）
    static void init() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (initialized_) {
            return;
        }
        server_ = std::make_unique<httplib::Server>();
        server_running_.store(true);
        server_thread_ = std::thread(startServer);
        initialized_ = true;
    }

    // 清理资源（程序退出前调用）
    static void cleanup() {
        {
            std::lock_guard<std::mutex> lock(lifecycle_mutex_);
            if (!initialized_) {
                return;
            }
            server_running_.store(false);
            if (server_) {
                server_->stop();
            }
        }

        if (server_thread_.joinable()) {
            server_thread_.join();
        }

        {
            std::lock_guard<std::mutex> lock(lifecycle_mutex_);
            current_frame_.clear();
            server_.reset();
            initialized_ = false;
        }
    }

    // 设置当前帧数据
    static void setFrame(const cv::Mat& frame) {
        std::vector<uchar> buf;
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(jpeg_quality_);
        cv::imencode(".jpg", frame, buf, compression_params);
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_.swap(buf);
        }
    }

private:
    // 服务器启动函数，供线程调用
    static void startServer() {
        auto* server = server_.get();
        if (!server) {
            std::cerr << "[VideoStreamer] server is null, skip start." << std::endl;
            return;
        }

        // 设置视频流接口
        server->Get("/stream", [](const httplib::Request& req, httplib::Response& res) {
            res.set_chunked_content_provider(
                "multipart/x-mixed-replace; boundary=frame",
                [](size_t offset, httplib::DataSink& sink) -> bool {
                    if (!server_running_.load()) {
                        return false;
                    }

                    std::vector<uchar> frame_data;
                    {
                        std::lock_guard<std::mutex> lock(frame_mutex_);
                        frame_data = current_frame_;
                    }
                    if (!frame_data.empty()) {
                        std::string boundary = "\r\n--frame\r\n";
                        std::string header = "Content-Type: image/jpeg\r\n\r\n";
                        sink.write(boundary.data(), boundary.size());
                        sink.write(header.data(), header.size());
                        sink.write(reinterpret_cast<const char*>(frame_data.data()), frame_data.size());
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    return server_running_.load();
                }
            );
            });
        std::cout << "Server started at http://" << local_ip_.c_str() << ":" << port_ << "/stream" << std::endl;
        if (!server->listen(local_ip_.c_str(), port_)) {
            std::cerr << "[VideoStreamer] listen failed at " << local_ip_ << ":" << port_ << std::endl;
        }
    }

private:
    inline static int port_ = 8081;
    inline static std::string local_ip_ = "0.0.0.0";
    inline static int jpeg_quality_ = 50;
    inline static std::unique_ptr<httplib::Server> server_;
    inline static std::thread server_thread_;
    inline static std::mutex lifecycle_mutex_;
    inline static std::mutex frame_mutex_;
    inline static std::vector<uchar> current_frame_;
    inline static std::atomic_bool server_running_{false};
    inline static bool initialized_ = false;
};

// Create by Qmian 2025/3/17
// #pragma once
// #include <opencv2/opencv.hpp>
// #include "httplib.h"
// #include <mutex>
// #include <thread>
// #include <vector>
// #include <memory>
// #include <iostream>
// #include <atomic>
// #include <queue>
// #include <condition_variable>

// class VideoStreamer {
// public:
//     VideoStreamer() = delete; // 禁止实例化

//     // 初始化服务器（需要在程序启动时调用一次）
//     static void init() {
//         if (!initialized_) {
//             server_ = std::make_unique<httplib::Server>();
//             server_running_ = true;
//             encoder_running_ = true;
//             server_thread_ = std::thread(startServer);
//             encoder_thread_ = std::thread(encoderLoop);
//             initialized_ = true;
//         }
//     }

//     // 清理资源（程序退出前调用）
//     static void cleanup() {
//         server_running_ = false;
//         encoder_running_ = false;
//         frame_cond_.notify_one();
        
//         if (encoder_thread_.joinable()) {
//             encoder_thread_.join();
//         }
        
//         if (server_thread_.joinable()) {
//             server_thread_.join();
//         }
        
//         server_.reset();
//         initialized_ = false;
//     }

//     // 设置当前帧数据 - 优化版
//     static void setFrame(const cv::Mat& frame) {
//         if (!initialized_ || !encoder_running_) return;
        
//         // 检查是否有活跃客户端，没有则不处理
//         if (active_clients_ == 0) return;
        
//         // 帧率控制 - 每隔几帧才处理一次
//         static int frame_counter = 0;
//         if (++frame_counter % frame_skip_ != 0) return;
        
//         {
//             std::lock_guard<std::mutex> lock(frame_queue_mutex_);
//             // 限制队列大小，防止内存占用过多
//             if (frame_queue_.size() >= max_queue_size_) {
//                 frame_queue_.pop();
//             }
//             frame_queue_.push(frame.clone());
//         }
//         frame_cond_.notify_one();
//     }

// private:
//     // 编码线程主循环
//     static void encoderLoop() {
//         while (encoder_running_) {
//             cv::Mat frame;
            
//             // 等待新帧或退出信号
//             {
//                 std::unique_lock<std::mutex> lock(frame_queue_mutex_);
//                 frame_cond_.wait(lock, []{ 
//                     return !encoder_running_ || !frame_queue_.empty(); 
//                 });
                
//                 if (!encoder_running_ && frame_queue_.empty()) break;
                
//                 if (!frame_queue_.empty()) {
//                     frame = frame_queue_.front();
//                     frame_queue_.pop();
//                 }
//             }
            
//             if (!frame.empty()) {
//                 // 在后台线程中执行JPEG编码
//                 std::vector<uchar> buf;
//                 std::vector<int> compression_params;
//                 compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
//                 compression_params.push_back(jpeg_quality_);
//                 cv::imencode(".jpg", frame, buf, compression_params);
                
//                 {
//                     std::lock_guard<std::mutex> lock(frame_mutex_);
//                     current_frame_.swap(buf);
//                 }
//             }
//         }
//     }

//     // 服务器启动函数，供线程调用
//     static void startServer() {
//         // 设置视频流接口
//         server_->Get("/stream", [](const httplib::Request& req, httplib::Response& res) {
//             active_clients_++;  // 增加活跃客户端计数
            
//             res.set_chunked_content_provider(
//                 "multipart/x-mixed-replace; boundary=frame",
//                 [](size_t offset, httplib::DataSink& sink) -> bool {
//                     if (!server_running_) return false;
                    
//                     std::vector<uchar> frame_data;
//                     {
//                         std::lock_guard<std::mutex> lock(frame_mutex_);
//                         frame_data = current_frame_;
//                     }
                    
//                     if (!frame_data.empty()) {
//                         std::string boundary = "\r\n--frame\r\n";
//                         std::string header = "Content-Type: image/jpeg\r\n\r\n";
//                         sink.write(boundary.data(), boundary.size());
//                         sink.write(header.data(), header.size());
//                         sink.write(reinterpret_cast<const char*>(frame_data.data()), frame_data.size());
//                     }
//                     return true;
//                 },
//                 [](bool success) {
//                     active_clients_--;  // 客户端断开连接时减少计数
//                 }
//             );
//         });
        
//         std::cout << "VideoStreamer started at http://" << local_ip_.c_str() << ":" << port_ << "/stream" << std::endl;
//         server_->listen(local_ip_.c_str(), port_);
//     }

// private:
//     inline static int port_ = 8081;
//     inline static std::string local_ip_ = "127.0.0.1";
//     inline static int jpeg_quality_ = 50;
//     inline static int frame_skip_ = 3;  // 每3帧处理1帧
//     inline static size_t max_queue_size_ = 2;  // 最大队列长度
    
//     inline static std::unique_ptr<httplib::Server> server_;
//     inline static std::thread server_thread_;
//     inline static std::thread encoder_thread_;
//     inline static std::mutex frame_mutex_;
//     inline static std::mutex frame_queue_mutex_;
//     inline static std::condition_variable frame_cond_;
//     inline static std::queue<cv::Mat> frame_queue_;
    
//     inline static std::vector<uchar> current_frame_;
//     inline static std::atomic<bool> server_running_ = false;
//     inline static std::atomic<bool> encoder_running_ = false;
//     inline static std::atomic<int> active_clients_ = 0;
//     inline static bool initialized_ = false;
// };
