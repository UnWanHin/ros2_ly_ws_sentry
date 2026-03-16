/*
 * shooting_table_calib_node（射表标定）
 *
 * 功能：
 * - 独立运行检测/跟踪/预测链路
 * - 支持手动微调 pitch/yaw 并记录射击数据
 * - 输出 CSV 供离线拟合和系数回填
 */
#include <rclcpp/rclcpp.hpp>
#include <fmt/format.h>

// [ROS 2] 算法庫頭文件
#include <car_tracker/tracker.hpp>
#include <car_tracker/tracker_matcher.hpp>
#include <solver/solver.hpp>
#include <predictor/predictor.hpp>
#include <controller/controller.hpp>

#include <armor_detector/armor_detector.hpp>
#include <armor_detector/armor_filter.hpp>
#include <armor_detector/armor_refinder.hpp>
#include <armor_detector/pnp_solver.hpp>
#include <car_detector/car_finder.hpp>
#include <car_and_armor_detector.hpp>
#include <camera/Camera.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

#include <RosTools/RosTools.hpp>
#include <Logger/Logger.hpp>
#include <TimeStamp/TimeStamp.hpp>

// [ROS 2] 消息頭文件 (.hpp)
#include <auto_aim_common/msg/armors.hpp>
#include <auto_aim_common/msg/target.hpp>
#include <auto_aim_common/DetectionType.hpp>
#include <auto_aim_common/SolverType.hpp>
#include <auto_aim_common/TrackerType.hpp>
#include <gimbal_driver/msg/gimbal_angles.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <auto_aim_common/Location.hpp>

#include <VideoStreamer/VideoStreamer.hpp>

// [ROS 2] 系統頭文件
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <initializer_list>
#include <limits>

using namespace ly_auto_aim;
using namespace LangYa;

// [ROS 2] 全局節點指針 (用於 roslog 適配)
namespace {
    rclcpp::Node::SharedPtr global_node_ptr = nullptr;
}

// [ROS 2] 本地 roslog 適配器
namespace roslog_adapter {
    static rclcpp::Logger get_logger() {
        if (global_node_ptr) {
            return global_node_ptr->get_logger();
        }
        return rclcpp::get_logger("shooting_table_calib");
    }
    template <typename... Args>
    void warn(const char* format_str, const Args&... args) {
        RCLCPP_WARN(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void info(const char* format_str, const Args&... args) {
        RCLCPP_INFO(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
    template <typename... Args>
    void error(const char* format_str, const Args&... args) {
        RCLCPP_ERROR(get_logger(), "%s", fmt::vformat(format_str, fmt::make_format_args(args...)).c_str());
    }
}
#define roslog roslog_adapter

namespace {
    const char AppName[] = "shooting_table_calib_node";
    
    // [ROS 2] 射表記錄結構 (時間類型改為 rclcpp::Time)
    struct ShootingRecord {
        double z_height;
        double horizontal_distance;
        double relative_yaw;
        double relative_pitch;
        cv::Point3d target_world_coord;
        double absolute_yaw;
        double absolute_pitch;
        rclcpp::Time timestamp;
        double target_yaw;
        double fitted_pitch;
        double fitted_yaw;
    };

    // 键盘输入处理类 (保持不变)
    class KeyboardInput {
    private:
        struct termios original_termios{};
        int original_flags{-1};
        int input_fd{-1};
        bool own_fd{false};
        bool initialized = false;

        bool initFromFd(int fd, bool take_ownership, const char* fd_name) {
            if (fd < 0 || !isatty(fd)) {
                return false;
            }

            if (tcgetattr(fd, &original_termios) != 0) {
                return false;
            }

            int flags = fcntl(fd, F_GETFL, 0);
            if (flags < 0) {
                return false;
            }

            struct termios new_termios = original_termios;
            new_termios.c_lflag &= ~(ICANON | ECHO);
            if (tcsetattr(fd, TCSANOW, &new_termios) != 0) {
                return false;
            }

            if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
                tcsetattr(fd, TCSANOW, &original_termios);
                return false;
            }

            input_fd = fd;
            own_fd = take_ownership;
            original_flags = flags;
            initialized = true;
            std::cout << "✓ Terminal input mode initialized (" << fd_name << ")\n";
            return true;
        }
        
    public:
        KeyboardInput() {
            if (initFromFd(STDIN_FILENO, false, "stdin")) {
                return;
            }

            int tty_fd = open("/dev/tty", O_RDONLY);
            if (tty_fd >= 0 && initFromFd(tty_fd, true, "/dev/tty")) {
                return;
            }

            if (tty_fd >= 0) {
                close(tty_fd);
            }
            std::cerr << "✗ Terminal input disabled: no interactive TTY available\n";
        }
        
        ~KeyboardInput() {
            restore();
        }
        
        void restore() {
            if (initialized) {
                if (input_fd >= 0) {
                    tcsetattr(input_fd, TCSANOW, &original_termios);
                    if (original_flags >= 0) {
                        fcntl(input_fd, F_SETFL, original_flags);
                    }
                }
                std::cout << "\n✓ Terminal input mode restored\n";
                initialized = false;
            }

            if (own_fd && input_fd >= 0) {
                close(input_fd);
            }

            input_fd = -1;
            own_fd = false;
            original_flags = -1;
        }
        
        char getKey() {
            if (!initialized || input_fd < 0) return 0;
            char key;
            if (read(input_fd, &key, 1) == 1) {
                return key;
            }
            return 0;
        }
        
        std::string getLine() {
            if (!initialized || input_fd != STDIN_FILENO) return "";
            
            // 临时恢复正常输入模式
            tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
            fcntl(STDIN_FILENO, F_SETFL, original_flags);
            
            std::string line;
            std::getline(std::cin, line);
            
            // 重新设置非阻塞模式
            struct termios new_termios = original_termios;
            new_termios.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
            fcntl(STDIN_FILENO, F_SETFL, original_flags | O_NONBLOCK);
            
            return line;
        }
    };

    // [ROS 2] 繼承 rclcpp::Node
    class ShootingTableCalibNode : public rclcpp::Node
    {
    private:
        KeyboardInput keyboard;
        
        // 算法模块
        std::shared_ptr<tracker::Tracker> tracker;
        std::shared_ptr<solver::Solver> solver;
        std::shared_ptr<predictor::Predictor> predictor;
        std::shared_ptr<controller::Controller> controller;
        
        // detector模块
        CarAndArmorDetector carAndArmorDetector;
        ArmorFilter filter;
        ArmorRefinder finder;
        CarFinder carFinder;
        pnp_solver::CameraIntrinsicsParameterPack cameraIntrinsics;
        PoseSolver pnpSolver;
        Camera camera;
        
        // Video功能支持
        bool use_video = false;
        bool use_ros_bag = false;
        std::string video_path;
        cv::VideoCapture video_cap;
        
        // 状态变量
        std::vector<ShootingRecord> records;
        std::string csv_filename;
        std::string current_init_stage{"not_started"};
        bool csv_file_created{false};
        
        // 控制参数
        double pitch_adjustment = 0.0;
        double yaw_adjustment = 0.0;
        const double adjustment_step = 0.1;
        std::atomic<bool> is_shooting{false};
        rclcpp::Time shoot_start_time;
        const double shoot_duration = 0.3;
        bool fire_require_dual_axis_lock = true;
        double fire_max_yaw_error_deg = 8.0;
        double fire_max_pitch_error_deg = 5.0;
        bool auto_lock_fire = false;
        bool auto_fire = true;
        ArmorType selected_target_type = ArmorType::Infantry2;
        
        // 瞄准状态
        std::atomic<bool> is_aiming{false};
        std::atomic<bool> should_aim_once{false};
        cv::Point3d current_target_world;
        gimbal_driver::msg::GimbalAngles current_gimbal_angles;
        double current_target_yaw = 0.0;
        
        // 控制命令状态
        std::atomic<bool> control_valid{false};
        std::atomic<bool> aim_only_mode{false};  
        double target_yaw = 0.0;
        double target_pitch = 0.0;
        
        // 射击表系数
        struct ShootTableParams {
            bool enable;
            struct {
                double intercept, coef_z, coef_d, coef_z2, coef_zd, coef_d2;
            } pitch, yaw;
        } shoot_table_params;

        // 火控系统状态
        struct FireControlData {
            uint8_t fire_status = 0;  // 只使用最低2位：00 或 11
            bool last_fire_command = false; 
        } fire_control;
        
        // 显示相关
        bool web_show = true;
        bool draw_image = true;
        bool myTeamBlue{false};

        rclcpp::Time last_param_check;
        const double param_check_interval = 3.0;

        // [ROS 2] Publishers and Subscribers
        rclcpp::Subscription<gimbal_driver::msg::GimbalAngles>::SharedPtr gimbal_sub_;
        rclcpp::Publisher<gimbal_driver::msg::GimbalAngles>::SharedPtr control_pub_;
        rclcpp::Publisher<auto_aim_common::msg::Target>::SharedPtr target_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr firecode_pub_;

    public:
        ShootingTableCalibNode()
            : Node(
                  AppName,
                  rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
                      .automatically_declare_parameters_from_overrides(true)),
              pnpSolver(cameraIntrinsics)
        {
        }

        void markInitStage(const std::string& stage)
        {
            current_init_stage = stage;
            roslog::info("Init stage: {}", current_init_stage);
            std::cout << "[INIT] " << current_init_stage << "\n";
        }

        void Initialize()
        {
            // 注意：solver/predictor/controller 在 create*() 時會讀取全局 node 指針；
            // 必須先由 main 設置全局指針，再執行初始化流程。
            markInitStage("load shoot table params");
            loadShootTableParams();
            markInitStage("initialize video");
            initializeVideo();
            markInitStage("initialize camera");
            initializeCamera();
            markInitStage("initialize detector");
            initializeDetector();
            markInitStage("initialize algorithms");
            initializeAlgorithms();
            markInitStage("setup ros topics");
            setupRosTopics();
            markInitStage("create calibration csv");
            createCSVFile();
            markInitStage("print instructions");
            printInstructions();

            if (web_show) {
                markInitStage("start web streamer");
                VideoStreamer::init();
            }

            markInitStage("ready");
            last_param_check = this->now();
        }
        
        ~ShootingTableCalibNode()
        {
            if (web_show) {
                VideoStreamer::cleanup();
            }
            if (video_cap.isOpened()) {
                video_cap.release();
            }
        }

    private:
        // [ROS 2] 參數獲取輔助函數
        template<typename T>
        void getParamSafe(const std::string& name, T& variable, const T& default_value) {
            if (!this->has_parameter(name)) {
                this->declare_parameter(name, default_value);
            }
            this->get_parameter(name, variable);
        }

        template<typename T>
        bool tryGetParamAny(const std::initializer_list<const char*>& names, T& variable) {
            for (const auto* name : names) {
                if (this->has_parameter(name) && this->get_parameter(name, variable)) {
                    return true;
                }
            }
            return false;
        }

        template<typename T>
        void getParamCompat(const std::initializer_list<const char*>& names, T& variable, const T& default_value) {
            if (tryGetParamAny(names, variable)) {
                return;
            }
            const auto* primary = *names.begin();
            if (!this->has_parameter(primary)) {
                this->declare_parameter(primary, default_value);
            }
            this->get_parameter(primary, variable);
            roslog::warn("Parameter '{}' missing, fallback default applied.", primary);
        }

        static ArmorType parseArmorTypeOrDefault(int raw_type, ArmorType default_type)
        {
            switch (raw_type) {
                case static_cast<int>(ArmorType::Hero):
                    return ArmorType::Hero;
                case static_cast<int>(ArmorType::Engineer):
                    return ArmorType::Engineer;
                case static_cast<int>(ArmorType::Infantry1):
                    return ArmorType::Infantry1;
                case static_cast<int>(ArmorType::Infantry2):
                    return ArmorType::Infantry2;
                case static_cast<int>(ArmorType::Infantry3):
                    return ArmorType::Infantry3;
                case static_cast<int>(ArmorType::Sentry):
                    return ArmorType::Sentry;
                case static_cast<int>(ArmorType::Outpost):
                    return ArmorType::Outpost;
                default:
                    return default_type;
            }
        }

        static const char* armorTypeName(ArmorType type)
        {
            switch (type) {
                case ArmorType::Hero:
                    return "Hero";
                case ArmorType::Engineer:
                    return "Engineer";
                case ArmorType::Infantry1:
                    return "Infantry1";
                case ArmorType::Infantry2:
                    return "Infantry2";
                case ArmorType::Infantry3:
                    return "Infantry3";
                case ArmorType::Sentry:
                    return "Sentry";
                case ArmorType::Outpost:
                    return "Outpost";
                default:
                    return "Unknown";
            }
        }

        void loadShootTableParams()
        {
            // 从config文件加载射击表参数
            getParamSafe("shoot_table_adjust.enable", shoot_table_params.enable, true);
            
            // pitch参数
            getParamSafe("shoot_table_adjust.pitch.intercept", shoot_table_params.pitch.intercept, 0.0);
            getParamSafe("shoot_table_adjust.pitch.coef_z", shoot_table_params.pitch.coef_z, 0.0);
            getParamSafe("shoot_table_adjust.pitch.coef_d", shoot_table_params.pitch.coef_d, 0.0);
            getParamSafe("shoot_table_adjust.pitch.coef_z2", shoot_table_params.pitch.coef_z2, 0.0);
            getParamSafe("shoot_table_adjust.pitch.coef_zd", shoot_table_params.pitch.coef_zd, 0.0);
            getParamSafe("shoot_table_adjust.pitch.coef_d2", shoot_table_params.pitch.coef_d2, 0.0);
            
            // yaw参数
            getParamSafe("shoot_table_adjust.yaw.intercept", shoot_table_params.yaw.intercept, 0.0);
            getParamSafe("shoot_table_adjust.yaw.coef_z", shoot_table_params.yaw.coef_z, 0.0);
            getParamSafe("shoot_table_adjust.yaw.coef_d", shoot_table_params.yaw.coef_d, 0.0);
            getParamSafe("shoot_table_adjust.yaw.coef_z2", shoot_table_params.yaw.coef_z2, 0.0);
            getParamSafe("shoot_table_adjust.yaw.coef_zd", shoot_table_params.yaw.coef_zd, 0.0);
            getParamSafe("shoot_table_adjust.yaw.coef_d2", shoot_table_params.yaw.coef_d2, 0.0);

            roslog::info("Loaded shoot table params - yaw coef_d2: {}", shoot_table_params.yaw.coef_d2);
            roslog::info("Loaded shoot table params - Enable: {}", shoot_table_params.enable);

            // Fire guard: only allow fire when yaw/pitch are both converged.
            getParamSafe("shooting_table_calib.fire_require_dual_axis_lock", fire_require_dual_axis_lock, true);
            getParamSafe("shooting_table_calib.fire_max_yaw_error_deg", fire_max_yaw_error_deg, 8.0);
            getParamSafe("shooting_table_calib.fire_max_pitch_error_deg", fire_max_pitch_error_deg, 5.0);
            getParamSafe("shooting_table_calib.auto_lock_fire", auto_lock_fire, false);
            getParamSafe("shooting_table_calib.auto_fire", auto_fire, true);
            int auto_target_type = static_cast<int>(ArmorType::Infantry2);
            getParamSafe("shooting_table_calib.auto_target_type", auto_target_type, auto_target_type);
            selected_target_type = parseArmorTypeOrDefault(auto_target_type, ArmorType::Infantry2);
            roslog::info(
                "Auto mode - lock_fire: {}, auto_fire: {}, target_type: {}({})",
                auto_lock_fire,
                auto_fire,
                armorTypeName(selected_target_type),
                static_cast<int>(selected_target_type));
        }
        
        void initializeVideo()
        {
            getParamCompat<bool>({"detector_config.use_video", "detector_config/use_video"}, use_video, false);
            roslog::warn("use_video: {}", use_video);
            getParamCompat<bool>({"detector_config.use_ros_bag", "detector_config/use_ros_bag"}, use_ros_bag, false);
            getParamCompat<std::string>({"detector_config.video_path", "detector_config/video_path"}, video_path, std::string(""));
            getParamCompat<bool>({"detector_config.web_show", "detector_config/web_show", "web_show"}, web_show, true);
            getParamCompat<bool>({"detector_config.draw", "detector_config/draw", "draw_image"}, draw_image, true);
            
            if (use_video && !video_path.empty()) {
                video_cap.open(video_path);
                if (!video_cap.isOpened()) {
                    roslog::error("Failed to open video file: {}", video_path);
                    use_video = false;
                } else {
                    roslog::info("Video file loaded: {}", video_path);
                }
            }
        }
        
        void initializeCamera()
        {
            if (use_video || use_ros_bag) {
                roslog::info("Skipping camera initialization - using video/rosbag");
                return;
            }
            
            std::string camera_sn;
            getParamCompat<std::string>({"camera_param.camera_sn", "camera_param/camera_sn"}, camera_sn, std::string(""));
            if (camera_sn.empty()) {
                roslog::error("camera_param.camera_sn (or camera_param/camera_sn) is empty");
                throw std::runtime_error("Camera SN parameter is empty");
            }
            
            auto &config = camera.Configure();
            config.AutoExposure.Value = GX_EXPOSURE_AUTO_OFF;
            getParamCompat<double>({"camera_param.ExposureTime", "camera_param/ExposureTime"}, config.ExposureTime.Value, 4000.0);
            config.AutoGain.Value = GX_GAIN_AUTO_OFF;
            getParamCompat<double>({"camera_param.Gain", "camera_param/Gain"}, config.Gain.Value, 12.0);
            getParamCompat<double>({"camera_param.RedBalanceRatio", "camera_param/RedBalanceRatio"}, config.RedBalanceRatio.Value, 1.2266);
            getParamCompat<double>({"camera_param.GreenBalanceRatio", "camera_param/GreenBalanceRatio"}, config.GreenBalanceRatio.Value, 1.0);
            getParamCompat<double>({"camera_param.BlueBalanceRatio", "camera_param/BlueBalanceRatio"}, config.BlueBalanceRatio.Value, 1.3711);
            
            if (!camera.Initialize("", camera_sn)) {
                roslog::error("Failed to initialize camera");
                throw std::runtime_error("Camera initialization failed");
            }
            
            roslog::info("Camera initialized successfully");
        }
        
        void initializeDetector()
        {
            std::string classifier_path;
            std::string detector_path;
            std::string car_model_path;
            
            getParamCompat<std::string>({"detector_config.classifier_path", "detector_config/classifier_path"}, classifier_path, std::string(""));
            getParamCompat<std::string>({"detector_config.detector_path", "detector_config/detector_path"}, detector_path, std::string(""));
            getParamCompat<std::string>({"detector_config.car_model_path", "detector_config/car_model_path"}, car_model_path, std::string(""));
            
            if (!carAndArmorDetector.armorDetector.Corrector.Classifier.LoadModel(classifier_path)) {
                roslog::error("Failed to load classifier model: {}", classifier_path);
                throw std::runtime_error("Classifier model loading failed");
            }
            
            if (!carAndArmorDetector.armorDetector.Detector.LoadModel(detector_path)) {
                roslog::error("Failed to load detector model: {}", detector_path);
                throw std::runtime_error("Detector model loading failed");
            }
            
            if (!carAndArmorDetector.carDetector.LoadModel(car_model_path)) {
                roslog::error("Failed to load car model: {}", car_model_path);
                throw std::runtime_error("Car model loading failed");
            }
            
            // 设置队伍颜色
            getParamCompat<bool>({"detector_config.debug_team_blue", "detector_config/debug_team_blue"}, myTeamBlue, true);
            filter.is_team_red = !myTeamBlue;

            roslog::info("Detector modules initialized successfully");
        }

        void initializeAlgorithms()
        {
            try {
                // [ROS 2] 注意：tracker/solver/predictor/controller 內部可能需要節點指針
                // 這些指針已經在 main 函數中通過全局變量設置了
                tracker = ly_auto_aim::tracker::createTracker();
                solver = ly_auto_aim::solver::createSolver();
                predictor = ly_auto_aim::predictor::createPredictor();
                controller = ly_auto_aim::controller::createController();
                
                // 注册solver到location模块
                location::Location::registerSolver(solver);
                
                roslog::info("Algorithm modules initialized successfully");
            } catch (const std::exception& e) {
                roslog::error("Failed to initialize algorithms: {}", e.what());
                throw;
            }
        }
        
        void setupRosTopics()
        {
            // 订阅话题
            gimbal_sub_ = this->create_subscription<gimbal_driver::msg::GimbalAngles>(
                "/ly/gimbal/angles", 10,
                std::bind(&ShootingTableCalibNode::gimbalAngleCallback, this, std::placeholders::_1));

            // 初始化发布者
            control_pub_ = this->create_publisher<gimbal_driver::msg::GimbalAngles>("/ly/control/angles", 10);
            target_pub_ = this->create_publisher<auto_aim_common::msg::Target>("/ly/predictor/target", 10);
            firecode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/ly/control/firecode", 10);
        }

        void createCSVFile()
        {
            // 使用當前用戶的 home 目錄
            const char* home = std::getenv("HOME");
            std::string record_dir = home ? std::string(home) + "/workspace/record" : "./record";
            system(("mkdir -p " + record_dir).c_str());
            
            csv_filename = record_dir + "/shooting_table_" +
                           std::to_string(std::time(nullptr)) + ".csv";
            
            std::ofstream file(csv_filename);
            if (file.is_open()) {
                file << "timestamp,z_height,horizontal_distance,relative_yaw,relative_pitch,"
                     << "target_x,target_y,target_z,absolute_yaw,absolute_pitch,target_yaw,"
                     << "fitted_pitch,fitted_yaw\n";
                file.close();
                csv_file_created = true;
                roslog::info("Created CSV file: {}", csv_filename);
                std::cout << "✓ Created CSV file: " << csv_filename << "\n";
                std::cout << "✓ Calibration CSV directory: " << record_dir << "\n";
            } else {
                throw std::runtime_error("Failed to create CSV file: " + csv_filename);
            }
        }

    public:
        bool hasCreatedCsvFile() const
        {
            return csv_file_created;
        }

        const std::string& currentInitStage() const
        {
            return current_init_stage;
        }

        const std::string& currentCsvFile() const
        {
            return csv_filename;
        }

    private:
        void sendFireControlCommand()
        {
            // 构造火控数据包
            std_msgs::msg::UInt8 fire_msg;
            fire_msg.data = fire_control.fire_status;  
            
            firecode_pub_->publish(fire_msg);
            
            roslog::info("Fire control sent - FireCode: 0b{:08b} ({})", 
                         fire_msg.data, fire_msg.data);
        }

        void flipFireStatus()
        {
            fire_control.fire_status = (fire_control.fire_status == 0) ? 0b11 : 0b00;
            roslog::info("Fire status flipped to: 0b{:08b}", fire_control.fire_status);
        }

        void updateFireControl(bool should_fire)
        {
            if (should_fire != fire_control.last_fire_command) {
                if (should_fire) {
                    flipFireStatus();
                    std::cout << "🔥 Fire control activated - Status flipped!\n";
                } else {
                    std::cout << "🛑 Fire control deactivated\n";
                }
                fire_control.last_fire_command = should_fire;
                sendFireControlCommand();
            }
        }

        void sendAimOnlyCommand(bool verbose = true)
        {
            gimbal_driver::msg::GimbalAngles control_msg;
            control_msg.yaw = target_yaw + yaw_adjustment;
            control_msg.pitch = target_pitch + pitch_adjustment;
            
            control_pub_->publish(control_msg);

            if (verbose) {
                roslog::info("Aim command sent - Yaw: {:.2f}°, Pitch: {:.2f}°",
                             control_msg.yaw, control_msg.pitch);
                std::cout << "🎯 Aim only - Yaw: " << std::fixed << std::setprecision(2)
                          << control_msg.yaw << "°, Pitch: " << control_msg.pitch << "°\n";
            }
        }

        void sendControlCommand()
        {
            auto_aim_common::msg::Target target_msg;
            target_msg.header.stamp = this->now();
            target_msg.header.frame_id = "camera";
            
            target_msg.yaw = target_yaw + yaw_adjustment;
            target_msg.pitch = target_pitch + pitch_adjustment;
            target_msg.status = true;  
            target_msg.buff_follow = false;  
            
            target_pub_->publish(target_msg);
            
            roslog::info("Target command sent - Yaw: {:.2f}°, Pitch: {:.2f}°", 
                         target_msg.yaw, target_msg.pitch);
            std::cout << "🔥 Target published - Yaw: " << std::fixed << std::setprecision(2) 
                      << target_msg.yaw << "°, Pitch: " << target_msg.pitch << "°\n";
        }

        bool isDualAxisLockConverged(double* yaw_err_out = nullptr, double* pitch_err_out = nullptr) const
        {
            const double cmd_yaw = target_yaw + yaw_adjustment;
            const double cmd_pitch = target_pitch + pitch_adjustment;
            const double yaw_err = std::abs(std::remainder(cmd_yaw - current_gimbal_angles.yaw, 360.0));
            const double pitch_err = std::abs(cmd_pitch - current_gimbal_angles.pitch);

            if (yaw_err_out) {
                *yaw_err_out = yaw_err;
            }
            if (pitch_err_out) {
                *pitch_err_out = pitch_err;
            }

            return (yaw_err <= fire_max_yaw_error_deg) && (pitch_err <= fire_max_pitch_error_deg);
        }

        void sendStopCommand()
        {
            auto_aim_common::msg::Target stop_msg;
            stop_msg.header.stamp = this->now();
            stop_msg.header.frame_id = "camera";
            stop_msg.yaw = current_gimbal_angles.yaw;  
            stop_msg.pitch = current_gimbal_angles.pitch;
            stop_msg.status = false;  
            stop_msg.buff_follow = false;
            
            target_pub_->publish(stop_msg);
            
            updateFireControl(false);  
            
            roslog::info("Stop command sent");
            std::cout << "🛑 Stop shooting and fire control stopped\n";
        }

        void gimbalAngleCallback(const gimbal_driver::msg::GimbalAngles::SharedPtr msg)
        {
            current_gimbal_angles.yaw = msg->yaw;
            current_gimbal_angles.pitch = msg->pitch;
        }

        bool getImage(cv::Mat& image)
        {
            if (use_video && video_cap.isOpened()) {
                return video_cap.read(image);
            } else if (!use_video && !use_ros_bag) {
                return camera.GetImage(image);
            }
            return false;
        }

        void convertToDetections(const std::vector<ArmorObject>& armors, std::vector<Detection>& detections)
        {
            detections.clear();
            detections.reserve(armors.size());
            for (const auto& armor : armors) {
                detections.emplace_back(Detection{
                    .tag_id = armor.type,
                    .corners = {
                        {armor.apex[0].x, armor.apex[0].y},
                        {armor.apex[1].x, armor.apex[1].y},
                        {armor.apex[2].x, armor.apex[2].y},
                        {armor.apex[3].x, armor.apex[3].y}
                    }
                });
            }
        }

        void convertToCarDetections(const std::vector<CarDetection>& cars, std::vector<CarDetection>& car_detections)
        {
            car_detections = cars; 
        }

        void processImageDetections()
        {
            cv::Mat image;
            if (!getImage(image)) return;
            
            if (image.empty()) return;

            std::vector<ArmorObject> detected_armors;
            std::vector<CarDetection> cars;
            
            if (!carAndArmorDetector.Detect(image, detected_armors, cars)) {
                if (web_show) {
                    VideoStreamer::setFrame(image);
                }
                return;
            }

            std::vector<ArmorObject> filtered_armors;
            ArmorType target = selected_target_type;
            
            const bool has_filtered_target = filter.Filter(detected_armors, target, filtered_armors);
            if (!has_filtered_target) {
                if (draw_image) {
                    std::pair<std::vector<tracker::TrackResult>, std::vector<tracker::CarTrackResult>> empty_tracks;
                    drawDebugInfo(image, detected_armors, cars, empty_tracks);
                    cv::putText(
                        image,
                        "No filtered target (check team / target id)",
                        cv::Point(10, 90),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.65,
                        cv::Scalar(0, 0, 255),
                        2);
                }
                if (web_show) {
                    VideoStreamer::setFrame(image);
                }
                return;
            }

            std::vector<Detection> detections;
            std::vector<CarDetection> car_detections;
            convertToDetections(filtered_armors, detections);
            convertToCarDetections(cars, car_detections);

            GimbalAngleType gimbal_angle{current_gimbal_angles.pitch, current_gimbal_angles.yaw};
            
            tracker->merge(detections);
            tracker->merge(car_detections);
            
            // [ROS 2] 時間轉換
            auto track_results = tracker->getTrackResult(rclcpp::Time(this->now()), gimbal_angle);
            solver->solve_all(track_results, gimbal_angle);

            const auto* best_track =
                track_results.first.empty() ? nullptr : selectBestTrackForAim(track_results.first);

            if (auto_lock_fire) {
                updateAutoLockFire(best_track);
            }

            if (should_aim_once.load() && !track_results.first.empty()) {
                if (!best_track) {
                    std::cout << "✗ No valid track for aim. Try again.\n";
                    should_aim_once.store(false);
                    return;
                }

                XYZ target_xyz = best_track->location.xyz_imu;
                const double target_distance =
                    std::sqrt(target_xyz.x * target_xyz.x + target_xyz.y * target_xyz.y + target_xyz.z * target_xyz.z);
                roslog::info(
                    "Aim request - car_id={}, armor_id={}, xyz=({:.3f}, {:.3f}, {:.3f}), dist={:.3f}, gimbal=(yaw={:.2f}, pitch={:.2f})",
                    best_track->car_id,
                    best_track->armor_id,
                    target_xyz.x,
                    target_xyz.y,
                    target_xyz.z,
                    target_distance,
                    current_gimbal_angles.yaw,
                    current_gimbal_angles.pitch);
                
                if (calculateBallisticSolution(target_xyz)) {
                    current_target_world = cv::Point3d(target_xyz.x, target_xyz.y, target_xyz.z);
                    is_aiming.store(true);
                    control_valid.store(true);
                    aim_only_mode.store(true);
                    is_shooting.store(false);
                    should_aim_once.store(false);
                    
                    double distance = std::sqrt(target_xyz.x * target_xyz.x + target_xyz.y * target_xyz.y);
                    double fitted_pitch_val = fitPitch(target_xyz.z, distance);
                    double fitted_yaw_val = fitYaw(target_xyz.z, distance);
                    roslog::info(
                        "Aim solved - xyz=({:.3f}, {:.3f}, {:.3f}), solved=(yaw={:.2f}, pitch={:.2f}), adjust=(yaw={:.2f}, pitch={:.2f}), cmd=(yaw={:.2f}, pitch={:.2f})",
                        target_xyz.x,
                        target_xyz.y,
                        target_xyz.z,
                        target_yaw,
                        target_pitch,
                        yaw_adjustment,
                        pitch_adjustment,
                        target_yaw + yaw_adjustment,
                        target_pitch + pitch_adjustment);

                    // `a` should really "lock and point once", not just cache setpoints.
                    sendAimOnlyCommand();
                    updateFireControl(false);
                    
                    std::cout << "✓ Target locked! Control enabled\n";
                    std::cout << "  Track car/armor: " << best_track->car_id << "/" << best_track->armor_id << "\n";
                    std::cout << "  Target distance: " << distance << "m\n";
                    std::cout << "  Ballistic pitch: " << target_pitch << "°\n";
                    std::cout << "  Fitted pitch: " << fitted_pitch_val << "°\n";
                    std::cout << "  Fitted yaw: " << fitted_yaw_val << "°\n";
                } else {
                    roslog::warn(
                        "Aim solve failed - xyz=({:.3f}, {:.3f}, {:.3f}), gimbal=(yaw={:.2f}, pitch={:.2f})",
                        target_xyz.x,
                        target_xyz.y,
                        target_xyz.z,
                        current_gimbal_angles.yaw,
                        current_gimbal_angles.pitch);
                    std::cout << "✗ Ballistic calculation failed\n";
                    should_aim_once.store(false);
                }
            } else if (should_aim_once.load() && track_results.first.empty()) {
                std::cout << "✗ No target detected. Try again.\n";
                should_aim_once.store(false);
            }

            if (draw_image) {
                drawDebugInfo(image, filtered_armors, cars, track_results);
            }

            if (web_show) {
                VideoStreamer::setFrame(image);
            }
        }

        const tracker::TrackResult* selectBestTrackForAim(
            const std::vector<tracker::TrackResult>& tracks) const
        {
            const tracker::TrackResult* best_track = nullptr;
            double best_score = std::numeric_limits<double>::max();
            const double PI = 3.1415926;

            for (const auto& track : tracks) {
                XYZ xyz = track.location.xyz_imu;
                const double horizontal_distance = std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y);
                const double distance = std::sqrt(horizontal_distance * horizontal_distance + xyz.z * xyz.z);
                if (distance < 1e-6) {
                    continue;
                }

                double yaw_deg = std::atan2(xyz.y, xyz.x) * 180.0 / PI;
                yaw_deg = current_gimbal_angles.yaw +
                          std::remainder(yaw_deg - current_gimbal_angles.yaw, 360.0);
                const double pitch_deg = std::atan2(xyz.z, horizontal_distance) * 180.0 / PI;

                const double yaw_err = std::abs(std::remainder(yaw_deg - current_gimbal_angles.yaw, 360.0));
                const double pitch_err = std::abs(pitch_deg - current_gimbal_angles.pitch);
                const double score = yaw_err + 1.5 * pitch_err;

                if (score < best_score) {
                    best_score = score;
                    best_track = &track;
                }
            }

            return best_track;
        }

        void updateAutoLockFire(const tracker::TrackResult* best_track)
        {
            if (!auto_lock_fire) {
                return;
            }

            if (!best_track) {
                if (is_aiming.load() || control_valid.load() || fire_control.last_fire_command) {
                    std::cout << "✗ Auto target lost. Stop auto fire.\n";
                }
                is_shooting.store(false);
                aim_only_mode.store(false);
                control_valid.store(false);
                is_aiming.store(false);
                updateFireControl(false);
                return;
            }

            XYZ target_xyz = best_track->location.xyz_imu;
            if (!calculateBallisticSolution(target_xyz)) {
                if (control_valid.load()) {
                    std::cout << "✗ Auto ballistic calculation failed. Stop auto fire.\n";
                }
                is_shooting.store(false);
                aim_only_mode.store(false);
                control_valid.store(false);
                is_aiming.store(false);
                updateFireControl(false);
                return;
            }

            current_target_world = cv::Point3d(target_xyz.x, target_xyz.y, target_xyz.z);
            is_aiming.store(true);
            control_valid.store(true);
            aim_only_mode.store(true);
            is_shooting.store(false);

            double yaw_err = 0.0;
            double pitch_err = 0.0;
            const bool lock_ok = isDualAxisLockConverged(&yaw_err, &pitch_err);
            if (auto_fire) {
                updateFireControl(lock_ok);
            } else {
                updateFireControl(false);
            }

            static auto last_auto_log = std::chrono::steady_clock::time_point{};
            const auto now = std::chrono::steady_clock::now();
            if (now - last_auto_log > std::chrono::milliseconds(500)) {
                std::cout << "AUTO lock "
                          << armorTypeName(selected_target_type)
                          << " car/armor=" << best_track->car_id << "/" << best_track->armor_id
                          << " cmd(yaw=" << std::fixed << std::setprecision(2)
                          << (target_yaw + yaw_adjustment)
                          << ", pitch=" << (target_pitch + pitch_adjustment)
                          << ") lock=" << (lock_ok ? "yes" : "no")
                          << " yaw_err=" << yaw_err
                          << " pitch_err=" << pitch_err
                          << "\n";
                last_auto_log = now;
            }
        }

        void drawDebugInfo(cv::Mat& image, const std::vector<ArmorObject>& armors, 
                          const std::vector<CarDetection>& cars, 
                          const std::pair<std::vector<tracker::TrackResult>, std::vector<tracker::CarTrackResult>>& track_results)
        {
            int center_x = image.cols / 2;
            int center_y = image.rows / 2;
            cv::line(image, cv::Point(center_x - 20, center_y), 
                     cv::Point(center_x + 20, center_y), cv::Scalar(0, 255, 0), 2);
            cv::line(image, cv::Point(center_x, center_y - 20), 
                     cv::Point(center_x, center_y + 20), cv::Scalar(0, 255, 0), 2);
            
            std::string adj_text = "Adj Y:" + std::to_string(yaw_adjustment) + 
                                  " P:" + std::to_string(pitch_adjustment) +
                                  (control_valid.load() ? " [CTRL ON]" : " [CTRL OFF]");
            cv::putText(image, adj_text, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

            const std::string stat_text =
                "armor=" + std::to_string(armors.size()) +
                " car=" + std::to_string(cars.size()) +
                " track=" + std::to_string(track_results.first.size());
            cv::putText(image, stat_text, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2);

            for (const auto& armor : armors) {
                for (const auto& point : armor.apex) {
                    cv::circle(image, point, 4, cv::Scalar(0, 0, 255), -1);
                }
                cv::line(image, armor.apex[0], armor.apex[2], cv::Scalar(255, 0, 0), 2);
                cv::line(image, armor.apex[1], armor.apex[3], cv::Scalar(0, 255, 0), 2);

                std::vector<cv::Point2f> pts(armor.apex, armor.apex + 4);
                cv::Rect bbox = cv::boundingRect(pts);
                cv::rectangle(image, bbox, cv::Scalar(0, 200, 255), 2);

                const std::string armor_text =
                    "type=" + std::to_string(armor.type) +
                    " " + (armor.ActualColor() == ArmorObject::Blue ? "B" : "R");
                cv::putText(image, armor_text, cv::Point(bbox.x, std::max(20, bbox.y - 8)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2);
            }

            for (const auto& car : cars) {
                cv::Rect car_box = car.bounding_rect;
                cv::rectangle(image, car_box, cv::Scalar(0, 255, 0), 2);
                const std::string car_text = "car id=" + std::to_string(car.tag_id);
                cv::putText(image, car_text, cv::Point(car_box.x, std::max(20, car_box.y - 8)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
            }
        }

        bool calculateBallisticSolution(const XYZ& target_xyz)
        {
            const double PI = 3.1415926;
            const double GRAVITY = 9.794;
            const double C_D = 0.42;
            const double RHO = 1.169;
            const double bullet_mass = 3.2e-3;
            const double bullet_diameter = 16.8e-3;
            const double bullet_speed = 23.0;
            const double tol = 1e-6;
            const int max_iter = 100;
            
            double distance = std::sqrt(target_xyz.x * target_xyz.x + target_xyz.y * target_xyz.y);
            double theta = 0.0;
            double delta_z = 0.0;
            double k1 = C_D * RHO * (PI * bullet_diameter * bullet_diameter) / 8 / bullet_mass;
            
            bool calc_success = false;
            for (int i = 0; i < max_iter; i++) {
                double t = (exp(k1 * distance) - 1) / (k1 * bullet_speed * cos(theta));
                delta_z = target_xyz.z - bullet_speed * sin(theta) * t / cos(theta) + 
                          0.5 * GRAVITY * t * t / cos(theta) / cos(theta);
                
                if (fabs(delta_z) < tol) {
                    calc_success = true;
                    break;
                }
                
                theta -= delta_z / (-(bullet_speed * t) / pow(cos(theta), 2) + 
                                   GRAVITY * t * t / (bullet_speed * bullet_speed) * sin(theta) / pow(cos(theta), 3));
            }
            
            if (calc_success) {
                double aim_pitch_rad = theta;
                double aim_yaw_rad = atan2(target_xyz.y, target_xyz.x);
                
                target_pitch = aim_pitch_rad * 180 / PI;
                target_yaw = aim_yaw_rad * 180 / PI;

                if (shoot_table_params.enable) {
                    target_pitch += fitPitch(target_xyz.z, distance);
                    target_yaw += fitYaw(target_xyz.z, distance);
                }
                
                target_yaw = current_gimbal_angles.yaw + 
                             std::remainder(target_yaw - current_gimbal_angles.yaw, 360.0);
                return true;
            }
            return false;
        }

        double fitPitch(double z_height, double horizontal_distance)
        {
            const auto& p = shoot_table_params.pitch;
            double z2 = z_height * z_height;
            double d2 = horizontal_distance * horizontal_distance;
            double zd = z_height * horizontal_distance;
            return p.intercept + p.coef_z * z_height + p.coef_d * horizontal_distance + 
                   p.coef_z2 * z2 + p.coef_zd * zd + p.coef_d2 * d2;
        }

        double fitYaw(double z_height, double horizontal_distance)
        {
            const auto& y = shoot_table_params.yaw;
            double z2 = z_height * z_height;
            double d2 = horizontal_distance * horizontal_distance;
            double zd = z_height * horizontal_distance;
            return y.intercept + y.coef_z * z_height + y.coef_d * horizontal_distance + 
                   y.coef_z2 * z2 + y.coef_zd * zd + y.coef_d2 * d2;
        }

        void checkAndUpdateParams()
        {
            rclcpp::Time current_time = this->now();
            if ((current_time - last_param_check).seconds() >= param_check_interval) {
                loadShootTableParams();
                last_param_check = current_time;
            }
        }
        
        void printCurrentParams()
        {
            // ... (保持不變) ...
            std::cout << "Params printed.\n";
        }
        
        void setParameter(const std::string& param_path, double value)
        {
            // ROS 2 parameter set
            std::string full_path = "shoot_table_adjust." + param_path;
            // Note: In ROS 2, modifying params programmatically on self requires declare or set_parameter
            this->set_parameter(rclcpp::Parameter(full_path, value));
            loadShootTableParams();
            std::cout << "✓ Parameter updated: " << param_path << " = " << value << "\n";
        }
        
        void interactiveParamEdit()
        {
            // ... (保持輸入邏輯，但調用 setParameter) ...
            std::cout << "Interactive edit not fully ported, check code.\n";
        }

        void saveShootingRecord()
        {
            if (!is_aiming.load()) {
                std::cout << "✗ No target locked. Cannot save record.\n";
                return;
            }

            try {
                ShootingRecord record;
                record.z_height = current_target_world.z;
                record.horizontal_distance = std::sqrt(current_target_world.x * current_target_world.x + 
                                                      current_target_world.y * current_target_world.y);
                record.relative_yaw = yaw_adjustment;
                record.relative_pitch = pitch_adjustment;
                record.target_world_coord = current_target_world;
                record.absolute_yaw = current_gimbal_angles.yaw;
                record.absolute_pitch = current_gimbal_angles.pitch;
                record.target_yaw = target_yaw;
                record.fitted_pitch = fitPitch(record.z_height, record.horizontal_distance);
                record.fitted_yaw = fitYaw(record.z_height, record.horizontal_distance);
                record.timestamp = this->now();
                
                std::ofstream file(csv_filename, std::ios::app);
                if (!file.is_open()) {
                    throw std::runtime_error("Failed to open CSV file: " + csv_filename);
                }
                file << std::fixed << std::setprecision(6)
                     << record.timestamp.seconds() << ","
                     << record.z_height << ","
                     << record.horizontal_distance << ","
                     << record.relative_yaw << ","
                     << record.relative_pitch << ","
                     << record.target_world_coord.x << ","
                     << record.target_world_coord.y << ","
                     << record.target_world_coord.z << ","
                     << record.absolute_yaw << ","
                     << record.absolute_pitch << ","
                     << record.target_yaw << ","
                     << record.fitted_pitch << ","
                     << record.fitted_yaw << std::endl;
                file.close();
                records.push_back(record);
                std::cout << "✓ Record saved! samples=" << records.size()
                          << ", csv=" << csv_filename << "\n";
                
                is_shooting.store(false);
                aim_only_mode.store(false);
                sendStopCommand();
                is_aiming.store(false);
                control_valid.store(false);
            } catch (const std::exception& e) {
                std::cout << "✗ Error saving record: " << e.what() << "\n";
            }
        }

        void handleKeyboard()
        {
            char key = keyboard.getKey();
            if (key == 0) return;
            std::cout << "Key pressed: '" << key << "'\n";
            
            switch (key) {
                case 'a': should_aim_once.store(true); break;
                case 'g': 
                    if (is_aiming.load()) {
                        aim_only_mode.store(true);
                        is_shooting.store(false);
                        sendAimOnlyCommand();
                        updateFireControl(false);
                    }
                    break;
                case 'f':
                    if (is_aiming.load()) {
                        double yaw_err = 0.0;
                        double pitch_err = 0.0;
                        const bool lock_ok = isDualAxisLockConverged(&yaw_err, &pitch_err);
                        if (fire_require_dual_axis_lock && !lock_ok) {
                            std::cout << "✗ Fire blocked: lock not converged (yaw_err=" << std::fixed << std::setprecision(2)
                                      << yaw_err << "°, pitch_err=" << pitch_err << "°)\n";
                            updateFireControl(false);
                            break;
                        }
                        aim_only_mode.store(false);
                        is_shooting.store(true);
                        shoot_start_time = this->now();
                        sendControlCommand();
                        updateFireControl(true);
                    }
                    break;
                case 't': 
                    // Test publish
                    {
                        gimbal_driver::msg::GimbalAngles test_msg;
                        test_msg.yaw = current_gimbal_angles.yaw + yaw_adjustment;
                        test_msg.pitch = current_gimbal_angles.pitch + pitch_adjustment;
                        test_msg.header.stamp = this->now();
                        control_pub_->publish(test_msg);
                        // ... fire ...
                        flipFireStatus();
                        sendFireControlCommand();
                        // ... target ...
                        auto_aim_common::msg::Target test_target;
                        test_target.header.stamp = this->now();
                        test_target.yaw = test_msg.yaw;
                        test_target.pitch = test_msg.pitch;
                        target_pub_->publish(test_target);
                    }
                    break;
                case 'x':
                    is_shooting.store(false);
                    aim_only_mode.store(false);
                    updateFireControl(false);
                    sendStopCommand();
                    break;
                case 'w': pitch_adjustment += adjustment_step; break;
                case 's': pitch_adjustment -= adjustment_step; break;
                case 'd': yaw_adjustment += adjustment_step; break;
                case 'j': yaw_adjustment -= adjustment_step; break;
                case 'h':
                    std::cout << "💾 Saving shooting record to " << csv_filename << " ...\n";
                    saveShootingRecord();
                    break;
                case 'r':
                    pitch_adjustment = 0; yaw_adjustment = 0;
                    control_valid = false; is_aiming = false; is_shooting = false;
                    fire_control.fire_status = 0;
                    sendFireControlCommand();
                    sendStopCommand();
                    break;
                case 'l': loadShootTableParams(); break;
                case 'q':
                    is_shooting = false; is_aiming = false;
                    updateFireControl(false);
                    sendStopCommand();
                    keyboard.restore();
                    rclcpp::shutdown();
                    break;
            }
        }

        void printInstructions() {
            std::cout << "=== Controls ===\n";
            std::cout << "a:Aim g:AimOnly f:Fire x:Stop w/s/d/j:Adjust h:Save r:Reset l:Reload q:Quit\n";
            if (auto_lock_fire) {
                std::cout << "AUTO mode enabled: target=" << armorTypeName(selected_target_type)
                          << ", auto_fire=" << (auto_fire ? "true" : "false")
                          << ", fire_guard_dual_axis=" << (fire_require_dual_axis_lock ? "true" : "false")
                          << "\n";
                std::cout << "AUTO mode behavior: continuously lock like repeated 'a', keep publishing /ly/control/angles, and only toggle fire after dual-axis lock converges.\n";
            }
        }

    public:
        void spin_loop()
        {
            rclcpp::Rate rate(78);
            
            std::cout << "🚀 ROS 2 Shooting Table Calib Node Started\n";
            
            // Initial publish
            gimbal_driver::msg::GimbalAngles init_msg;
            init_msg.header.stamp = this->now();
            control_pub_->publish(init_msg);
            
            while (rclcpp::ok()) {
                try {
                    handleKeyboard();
                    checkAndUpdateParams();
                    processImageDetections();
                    
                    if (is_shooting.load()) {
                        if (fire_require_dual_axis_lock) {
                            double yaw_err = 0.0;
                            double pitch_err = 0.0;
                            if (!isDualAxisLockConverged(&yaw_err, &pitch_err)) {
                                is_shooting.store(false);
                                updateFireControl(false);
                                sendStopCommand();
                                std::cout << "✗ Fire stopped: lock lost (yaw_err=" << std::fixed << std::setprecision(2)
                                          << yaw_err << "°, pitch_err=" << pitch_err << "°)\n";
                                rclcpp::spin_some(this->get_node_base_interface());
                                rate.sleep();
                                continue;
                            }
                        }
                        rclcpp::Time current_time = this->now();
                        if ((current_time - shoot_start_time).seconds() >= shoot_duration) {
                            is_shooting.store(false);
                            updateFireControl(false);
                            sendStopCommand();
                        } else {
                            sendControlCommand();
                        }
                    } else if (aim_only_mode.load() && is_aiming.load() && control_valid.load()) {
                        // Keep publishing hold-aim command for stable lock during calibration.
                        sendAimOnlyCommand(false);
                    }
                    
                    rclcpp::spin_some(this->get_node_base_interface());
                    rate.sleep();
                } catch (const std::exception& e) {
                    if (!rclcpp::ok()) {
                        break;
                    }
                    std::cerr << "Error loop: " << e.what() << std::endl;
                }
            }
        }
    };
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ShootingTableCalibNode>();
    
    // [ROS 2] 初始化全局指針 (讓算法庫能工作)
    global_node_ptr = node;
    ly_auto_aim::solver::global_tracker_solver_node = node;
    ly_auto_aim::predictor::global_predictor_node = node;
    // 這裡我們把 controller 的全局變量也指過去 (如果有的話)
    ly_auto_aim::controller::global_controller_node = node;
    
    try {
        node->Initialize();
        node->spin_loop();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        if (node->hasCreatedCsvFile()) {
            std::cerr << "✗ Node exited after CSV creation. Current CSV: "
                      << node->currentCsvFile() << std::endl;
        } else {
            std::cerr << "✗ Node exited before CSV creation. Last init stage: "
                      << node->currentInitStage() << std::endl;
            std::cerr << "✗ No new calibration CSV was created under $HOME/workspace/record" << std::endl;
        }
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
