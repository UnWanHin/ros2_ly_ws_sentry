#pragma once

#include <ceres/ceres.h>

#include <chrono>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <random>
#include <shared_mutex> //MUTEX
#include <thread>

#include <Param/param.hpp>

#define ANGLE_BETWEEN_FAN_BLADES (72 * CV_PI / 180)
#define MIN_FIT_DATA_SIZE 20
#define MAX_FIT_DATA_SIZE 12000

enum class Direction { UNKNOWN, STABLE, ANTI_CLOCKWISE, CLOCKWISE };  // 旋转方向
enum class Convexity { UNKNOWN, CONCAVE, CONVEX };                    // 拟合曲线凹凸性

#define CONSOLE_OUTPUT 2

static std::atomic<bool> STOP_THREAD(false);
static std::atomic<bool> VALID_PARAMS(false);
static std::mutex MUTEX; //MUTEX
extern std::mutex MUTEX;

namespace power_rune {
// define Eigen5d
typedef Eigen::Matrix<double, 5, 1> Vector5d;
//type
inline double angle2Radian(double angle) noexcept { return angle * CV_PI / 180; }
inline double radian2Angle(double radian) noexcept { return radian * 180 / CV_PI; }
inline std::pair<double, double> solveQuadraticEquation(double a, double b, double c) {
    std::pair<double, double> result((-b - sqrt((double)(b * b - 4 * a * c))) / (2 * a),
                                     (-b + sqrt((double)(b * b - 4 * a * c))) / (2 * a));
    return result;
}

struct Frame {
    Frame() = default;
    Frame(const cv::Mat& image, const std::chrono::steady_clock::time_point& time, double pitch, double yaw, double roll)
        : m_image{image}, m_time{time}, m_roll{roll}, m_pitch{pitch}, m_yaw{yaw} {}
    cv::Mat m_image;
    std::chrono::steady_clock::time_point m_time;
    double m_roll, m_pitch, m_yaw;
    void set(const cv::Mat& image, const std::chrono::steady_clock::time_point& time, double pitch,
             double yaw, double roll) {
        m_image = image;
        m_time = time;
        m_roll = roll;
        m_pitch = pitch;
        m_yaw = yaw;
    }
};

class BuffCalculator {
    public:
        Frame buff_frame;
        bool is_shift = false;
        explicit BuffCalculator(const param::Param& json_param) {
            // : m_worldPoints{{(float)(-0.5 * Param::ARMOR_INSIDE_WIDTH), (float)Param::ARMOR_INSIDE_Y, 0.0},
            //                 {(float)(0.5 * Param::ARMOR_INSIDE_WIDTH), (float)Param::ARMOR_INSIDE_Y, 0.0},
            //                 {0.0, (float)(-Param::ARMOR_OUTSIDE_Y - Param::ARMOR_OUTSIDE_HEIGHT), 0.0},
            //                 {(float)(-0.5 * Param::ARMOR_SIZE), (float)-Param::ARMOR_OUTSIDE_Y, 0.0},
            //                 {(float)(0.5 * Param::ARMOR_SIZE), (float)-Param::ARMOR_OUTSIDE_Y, 0.0},
            //                 {0.0, (float)Param::POWER_RUNE_RADIUS, 0.0}},
            
            m_direction = Direction::UNKNOWN;
            m_convexity = Convexity::UNKNOWN;
            m_totalShift = 0;
            m_firstDetect = true;
            m_angleLast = 0;
            m_worldPoints = {{0.0, (float)(json_param["buff"]["power_radius"].Double()), 0.0}, //R标
                            {0.0, 0.0, 0.0}, //裝甲板中心
                            {0.0, (float)(-0.5 * json_param["buff"]["armor_width"].Double()), 0.0}, //4點外點
                            {(float)(0.5 * json_param["buff"]["armor_width"].Double()), 0.0, 0.0},
                            {0.0, (float)(0.5 * json_param["buff"]["armor_width"].Double()), 0.0},
                            {(float)(-0.5 * json_param["buff"]["armor_width"].Double()), 0.0, 0.0}};
            FPS = json_param["buff"]["buff_calculate_FPS"].Double();
            // m_directionThresh = {100 / (1000 / FPS) < 2 ? 2 : 100 / (1000 / FPS)};
            m_directionThresh = {static_cast<int>(100 / (1000 / FPS) < 2.0 ? 2 : 100 / (1000 / FPS))};
            COMPANSATE_TIME = json_param["buff"]["COMPANSATE_TIME"].Double();
            COMPANSATE_PITCH = json_param["buff"]["COMPANSATE_PITCH"].Double();
            COMPANSATE_YAW = json_param["buff"]["COMPANSATE_YAW"].Double();
            GRAVITY = json_param["buff"]["GRAVITY"].Double();
            
            m_fitData.reserve(FPS * 20);
            m_fitThread = std::thread(&BuffCalculator::fit, this);

            auto intrinsicArray = json_param["solver"]["camera_intrinsic_matrix"].to<std::vector<std::vector<double>>>();
            Eigen::Matrix3d cameraIntrinsicMatrix;
            Vector5d distorationCoefficients;// k1,k2,p1,p2,k3
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    cameraIntrinsicMatrix(i, j) = intrinsicArray[i][j];
            auto distortionCoefficientsArray = json_param["solver"]["camera_distortion_matrix"].to<std::vector<double>>();
            for (int i = 0; i < 5; ++i)
                distorationCoefficients(i) = distortionCoefficientsArray[i];

            cv::eigen2cv(cameraIntrinsicMatrix, cameraMatrix);
            cv::eigen2cv(distorationCoefficients, distCoeffs);

            tvec_c2g_data = json_param["buff"]["tvec_c2g"].to<std::vector<double>>();
            
            power_radius = json_param["buff"]["power_radius"].Double();
            armorWorld = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
            centerWorld = (cv::Mat_<double>(4, 1) << 0, json_param["buff"]["power_radius"].Double(), 0, 1);
            small_power_rune_rotation_speed = json_param["buff"]["SMALL_POWER_RUNE_ROTATION_SPEED"].Double();

        }

        ~BuffCalculator() {
            STOP_THREAD.store(true);
            m_fitThread.join();
        }

        std::pair<double, double> getBuffResult(){
            std::pair<double, double> result_pitch_yaw(this->m_predictPitch, this->m_predictYaw);
            return result_pitch_yaw;
        }

        bool calculate(const Frame &frame, std::vector<cv::Point2f> &cameraPoints, const float &actual_bullet_speed);
        inline cv::Point3f getPredictRobot() { return m_predictRobot; }
        inline cv::Point2f getPredictPixel() { return m_predictPixel; }
        inline std::pair<double, double> getPredictPitchYaw() {
            return std::make_pair(m_predictPitch, m_predictYaw);
        }
        cv::Point2f getPixelFromCamera(const cv::Mat &intrinsicMatrix, const cv::Mat &cameraPoint);
        cv::Point2f getPixelFromRobot(const cv::Point3f &robot, const cv::Mat &w2c, const cv::Mat &w2r);
        std::pair<double, double> getPitchYawFromRobotCoor(const cv::Point3f &target, double bulletSpeed);
        double get_predictPitch(){
            return m_predictPitch;
        }
        double get_predictYaw(){
            return m_predictYaw;
        }

    private:
        // void preprocess(const Frame &frame, std::vector<cv::Point2f> &cameraPoints, const float actual_bullet_speed);
        bool matrixCal();
        void setFirstDetect();
        void angleCal();
        void directionCal();
        bool predict();
        void fit();
        bool fitOnce();

        // Eigen::Matrix3d cameraIntrinsicMatrix;
        // Vector5d distorationCoefficients;// k1,k2,p1,p2,k3
        cv::Mat cameraMatrix, distCoeffs;
        std::vector<double> tvec_c2g_data;

        std::vector<cv::Point2f> m_cameraPoints;
        std::vector<cv::Point3f> m_worldPoints;
        Direction m_direction;                              // 旋转方向
        Convexity m_convexity;                              // 拟合数据凹凸性
        int m_totalShift;                                   // 总体的装甲板切换数
        double m_bulletSpeed;                               // 子弹速度
        std::chrono::steady_clock::time_point m_frameTime;  // 当前帧的时间戳
        std::chrono::steady_clock::time_point m_startTime;  // 开始的时间戳
        double COMPANSATE_TIME;
        double COMPANSATE_PITCH;
        double COMPANSATE_YAW;
        double GRAVITY;
        double FPS;
        int count;

        cv::Mat armorWorld;
        cv::Mat centerWorld;

        bool m_firstDetect;     // 第一次检测的标志位，第一次检测有效之后置为 true
        cv::Mat m_matW2C;       // 世界坐标系转相机坐标系的 4x4 变换矩阵
        cv::Mat m_matC2G;       // 相机坐标系转云台坐标系的 4x4 变换矩阵
        cv::Mat m_matG2R;       // 云台坐标系转机器人坐标系的 4x4 变换矩阵
        cv::Mat m_matW2R;       // 世界坐标系转机器人坐标系的 4x4 变换矩阵
        cv::Mat m_rMatW2R;      // 世界坐标系转机器人坐标系的 3x3 旋转矩阵
        cv::Mat m_rMatW2RBase;  // 第一次检测有效的世界坐标系转机器人坐标系的 3x3 旋转矩阵
        double m_angleRel;   // 这一帧相对于第一帧的旋转角度（去除了装甲板切换的影响）
        double m_angleLast;  // 上一帧相对于第一帧的旋转角度（不考虑装甲板切换
        double m_distance2Target;
        std::vector<std::pair<double, double>> m_fitData;  // 拟合数据
        std::vector<double> m_directionData;               // 计算旋转方向的数据
        std::array<double, 5> m_params;                    // 拟合参数
        cv::Point3f m_armorRobot;                          // 装甲板中心的机器人坐标
        cv::Point3f m_centerRobot;                         // 中心 R 的机器人坐标
        cv::Point3f m_predictRobot;                        // 预测击打的机器人坐标
        cv::Point2f m_predictPixel;                        // 预测击打的像素坐标
        double m_receiveRoll;                              // 当前帧的roll
        double m_receivePitch;                             // 当前帧的pitch
        double m_receiveYaw;                               // 当前帧的yaw
        double m_predictPitch;
        double m_predictYaw;
        int m_directionThresh;
        std::thread m_fitThread;
        std::shared_mutex m_mutex; //MUTEX

        double small_power_rune_rotation_speed;
        double power_radius;
        //    m_predictPitch = predictPitch;
        //    m_predictYaw = predictYaw;
};

/**
 * @brief 惩罚项，让拟合的参数更加贴近预设的参数
 */
class CostFunctor1 : public ceres::SizedCostFunction<1, 5> {
   public:
    CostFunctor1(double truth_, int id_) : truth(truth_), id(id_) {}
    virtual ~CostFunctor1(){};
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        double pre = parameters[0][id];
        residuals[0] = pre - truth;
        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                for (int i = 0; i < 5; ++i) {
                    if (i == id) {
                        jacobians[0][i] = 1;
                    } else {
                        jacobians[0][i] = 0;
                    }
                }
            }
        }
        return true;
    }
    double truth;
    int id;
};

/**
 * @brief 拟合项
 */
class CostFunctor2 : public ceres::SizedCostFunction<1, 5> {
   public:
    CostFunctor2(double t_, double y_) : t(t_), y(y_) {}
    virtual ~CostFunctor2(){};
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        double a = parameters[0][0];
        double w = parameters[0][1];
        double t0 = parameters[0][2];
        double b = parameters[0][3];
        double c = parameters[0][4];
        double cs = cos(w * (t + t0));
        double sn = sin(w * (t + t0));
        residuals[0] = -a * cs + b * t + c - y;
        if (jacobians != NULL) {
            if (jacobians[0] != NULL) {
                jacobians[0][0] = -cs;
                jacobians[0][1] = a * (t + t0) * sn;
                jacobians[0][2] = a * w * sn;
                jacobians[0][3] = t;
                jacobians[0][4] = 1;
            }
        }
        return true;
    }
    double t, y;
};

cv::Mat world2Camera(const std::vector<cv::Point3f> &worldPoints,
                     const std::vector<cv::Point2f> &cameraPoints, const cv::Mat &intrinsicMatrix,
                     const cv::Mat &distCoeffs);
cv::Mat camera2Gimbal(const std::array<double, 3> &rVec, const std::array<double, 3> &tVec);
cv::Mat gimbal2Robot(double pitch, double yaw, double roll);
// cv::Point2f getPixelFromCamera(const cv::Mat &intrinsicMatrix, const cv::Mat &cameraPoint);
// cv::Point2f getPixelFromRobot(const cv::Point3f &robot, const cv::Mat &w2c, const cv::Mat &w2r);
Convexity getConvexity(const std::vector<std::pair<double, double>> &data);
std::array<double, 5> ransacFitting(const std::vector<std::pair<double, double>> &data, Convexity convexity);
std::array<double, 5> leastSquareEstimate(const std::vector<std::pair<double, double>> &data,
                                          const std::array<double, 5> &params, Convexity convexity);
// std::pair<double, double> getPitchYawFromRobotCoor(const cv::Point3f &target, double bulletSpeed);

/**
 * @brief 得到小符旋转角
 * @param[in] distance      到装甲板中心的距离
 * @param[in] bulletSpeed   弹速
 * @param[in] PMSpeed       小符旋转速度
 * @param[in] compansate    时间补偿
 * @return double
 */
inline double getRotationAngleSmall(double distance, double bulletSpeed, double rotationSpeed,
                                    double compansate) noexcept {
    std::cout<< "rotationSpeed: "<<rotationSpeed<<
    "distance: "<< distance<<"bulletSpeed:"<<bulletSpeed << "compansate:"<<compansate<<std::endl;
    return rotationSpeed * (distance / bulletSpeed + compansate / 1e3);
}

/**
 * @brief 得到大符角度，注意这里是利用参数计算出来的，相对于第一次识别的角度
 * @param[in] time          时间
 * @param[in] params        参数
 * @return double
 */
inline double getAngleBig(double time, const std::array<double, 5> &params) noexcept {
    // angle = −Acos(ω(t+ϕ))+Bt+C
    return -params[0] * std::cos(params[1] * (time + params[2])) + params[3] * time + params[4];
}

/**
 * @brief 得到大符旋转角度，这里是相对于最后一次识别，预测的旋转角
 * @param[in] distance      到装甲板中心的距离
 * @param[in] bulletSpeed   弹速
 * @param[in] params        参数
 * @param[in] compansate    补偿
 * @param[in] frameTime     最后一次识别的时间戳
 * @return double
 */
inline double getRotationAngleBig(double distance, double bulletSpeed, const std::array<double, 5> &params,
                                  double compansate, int64_t frameTime) noexcept {
    double predictTime{distance / bulletSpeed + (frameTime + compansate) * 1e-3}; // 子弹飞行时间+最后一次识别时间+补偿时间(秒)
    double currentTime{frameTime * 1e-3};
    return getAngleBig(predictTime, params) - getAngleBig(currentTime, params); //predict - current
}

/**
 * @brief 两个二维点间距离
 * @param[in] pt1
 * @param[in] pt2
 * @return double
 */
inline double pointPointDistance(const cv::Point2f& pt1, const cv::Point2f& pt2) noexcept {
    return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

/**
 * @brief 两个三维点间距离
 * @param[in] pt1
 * @param[in] pt2
 * @return double
 */
inline double pointPointDistance(const cv::Point3f& pt1, const cv::Point3f& pt2) noexcept {
    return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y) +
                     (pt1.z - pt2.z) * (pt1.z - pt2.z));
}

}  // namespace power_rune
