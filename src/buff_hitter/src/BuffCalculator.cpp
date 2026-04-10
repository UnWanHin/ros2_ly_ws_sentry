#include <atomic>
#include <cmath>
#include <mutex>

#include "BuffCalculator.hpp"
#include <Logger/Logger.hpp>
#include "Param/param.hpp"

namespace power_rune {

void BuffCalculator::loadShootTableAdjust(const param::Param& json_param) {
    if (json_param.exists("buff") && json_param["buff"].exists("shoot_table_adjust")) {
        auto static_param = json_param["buff"]["shoot_table_adjust"];
        static_shoot_table_adjust_enable = static_param["enable"].Bool();
        if (static_shoot_table_adjust_enable) {
            const char* static_keys[] = {"intercept", "coef_z", "coef_d", "coef_z2", "coef_zd", "coef_d2"};
            for (std::size_t i = 0; i < sizeof(static_keys) / sizeof(static_keys[0]); ++i) {
                static_pitch_adjust_param[i] = static_param["pitch"][static_keys[i]].Double();
                static_yaw_adjust_param[i] = static_param["yaw"][static_keys[i]].Double();
            }
            roslog::info("buff static shoot table adjust enabled from buff.shoot_table_adjust");
        }
    }

    if (!json_param["buff"].exists("buff_shooting_table_calib")) {
        return;
    }

    auto calib_param = json_param["buff"]["buff_shooting_table_calib"];
    buff_shoot_table_adjust_enable = calib_param["enable"].Bool();
    if (calib_param.exists("apply_on_big_buff_only")) {
        buff_shoot_table_big_only = calib_param["apply_on_big_buff_only"].Bool();
    }

    if (!buff_shoot_table_adjust_enable) {
        return;
    }

    const char* keys[] = {"intercept", "coef_sin", "coef_cos", "coef_sin2",
                          "coef_cos2", "coef_dist", "coef_height"};
    for (std::size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
        buff_pitch_adjust_param[i] = calib_param["pitch"][keys[i]].Double();
        buff_yaw_adjust_param[i] = calib_param["yaw"][keys[i]].Double();
    }

    roslog::info("buff shooting table calib enabled: big_only={}", buff_shoot_table_big_only);
}

double BuffCalculator::fitStaticShootTableDeg(const std::array<double, 6>& coeffs, double z_height_m,
                                              double horizontal_distance_m) const {
    const double z2 = z_height_m * z_height_m;
    const double d2 = horizontal_distance_m * horizontal_distance_m;
    const double zd = z_height_m * horizontal_distance_m;
    return coeffs[0] +
           coeffs[1] * z_height_m +
           coeffs[2] * horizontal_distance_m +
           coeffs[3] * z2 +
           coeffs[4] * zd +
           coeffs[5] * d2;
}

double BuffCalculator::fitBuffPeriodicDeg(const std::array<double, 7>& coeffs, double rotation_angle,
                                          double distance_m, double height_m) const {
    const double sin_theta = std::sin(rotation_angle);
    const double cos_theta = std::cos(rotation_angle);
    const double sin_2theta = std::sin(2.0 * rotation_angle);
    const double cos_2theta = std::cos(2.0 * rotation_angle);
    return coeffs[0] +
           coeffs[1] * sin_theta +
           coeffs[2] * cos_theta +
           coeffs[3] * sin_2theta +
           coeffs[4] * cos_2theta +
           coeffs[5] * distance_m +
           coeffs[6] * height_m;
}

/**
 * @brief 解算，分别进行预处理，矩阵解算，设置第一次检测，角度解算，旋转方向解算和预测
 */
bool BuffCalculator::calculate(const Frame &frame, std::vector<cv::Point2f> &cameraPoints, int buff_mode, const float &actual_bullet_speed, const bool reload_big_buff) {
    m_cameraPoints = cameraPoints;
    m_frameTime = frame.m_time;
    m_receiveRoll = frame.m_roll;
    mode_auto_request_ = (buff_mode == 0);
    m_buff_mode = mode_auto_request_ ? estimated_buff_mode_ : buff_mode;
    m_receivePitch = frame.m_pitch + COMPANSATE_PITCH;
    m_receiveYaw = frame.m_yaw + COMPANSATE_YAW;
    std::cout<<"m_receiveRoll:"<<m_receiveRoll<<std::endl;
    std::cout<<"m_receivePitch:"<<m_receivePitch<<std::endl;
    std::cout<<"m_receiveYaw:"<<m_receiveYaw<<std::endl;

    //設彈速bullet Speed
    m_bulletSpeed = actual_bullet_speed;
    std::cout << "m_bulletSpeed:" << m_bulletSpeed << std::endl;
    
    //此處就直接調用矩陣解算來對"世界坐标系"2"相机坐标系"等轉換
    //TODO 此處前完成了所有特征點的坐标提取
    if (matrixCal() == false) {
        return false;
    }
    setFirstDetect();
    // if(++count == 10) {angleCal(); count = 0;}
    if (m_reload_big_buff != reload_big_buff) {//uint8_t shoot_flag;
        m_fitData.clear();
        m_reload_big_buff = !m_reload_big_buff;
    }
    angleCal();
    directionCal();
    if (mode_auto_request_) {
        m_buff_mode = estimated_buff_mode_;
    }
    std::cout<<"111111111111111111111"<<std::endl;
    if (m_direction == Direction::UNKNOWN) {
        return false;
    }
    if (predict() == false) { // 这里面判定大小符模式
        return false;
    }
    return true;
}

// /**
//  * @brief 预处理，更新时间戳并设置弹速
//  */
// //TODO 这里傳入了frame包括時間
// void BuffCalculator::preprocess(const Frame &frame, std::vector<cv::Point2f> &cameraPoints, const float &actual_bullet_speed) {
//     m_cameraPoints = cameraPoints;
//     m_frameTime = frame.m_time;
//     m_receiveRoll = frame.m_roll;
//     m_receivePitch = frame.m_pitch;
//     m_receiveYaw = frame.m_yaw;
//     std::cout<<"m_receiveRoll:"<<m_receiveRoll<<std::endl;
//     std::cout<<"m_receivePitch:"<<m_receivePitch<<std::endl;
//     std::cout<<"m_receiveYaw:"<<m_receiveYaw<<std::endl;

//     //設彈速bullet Speed
//     m_bulletSpeed = actual_bullet_speed;
//     std::cout << "m_bulletSpeed:" << m_bulletSpeed << std::endl;
//     m_direction{Direction::UNKNOWN};
//     m_convexity{Convexity::UNKNOWN};
//     m_totalShift{0};
//     m_firstDetect{true};
//     m_angleLast{0};
// }

/**
 * @brief
 * 矩阵解算，分别进行世界坐标系到相机坐标系，相机坐标系到云台坐标系，云台坐标系到机器人坐标系的转换，与旋转矩阵的设置
 */
bool BuffCalculator::matrixCal() {
//TODO 513
    // 进行坐标变换并设置旋转矩阵
    m_matW2C = world2Camera(m_worldPoints, m_cameraPoints, cameraMatrix, distCoeffs);
    std::cout<<"設定的m_worldPoints:"<<std::endl;
    for(auto& loca : m_worldPoints){std::cout<<loca<<std::endl;}
    cv::Mat tvec_c2g = cv::Mat(tvec_c2g_data).reshape(1, tvec_c2g_data.size());
    m_matC2G =
        camera2Gimbal({0.0, 0.0, 0.0}, tvec_c2g);
    // std::cout<<"CAMERA_TO_GIMBAL_TRANSLATION_VECTOR:"<<std::endl;
    
    m_matG2R =
        gimbal2Robot(angle2Radian(m_receivePitch), angle2Radian(m_receiveYaw), angle2Radian(m_receiveRoll));
    m_matW2R = m_matG2R * m_matC2G * m_matW2C;
    m_rMatW2R = m_matW2R(cv::Rect(0, 0, 3, 3));
    m_distance2Target = cv::norm(m_matW2C.col(3)) * 1e-3;
    // std::cout<<"org_distance2Target:"<<org_distance2Target<<"m"<<std::endl;
    // m_distance2Target = fitDistance(org_distance2Target);
    std::cout<<"m_distance2Target:"<<m_distance2Target<<"m"<<std::endl;
    // std::cout<<"dt_distan:"<<m_distance2Target-org_distance2Target<<"m"<<std::endl;

    // if (cv::inRange<double>(m_distance2Target, m_param["min_distance_to_buff"].Double(), m_param["max_distance_to_buff"].Double()) == false) {
    //     std::cout << "out of the range of the distance to buff" << std::endl;
    //     return false;
    // }
    // 记录装甲板和中心 R 的机器人坐标
    //TODO
    // cv::Mat armorWorld = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
    // cv::Mat centerWorld = (cv::Mat_<double>(4, 1) << 0, m_param["power_radius"].Double(), 0, 1);
    cv::Mat armorRobot{m_matW2R * armorWorld};
    cv::Mat centerCamera{m_matW2C * centerWorld};
    cv::Mat centerRobot{m_matW2R * centerWorld};
    m_armorRobot = {(float)(armorRobot.at<double>(0, 0)), (float)(armorRobot.at<double>(1, 0)),
                    (float)(armorRobot.at<double>(2, 0))};
    //R标中心坐标
    m_centerRobot = {(float)(centerRobot.at<double>(0, 0)), (float)(centerRobot.at<double>(1, 0)),
                     (float)(centerRobot.at<double>(2, 0))};
#if CONSOLE_OUTPUT >= 2
    MUTEX.lock();
    std::cout << "armor center coordinate: " << m_armorRobot << std::endl;
    std::cout << "center R coordinate: " << m_centerRobot << std::endl;
    //这里是解算完之后的裝甲板中心、R标坐标(Robot coordinate)
    MUTEX.unlock();
#endif
    return true;
}

/**
 * @brief 设置第一次检测的旋转矩阵和时间戳，便于进行角度解算
 */
void BuffCalculator::setFirstDetect() {
    if (m_firstDetect) {
        m_firstDetect = false;
        m_rMatW2RBase = m_rMatW2R.clone();
        m_startTime = m_frameTime;
    }
}

/**
 * @brief 角度解算
 */
void BuffCalculator::angleCal() {
    // 计算相对于第一次检测旋转的角度 angelAbs
    cv::Mat rMatRel{m_rMatW2RBase.inv() * m_rMatW2R};
    double angleAbs{-std::atan2(rMatRel.at<double>(0, 1), rMatRel.at<double>(0, 0))};
    std::cout<<"angleAbs: "<<angleAbs<<std::endl;
    // 减去上一次得到的角度，得到相对于上一次检测旋转的角度
    double angleMinus{angleAbs - m_angleLast};
    m_angleLast = angleAbs;
    // 用这个角度除以两片扇叶的夹角，得到装甲板切换数，并计算总的装甲板切换数
    int shift = std::round(angleMinus / ANGLE_BETWEEN_FAN_BLADES);
    is_shift = shift != 0;
    m_totalShift += shift;

    //xty::
    // 用 angelAbs 减去装甲板切换的角度，即可得到连续的角度
    m_angleRel = angleAbs - m_totalShift * ANGLE_BETWEEN_FAN_BLADES;
    std::cout<<"m_angleRel: "<<m_angleRel<<", "<<"m_totalShift: "<<m_totalShift<<std::endl;
    double time{std::chrono::duration_cast<std::chrono::microseconds>(m_frameTime - m_startTime).count() /
                1e6};
    updateAutoModeEstimate(time, m_angleRel);
    // 存储相对于第一次识别的时间间隔和角度的绝对值，日后进行拟合
    if (m_buff_mode == 2 || mode_auto_request_) {
        std::unique_lock lock(m_mutex);
        m_fitData.emplace_back(time, std::abs(m_angleRel));
    }
}

void BuffCalculator::updateAutoModeEstimate(double time_sec, double angle_rel) {
    if (!auto_mode_enable_) {
        return;
    }
    mode_history_.emplace_back(time_sec, angle_rel);
    while (!mode_history_.empty() &&
           (time_sec - mode_history_.front().first) > auto_mode_window_sec_) {
        mode_history_.pop_front();
    }
    if (mode_history_.size() < static_cast<size_t>(auto_mode_min_samples_)) {
        return;
    }

    std::vector<double> omegas;
    omegas.reserve(mode_history_.size() - 1);
    for (size_t i = 1; i < mode_history_.size(); ++i) {
        const double dt = mode_history_[i].first - mode_history_[i - 1].first;
        if (dt < 1e-3) {
            continue;
        }
        const double dtheta = mode_history_[i].second - mode_history_[i - 1].second;
        omegas.push_back(dtheta / dt);
    }
    if (omegas.size() < static_cast<size_t>(std::max(4, auto_mode_min_samples_ / 2))) {
        return;
    }

    double omega_sum = 0.0;
    for (const auto omega : omegas) {
        omega_sum += omega;
    }
    const double omega_mean = omega_sum / static_cast<double>(omegas.size());
    if (std::abs(omega_mean) < auto_mode_min_abs_omega_) {
        return;
    }

    double var_sum = 0.0;
    for (const auto omega : omegas) {
        const double delta = omega - omega_mean;
        var_sum += delta * delta;
    }
    const double omega_std = std::sqrt(var_sum / static_cast<double>(omegas.size()));

    if (estimated_buff_mode_ == 1) {
        if (omega_std > auto_mode_std_high_) {
            estimated_buff_mode_ = 2;
            std::cout << "[BUFF][AUTO_MODE] switch to BIG, omega_std=" << omega_std << std::endl;
        }
    } else {
        if (omega_std < auto_mode_std_low_) {
            estimated_buff_mode_ = 1;
            std::cout << "[BUFF][AUTO_MODE] switch to SMALL, omega_std=" << omega_std << std::endl;
        }
    }
}

/**
 * @brief 旋转方向解算
 */
void BuffCalculator::directionCal() {
    std::cout<<"222222222"<<std::endl;
    if (m_direction == Direction::UNKNOWN || m_direction == Direction::STABLE) {
    std::cout<<"222222222"<<std::endl;
        m_directionData.push_back(m_angleRel);
        if ((int)m_directionData.size() >= m_directionThresh) {
            // 计算角度差并投票
            int stable = 0, anti = 0, clockwise = 0;
            for (size_t i = 0; i < m_directionData.size() / 2; ++i) {
                auto temp{m_directionData.at(i + m_directionData.size() / 2) - m_directionData.at(i)};
                std::cout<<temp<<std::endl;
                if (temp > +1.5e-2) {
                    clockwise++;
                } else if (temp < -1.5e-2) {
                    anti++;
                } else {
                    stable++;
                }
            }
            // 得票数最多的为对应旋转方向
            std::cout<<"stable, clockwise, anti"<<stable<<", "<<clockwise<<", "<<anti<<std::endl;
            if (int temp{std::max({stable, clockwise, anti})}; std::max({stable, clockwise, anti}) == 0) {
                temp = stable;
                m_direction = Direction::STABLE;
                std::cout<<"all 0 set stable"<<std::endl;
            } else if (temp == clockwise) {
                m_direction = Direction::CLOCKWISE;
                std::cout<<"投票得clockwise"<<std::endl;
            } else if (temp == anti) {
                m_direction = Direction::ANTI_CLOCKWISE;
                std::cout<<"投票得anti clockwise"<<std::endl;
            } else {
                m_direction = Direction::STABLE;
                std::cout<<"投票得STABLE"<<std::endl;
            }
        }
    }
#if CONSOLE_OUTPUT >= 2
    MUTEX.lock();
    std::cout << "direction: "
              << (m_direction == Direction::CLOCKWISE        ? "clockwise"
                  : m_direction == Direction::ANTI_CLOCKWISE ? "anti-clockwise"
                  : m_direction == Direction::STABLE         ? "stable"
                                                             : "unknown")
              << std::endl;
    MUTEX.unlock();
#endif
    if(force_stable){
        m_direction = Direction::STABLE; //TODO
    }
}

/**
 * @brief 预测
 */
//TODO對下一出現位置进行預判
//TODO 520 調預測點: 可通過改angle解算中最終值, 得出了現距離解算系近了的, 導致predict的彈道快左, 改!!
bool BuffCalculator::predict() {
    // bool debug_stable = false;
    std::cout<<"进入BuffCalculator::predict"<<std::endl;
    double angle;
    if (m_direction == Direction::STABLE) {
        angle = 0.0;
    } else {
        if (m_buff_mode == 2) { //先測大符模式
            std::cout<<"9999999999999"<<std::endl;
            auto frameTime{
                std::chrono::duration_cast<std::chrono::milliseconds>(m_frameTime - m_startTime).count()};
            if (VALID_PARAMS.load() == false) {
                if (mode_auto_request_) {
                    angle = getRotationAngleSmall(m_distance2Target, m_bulletSpeed,
                                                  small_power_rune_rotation_speed, COMPANSATE_TIME);
                    m_buff_mode = 1;
                } else {
                    std::cout<<"valid_params.load() false"<<std::endl;
                    return false;
                }
            } else {
                angle = getRotationAngleBig(m_distance2Target, m_bulletSpeed, m_params, COMPANSATE_TIME +10,
                                            frameTime);
            }
        } else {
            angle = getRotationAngleSmall(m_distance2Target, m_bulletSpeed,
                                          small_power_rune_rotation_speed, COMPANSATE_TIME);
        }
        if (m_direction == Direction::ANTI_CLOCKWISE) {
            angle = -angle;
        }
    }
    m_predictRotationAngle = angle;
    cv::Mat matrixWorld = (cv::Mat_<double>(4, 1) << power_radius * std::sin(angle),
                           power_radius - power_radius * std::cos(angle), 0.0, 1.0);
    cv::Mat matrixRobot{m_matW2R * matrixWorld};
    m_predictRobot = {(float)(matrixRobot.at<double>(0, 0)), (float)(matrixRobot.at<double>(1, 0)),
                      (float)(matrixRobot.at<double>(2, 0))};
    auto [predictPitch, predictYaw] = getPitchYawFromRobotCoor(m_predictRobot, m_bulletSpeed);
    if (buff_shoot_table_adjust_enable && (!buff_shoot_table_big_only || m_buff_mode == 2)) {
        const double distance_m = getPredictDistanceM();
        const double height_m = getPredictHeightM();
        predictPitch += fitBuffPeriodicDeg(buff_pitch_adjust_param, m_predictRotationAngle, distance_m, height_m);
        predictYaw += fitBuffPeriodicDeg(buff_yaw_adjust_param, m_predictRotationAngle, distance_m, height_m);
    }
    m_predictPitch = predictPitch;
    m_predictYaw = predictYaw;
    m_predictPixel = getPixelFromRobot(m_predictRobot, m_matW2C, m_matW2R);
#if CONSOLE_OUTPUT >= 2
    MUTEX.lock();
    std::cout << "predict angle: " << angle << std::endl;
    std::cout << "predict coordinate: " << m_predictRobot << std::endl;
    std::cout << "predict pitch and yaw: " << m_predictPitch << ", " << m_predictYaw << std::endl;
    MUTEX.unlock();
#endif
    return true;
}

/**
 * @brief 拟合一次
 */
bool BuffCalculator::fitOnce() {
    // 如果数据量过少，则确定凹凸性
    if (m_fitData.size() < (size_t)2 * MIN_FIT_DATA_SIZE) {
        m_convexity = getConvexity(m_fitData);
    }
    // 利用 ransac 算法计算参数
    m_params = ransacFitting(m_fitData, m_convexity);
    return true;
}

/**
 * @brief 拟合线程的主函数
 */
void BuffCalculator::fit() {
    // if (Param::MODE != Mode::BIG) {//TODO
    //     return;
    // }

    decltype(m_fitData) fitData;
    while (STOP_THREAD.load() == false) {
        {
            std::shared_lock lock(m_mutex);
            fitData = m_fitData;
        }
        // 数据量过少时，直接返回
        if (m_fitData.size() < (size_t)MIN_FIT_DATA_SIZE) {
            continue;
        }
        bool result = fitOnce();
        VALID_PARAMS.store(result);
#if CONSOLE_OUTPUT >= 1
        MUTEX.lock();
        if (result == true) {
            std::cout << "params: A  ω  ϕ  B  C\n";
            std::for_each(m_params.begin(), m_params.end(), [](auto &&it) { std::cout << it << " "; });
            std::cout << std::endl;
        }
        MUTEX.unlock();
#endif
        if (m_fitData.size() > (size_t)MAX_FIT_DATA_SIZE) {
            m_fitData.erase(m_fitData.begin(), m_fitData.begin() + m_fitData.size() / 2);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1e4 / FPS)));
    }
}


cv::Point2f BuffCalculator::getPixelFromCamera(const cv::Mat &intrinsicMatrix, const cv::Mat &cameraPoint) {
    double fx = intrinsicMatrix.at<double>(0, 0);
    double fy = intrinsicMatrix.at<double>(1, 1);
    double cx = intrinsicMatrix.at<double>(0, 2);
    double cy = intrinsicMatrix.at<double>(1, 2);
    double X = cameraPoint.at<double>(0, 0);
    double Y = cameraPoint.at<double>(1, 0);
    double Z = cameraPoint.at<double>(2, 0);
    double u = (fx * X + cx * Z) / Z;
    double v = (fy * Y + cy * Z) / Z;
    return cv::Point2f(u, v);
}

cv::Point2f BuffCalculator::getPixelFromRobot(const cv::Point3f &robot, const cv::Mat &w2c, const cv::Mat &w2r) {
    cv::Mat matrixRobotPoint = (cv::Mat_<double>(4, 1) << robot.x, robot.y, robot.z, 1.0);
    cv::Mat matrixCameraPoint{w2c * (w2r.inv() * matrixRobotPoint)};
    return getPixelFromCamera(cameraMatrix, matrixCameraPoint); //Param:: INTRINSIC_MATRIX
}

std::pair<double, double> BuffCalculator::getPitchYawFromRobotCoor(const cv::Point3f &target, double bulletSpeed) {
    std::cout<<"target"<<target.x<< " "<<target.y<<" "<<target.z
<<std::endl;
    double horizontal{pointPointDistance({0.0, 0.0}, {target.x, target.z}) * 1e-3};
    double a{-0.5 * GRAVITY * std::pow(horizontal, 2) / std::pow(bulletSpeed, 2)};
    double b{horizontal};
    double c{a + target.y * 1e-3};
    double result{solveQuadraticEquation(a, b, c).second};
    //TODO 先看看相机外參 xyz 正負
    //TODO 先做正對靜止靶調pitch yaw
    //TODO 隨后測不同距離下COMPANSATE有效與無 做excel表 無效就做線性增量
    //TODO 
    double pitch{radian2Angle(std::atan(result)) + AFTER_PITCH}; //+ COMPANSATE_PITCH//弧度轉角度 //todo 看COMPANSATE_pitch/yaw能否解決不同距離, 不能就做線性
    double yaw{radian2Angle(-std::atan2(target.x, target.z)) + AFTER_YAW}; //+ COMPANSATE_YAW
    const double height_m = target.y * 1e-3;
    if (static_shoot_table_adjust_enable) {
        pitch += fitStaticShootTableDeg(static_pitch_adjust_param, height_m, horizontal);
        yaw += fitStaticShootTableDeg(static_yaw_adjust_param, height_m, horizontal);
    }
    std::cout<<"after_pitch:"<<pitch<<std::endl;
    std::cout<<"after_yaw:"<<yaw<<std::endl;
    return std::make_pair(pitch, yaw);
}

// std::vector<double, double> BuffCalculator::getBuffResult(){
//     std::pair<double, double> result_pitch_yaw(m_predictPitch, m_predictYaw);
//     return this->result_pitch_yaw;
// }

/**
 * @brief 世界坐标系转相机坐标系
 * @param[in] worldPoints   世界坐标系坐标
 * @param[in] cameraPoints  相机坐标系坐标
 * @return cv::Mat
 */
cv::Mat world2Camera(const std::vector<cv::Point3f> &worldPoints,
                     const std::vector<cv::Point2f> &cameraPoints, const cv::Mat &intrinsicMatrix,
                     const cv::Mat &distCoeffs) {
    cv::Mat rVec, tVec, rMat;
    cv::solvePnP(worldPoints, cameraPoints, intrinsicMatrix, distCoeffs, rVec, tVec, false,
                 cv::SOLVEPNP_ITERATIVE);
    cv::Rodrigues(rVec, rMat);
    cv::Mat w2c{cv::Mat::zeros(cv::Size(4, 4), CV_64FC1)};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            w2c.at<double>(i, j) = rMat.at<double>(i, j);
        }
    }
    for (int i = 0; i < 3; ++i) {
        w2c.at<double>(i, 3) = tVec.at<double>(i, 0);
    }
    w2c.at<double>(3, 3) = 1.0;
    return w2c;
}

/**
 * @brief 相机坐标系转云台坐标系
 * @param[in] r             旋转参数
 * @param[in] t             平移参数
 * @return cv::Mat
 */
cv::Mat camera2Gimbal(const std::array<double, 3> &r, const std::array<double, 3> &t) {
    cv::Mat rVec = (cv::Mat_<double>(3, 1) << r[0], r[1], r[2]);
    cv::Mat tVec = (cv::Mat_<double>(3, 1) << t[0], t[1], t[2]);
    cv::Mat rMat;
    cv::Rodrigues(rVec, rMat);
    cv::Mat c2g{cv::Mat::zeros(cv::Size(4, 4), CV_64FC1)};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            c2g.at<double>(i, j) = rMat.at<double>(i, j);
        }
    }
    for (int i = 0; i < 3; ++i) {
        c2g.at<double>(i, 3) = tVec.at<double>(i, 0);
    }
    c2g.at<double>(3, 3) = 1;
    return c2g;
}

/**
 * @brief 云台坐标系转机器人坐标系
 * @param[in] pitch
 * @param[in] yaw
 * @return cv::Mat
 */
cv::Mat gimbal2Robot(double pitch, double yaw, double roll) {
    cv::Mat matY = (cv::Mat_<double>(4, 4) << std::cos(-yaw), 0, std::sin(-yaw), 0, 0, 1, 0, 0,
                    -std::sin(-yaw), 0, std::cos(-yaw), 0, 0, 0, 0, 1);
    cv::Mat matX = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, std::cos(pitch), -std::sin(pitch), 0, 0,
                    std::sin(pitch), std::cos(pitch), 0, 0, 0, 0, 1);
    cv::Mat matZ = (cv::Mat_<double>(4, 4) << std::cos(roll), -std::sin(roll), 0, 0, std::sin(roll),
                    std::cos(roll), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    // 实际测试中，roll 不起作用，这可能和电控对欧拉角的解算有关
    return matY * matX;
}

/**
 * @brief 凹凸性计算
 * @param[in] data          角度数据
 * @return Convexity
 */
Convexity getConvexity(const std::vector<std::pair<double, double>> &data) {
    auto first{data.begin()}, last{data.end() - 1};
    double slope{(last->second - first->second) / (last->first - first->first)};
    double offset{(first->second * last->first - last->second * first->first) / (last->first - first->first)};
    int concave{0}, convex{0};
    for (const auto &i : data) {
        if (slope * i.first + offset > i.second) {
            concave++;
        } else {
            convex++;
        }
    }
    const int standard{static_cast<int>(data.size() * 0.75)};
    return concave > standard  ? Convexity::CONCAVE
           : convex > standard ? Convexity::CONVEX
                               : Convexity::UNKNOWN;
}

/**
 * @brief ransac 算法，返回拟合参数
 * @param[in] data          角度数据
 * @param[in] convexity     凹凸性
 * @return std::array<double, 5>
 */
std::array<double, 5> ransacFitting(const std::vector<std::pair<double, double>> &data, Convexity convexity) {
    // inliers 为符合要求的点，outliers 为不符合要求的点
    std::vector<std::pair<double, double>> inliers, outliers;
    // 初始时，inliers 为全部点
    inliers.assign(data.begin(), data.end());
    // 迭代次数
    int iterTimes{data.size() < 400 ? 200 : 20};
    // 初始参数


    //xty::
    //a/w应设为0.47？
    //std::array<double, 5> params{0.9125, 1.942, 0, 1.178, 0};
    std::array<double, 5> params{0.469, 1.942, 0, 1.177, 0};
    
    for (int i = 0; i < iterTimes; ++i) {
        decltype(inliers) sample;
        // 如果数据点较多，则将数据打乱，取其中一部分
        if (inliers.size() > 400) {
            std::shuffle(
                inliers.begin(), inliers.end() - 100,
                std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
            sample.assign(inliers.end() - 200, inliers.end());
        } else {
            sample.assign(inliers.begin(), inliers.end());
        }
        // 进行拟合
        params = leastSquareEstimate(sample, params, convexity);
        // 对 inliers 每一个计算误差
        std::vector<double> errors;
        for (const auto &inlier : inliers) {
            errors.push_back(std::abs(inlier.second - getAngleBig(inlier.first, params)));
        }
        // 如果数据量较大，则对点进行筛选
        if (data.size() > 800) {
            std::sort(errors.begin(), errors.end());
            const int index{static_cast<int>(errors.size() * 0.95)};
            const double threshold{errors[index]};
            // 剔除 inliers 中不符合要求的点
            for (size_t i = 0; i < inliers.size() - 100; ++i) {
                if (std::abs(inliers[i].second - getAngleBig(inliers[i].first, params)) > threshold) {
                    outliers.push_back(inliers[i]);
                    inliers.erase(inliers.begin() + i);
                }
            }
            // 将 outliers 中符合要求的点加进来
            for (size_t i = 0; i < outliers.size(); ++i) {
                if (std::abs(outliers[i].second - getAngleBig(outliers[i].first, params)) < threshold) {
                    inliers.emplace(inliers.begin(), outliers[i]);
                    outliers.erase(outliers.begin() + i);
                }
            }
        }
    }
    // 返回之前对所有 inliers 再拟合一次
    return params;
}

/**
 * @brief 最小二乘拟合，返回参数列表
 * @param[in] points        数据点
 * @param[in] params        初始参数
 * @param[in] convexity     凹凸性
 * @return std::array<double, 5>
 */
std::array<double, 5> leastSquareEstimate(const std::vector<std::pair<double, double>> &points,
                                          const std::array<double, 5> &params, Convexity convexity) {
    std::array<double, 5> ret = params;
    ceres::Problem problem;
    for (size_t i = 0; i < points.size(); i++) {
        ceres::CostFunction *costFunction = new CostFunctor2(points[i].first, points[i].second);
        ceres::LossFunction *lossFunction = new ceres::SoftLOneLoss(0.1);
        problem.AddResidualBlock(costFunction, lossFunction, ret.begin());
    }
    std::array<double, 3> omega;
    if (points.size() < 100) {
        // 在数据量较小时，可以利用凹凸性定参数边界
        if (convexity == Convexity::CONCAVE) {
            problem.SetParameterUpperBound(ret.begin(), 2, -2.8);
            problem.SetParameterLowerBound(ret.begin(), 2, -4);
        } else if (convexity == Convexity::CONVEX) {
            problem.SetParameterUpperBound(ret.begin(), 2, -1.1);
            problem.SetParameterLowerBound(ret.begin(), 2, -2.3);
        }
        omega = {10., 1., 1.};
    } else {
        // 而数据量较多后，则不再需要凹凸性辅助拟合
        omega = {60., 50., 50.};
    }
    ceres::CostFunction *costFunction1 = new CostFunctor1(ret[0], 0);
    ceres::LossFunction *lossFunction1 =
        new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[0], ceres::TAKE_OWNERSHIP);
    problem.AddResidualBlock(costFunction1, lossFunction1, ret.begin());
    ceres::CostFunction *costFunction2 = new CostFunctor1(ret[1], 1);
    ceres::LossFunction *lossFunction2 =
        new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[1], ceres::TAKE_OWNERSHIP);
    problem.AddResidualBlock(costFunction2, lossFunction2, ret.begin());
    ceres::CostFunction *costFunction3 = new CostFunctor1(ret[3], 3);
    ceres::LossFunction *lossFunction3 =
        new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[2], ceres::TAKE_OWNERSHIP);
    problem.AddResidualBlock(costFunction3, lossFunction3, ret.begin());

    //xty::
    // 参数边界（使用 a_code = a/ω 语义）
    problem.SetParameterLowerBound(ret.begin(), 0, 0.39);   // a/ω 下界
    problem.SetParameterUpperBound(ret.begin(), 0, 0.555);  // a/ω 上界
    problem.SetParameterLowerBound(ret.begin(), 1, 1.884);  // ω 下界
    problem.SetParameterUpperBound(ret.begin(), 1, 2.000);  // ω 上界
    problem.SetParameterLowerBound(ret.begin(), 3, 1.045);  // b 下界
    problem.SetParameterUpperBound(ret.begin(), 3, 1.31);   // b 上界
    

    //xty::
    // 增加物理约束残差：A * ω + b = 2.09，其中 A 表示 a/ω
    ceres::CostFunction* physCost = new CostFunctorPhys();
    ceres::LossFunction* physLoss = new ceres::ScaledLoss(new ceres::HuberLoss(0.1), 1000.0, ceres::TAKE_OWNERSHIP);
    problem.AddResidualBlock(physCost, physLoss, ret.begin());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 50;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    return ret;
}


// std::pair<double, double> BuffCalculator::getPitchYawFromRobotCoor(const cv::Point3f &target, double bulletSpeed) {
//     double horizontal{pointPointDistance({0.0, 0.0}, {target.x, target.z}) * 1e-3};
//     double a{-0.5 * GRAVITY * std::pow(horizontal, 2) / std::pow(bulletSpeed, 2)};
//     double b{horizontal};
//     double c{a + target.y * 1e-3};
//     double result{solveQuadraticEquation(a, b, c).second};
//     double pitch{radian2Angle(std::atan(result)) + COMPANSATE_PITCH};
//     double yaw{radian2Angle(-std::atan2(target.x, target.z)) + COMPANSATE_YAW};
//     return std::make_pair(pitch, yaw);
// }

// cv::Point2f BuffCalculator::getPixelFromCamera(const cv::Mat &intrinsicMatrix, const cv::Mat &cameraPoint) {
//     double fx = intrinsicMatrix.at<double>(0, 0);
//     double fy = intrinsicMatrix.at<double>(1, 1);
//     double cx = intrinsicMatrix.at<double>(0, 2);
//     double cy = intrinsicMatrix.at<double>(1, 2);
//     double X = cameraPoint.at<double>(0, 0);
//     double Y = cameraPoint.at<double>(1, 0);
//     double Z = cameraPoint.at<double>(2, 0);
//     double u = (fx * X + cx * Z) / Z;
//     double v = (fy * Y + cy * Z) / Z;
//     return cv::Point2f(u, v);
// }

// cv::Point2f BuffCalculator::getPixelFromRobot(const cv::Point3f &robot, const cv::Mat &w2c, const cv::Mat &w2r) {
//     cv::Mat matrixRobotPoint = (cv::Mat_<double>(4, 1) << robot.x, robot.y, robot.z, 1.0);
//     cv::Mat matrixCameraPoint{w2c * (w2r.inv() * matrixRobotPoint)};
//     return getPixelFromCamera(cameraMatrix, matrixCameraPoint); //Param:: INTRINSIC_MATRIX
// }

}  // namespace power_rune
