// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "predictor/OutpostPredictor.hpp"
#include "Logger/Logger.hpp"

using namespace LY_UTILS;


namespace PREDICTOR
{
    OutpostPredictor::OutpostPredictor()
    {
        this->is_initialized = false;
        this->outpost_estimator = std::make_unique<OutpostEstimator>();
    }

    void OutpostPredictor::initPredictor(SOLVER::ArmorPose &armor_pose, int derection)
    {
        roslog::info("PREDICTOR INIT");
        this->is_initialized = true;
        this->outpost_estimator = std::make_unique<OutpostEstimator>();
        this->outpost_estimator->initEstimator(armor_pose, derection);
        if(derection == -1) roslog::info("Clockwise");
        else if(derection == 1) roslog::info("CounterClockwise");
        
    }

    void OutpostPredictor::resetPredictor()
    {
        roslog::info("PREDICTOR RESET");
        this->is_initialized = false;
    }


    OutpostInformation OutpostPredictor::runPredictor(int &delta_time, SOLVER::ArmorPose &armor_pose, bool use_measure_update)
    {
#define PRIORI true
#define POSTERIORI false
        this->delta_t = delta_time / 1000.0;
        this->delta_t = 0.015;
        roslog::info("Delat_t:{}",this->delta_t);
        if (use_measure_update) // 量测更新
        {
            this->outpost_estimator->setDeltaTime(this->delta_t);
            bool check = this->outpost_estimator->inputNewMeasurement(armor_pose,this->delta_t);
            auto outpost_information = this->outpost_estimator->getInformation(PRIORI);
            roslog::info("MEASUREMENT UPDATING...");

            if(!check){
                roslog::info("Reset Predictor");
                this->resetPredictor();
                outpost_information.is_valid = false;
            }
            return outpost_information;
        }
        else // 迭代更新
        {
            this->outpost_estimator->setDeltaTime(this->delta_t);
            this->outpost_estimator->noMeasurementUpdate();
            auto outpost_information = this->outpost_estimator->getInformation(POSTERIORI);
            roslog::info("ITERATION UPDATING...");
            return outpost_information;
        }
    }

    OutpostEstimator::OutpostEstimator()
    {
        this->derection = 0; // 未判断方向
        this->outpost_state_filter = std::make_unique<ExtendedKalman<double, 8, 4>>();
        // measure : (yaw_a,pitch_a,dis_a,theta_a,prefect_omega)
        Eigen::Matrix<bool, 4, 1> radianMeasure;
        radianMeasure << true, true, false, true;
        this->outpost_state_filter->setRadianMeasure(radianMeasure); // 设置测量维度为弧度制
        this->last_armor_pose = SOLVER::ArmorPose();// 初始化上一次装甲板信息
    }

    void OutpostEstimator::initEstimator(SOLVER::ArmorPose &armor_pose, int derection)
    {
        double distance = armor_pose.pyd.distance;
        double yaw = armor_pose.pyd.yaw;
        double pitch = armor_pose.pyd.pitch;

        // distance yaw pitch 转换成 x y z
        double x = distance * cos(pitch) * sin(-yaw);
        double y = distance * cos(pitch) * cos(yaw);
        double z = distance * sin(pitch);
        roslog::info("x>>>>:{} y>>>>:{} z>>>>:{}", x, y, z);
        double x_c = x - OUTPOST_RADIUS * sin(armor_pose.theta_world);
        double y_c = y + OUTPOST_RADIUS * cos(armor_pose.theta_world);
        double z_c = z;
        roslog::info("x_c>>>>:{} y_c>>>>:{} z_c>>>>:{}", x_c, y_c, z_c);
        this->derection = derection;                                                                                   // 设置方向
        this->outpost_state_filter->X_k << x_c, 0, y_c, 0, z_c, 0, armor_pose.theta_world,this->derection * OUTPOST_OMEGA; // 初始状态向量
        this->outpost_state_filter->P = Eigen::Matrix<double, 8, 8>::Identity() * 0.1;                                 // 初始协方差矩阵

        last_armor_pose = armor_pose; // 保存上一次装甲板信息
        roslog::info("OUTPOST ESTIMATOR INIT");
    }

    bool OutpostEstimator::inputNewMeasurement(SOLVER::ArmorPose &armor_pose,double dt)
    {
        setMeasureNoise(armor_pose.pyd.distance); // 设置测量噪声
        setTransitionMatrix();                    // 设置状态转移矩阵
        setProcessNoise();                        // 设置过程噪声矩阵

        this->outpost_state_filter->predict(); 

        double x = armor_pose.pyd.distance * cos(armor_pose.pyd.pitch) * sin(-armor_pose.pyd.yaw);
        double y = armor_pose.pyd.distance * cos(armor_pose.pyd.pitch) * cos(armor_pose.pyd.yaw);
        double z = armor_pose.pyd.distance * sin(armor_pose.pyd.pitch);
        roslog::info("x>>>>:{} y>>>>:{} z>>>>:{}", x, y, z);
        double x_c = x - OUTPOST_RADIUS * sin(armor_pose.theta_world);
        double y_c = y + OUTPOST_RADIUS * cos(armor_pose.theta_world);
        double z_c = z;
        roslog::info("x_c>>>>:{} y_c>>>>:{} z_c>>>>:{}", x_c, y_c, z_c);

        Eigen::Matrix<double, 4, 1> measurement;
        measurement << 
            armor_pose.pyd.yaw, 
            armor_pose.pyd.pitch,
            armor_pose.pyd.distance,
            armor_pose.theta_world;

        Eigen::Matrix<double, 4, 1> innovation = measurement - this->outpost_state_filter->H * this->outpost_state_filter->X_j;
        // 打印innovation的norm
        roslog::info("Innovation norm: {}", innovation.norm());
        // 新息协方差矩阵 S = H*P*H^T + R
        Eigen::Matrix<double, 4, 4> S = this->outpost_state_filter->H * 
                                        this->outpost_state_filter->P * 
                                        this->outpost_state_filter->H.transpose() + 
                                        this->outpost_state_filter->R;
        // 执行卡方检验
        bool is_measurement_valid_ = ChiSquareTest(innovation, S);

        if(this->outpost_state_filter->X_j[1]==0||this->outpost_state_filter->X_j[3]==0)
        is_measurement_valid_ = true;

        if (true){
        this->outpost_state_filter->update(center_state_2_measure, measurement); // 更新
        }else{
            roslog::info("Measurement Invalid");
        }
        return true;
    }

    void OutpostEstimator::noMeasurementUpdate()
    {
        this->outpost_state_filter->predict(); // 预测
    }

    OutpostInformation OutpostEstimator::getInformation(bool is_measure_update)
    {
        Eigen::Matrix<double, 8, 1> state_vec;
        if (is_measure_update)
        {
            state_vec = this->outpost_state_filter->X_k; // 返回当前状态向量
        }
        else
        {
            state_vec = this->outpost_state_filter->X_j; // 返回预测状态向量
        }

        OutpostInformation outpost_information;
        outpost_information.center_position = Eigen::Vector3d(state_vec[0], state_vec[2], state_vec[4]); // 中心位置
        outpost_information.center_velocity = Eigen::Vector3d(state_vec[1], state_vec[3], state_vec[5]); // 中心速度
        outpost_information.outpost_radius = OUTPOST_RADIUS;                                             // 半径
        outpost_information.outpost_theta = state_vec[6];                                                // theta
        outpost_information.outpost_omega = state_vec[7];                                                // omega
        return outpost_information;
    }

    void OutpostEstimator::setMeasureNoise(float &distance)
    {
        // 设置测量噪声
#define YAW_MEASURE_NOISE 1
#define PITCH_MEASURE_NOISE 1
#define ARMOR_THETA_MEASURE_NOISE 1.5
        double distance_measure_noise;
        if (distance < 1.5)
        {
            distance_measure_noise = pow(distance * 0.1, 2);
        }
        else if (distance < 4.5)
        {
            distance_measure_noise = pow(0.12*distance, 2);
        }
        else
        {
            distance_measure_noise = pow(0.18*distance, 2);
        }
        this->outpost_state_filter->R << YAW_MEASURE_NOISE, 0, 0, 0, // 0:yaw
            0, PITCH_MEASURE_NOISE, 0, 0,                               // 1:pitch
            0, 0, distance_measure_noise, 0,                            // 2:distance
            0, 0, 0, ARMOR_THETA_MEASURE_NOISE;                        // 3:theta
    }

    void OutpostEstimator::setTransitionMatrix()
    {
        // 设置状态转移矩阵
        this->outpost_state_filter->F << 
            1, dt, 0, 0, 0, 0, 0, 0, // 0:x_c
            0, 1, 0, 0, 0, 0, 0, 0,  // 1:v_x
            0, 0, 1, dt, 0, 0, 0, 0, // 2:y_c
            0, 0, 0, 1, 0, 0, 0, 0,  // 3:v_y
            0, 0, 0, 0, 1, dt, 0, 0, // 4:z_c
            0, 0, 0, 0, 0, 1, 0, 0,  // 5:v_z
            0, 0, 0, 0, 0, 0, 1, dt, // 6:theta_c
            0, 0, 0, 0, 0, 0, 0, 1; // 7:omega
    }

    void OutpostEstimator::setProcessNoise()
    {
        // 设置过程噪声矩阵
        Eigen::Matrix<double, 2, 1> process_noice_vec;
        process_noice_vec << 0.5 * dt * dt, dt;

#define PROCESS_NOISE_X 0.1
#define PROCESS_NOISE_Y 0.1
#define PROCESS_NOISE_Z 0.1
#define PROCESS_NOISE_THETA 0.1

        Eigen::Matrix2d process_noise_matrix_x;
        process_noise_matrix_x = process_noice_vec * PROCESS_NOISE_X * process_noice_vec.transpose();

        Eigen::Matrix2d process_noise_matrix_y;
        process_noise_matrix_y = process_noice_vec * PROCESS_NOISE_Y * process_noice_vec.transpose();

        Eigen::Matrix2d process_noise_matrix_z;
        process_noise_matrix_z = process_noice_vec * PROCESS_NOISE_Z * process_noice_vec.transpose();

        Eigen::Matrix2d process_noise_matrix_theta;
        process_noise_matrix_theta = process_noice_vec * PROCESS_NOISE_THETA * process_noice_vec.transpose();

        this->outpost_state_filter->Q.block<2, 2>(0, 0) = process_noise_matrix_x;
        this->outpost_state_filter->Q.block<2, 2>(2, 2) = process_noise_matrix_y;
        this->outpost_state_filter->Q.block<2, 2>(4, 4) = process_noise_matrix_z;
        this->outpost_state_filter->Q.block<2, 2>(6, 6) = process_noise_matrix_theta;
    }

   bool OutpostEstimator::ChiSquareTest(
    const Eigen::Matrix<double, 4, 1>& innovation,
    const Eigen::Matrix<double, 4, 4>& innovation_cov
    ) {
        Eigen::Matrix<double, 4, 1> weighted_innovation = innovation_cov.llt().solve(innovation);
        chi_square_value_ = innovation.dot(weighted_innovation);

        bool is_valid = (chi_square_value_ <= chi_square_threshold_);
    
        roslog::info("Chi-square value: {} | Threshold: {}", chi_square_value_, chi_square_threshold_);
        return is_valid;
    }
}