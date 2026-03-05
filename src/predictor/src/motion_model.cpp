#include "predictor/motion_model.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ly_auto_aim::inline predictor{
    inline Eigen::Matrix3d build_Q_PVA(double q_jerk_base, double dt) {
        Eigen::Matrix3d Q_sub;
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;

        Q_sub << q_jerk_base * dt5 / 20.0, q_jerk_base * dt4 / 8.0,  q_jerk_base * dt3 / 6.0,
                q_jerk_base * dt4 / 8.0,  q_jerk_base * dt3 / 3.0,  q_jerk_base * dt2 / 2.0,
                q_jerk_base * dt3 / 6.0,  q_jerk_base * dt2 / 2.0,  q_jerk_base * dt;
        return Q_sub;
    }

// Helper to build a 2x2 Q sub-matrix for [angle, angular_velocity]
// based on continuous white noise in angular acceleration (spectral density q_ang_accel)
    inline Eigen::Matrix2d build_Q_AngleVel(double q_ang_accel_base, double dt) {
        Eigen::Matrix2d Q_sub;
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;

        Q_sub << q_ang_accel_base * dt3 / 3.0, q_ang_accel_base * dt2 / 2.0,
                q_ang_accel_base * dt2 / 2.0, q_ang_accel_base * dt;
        return Q_sub;
    }

void MotionModel::initMotionModel()
{
//    MatrixXX P = MatrixXX::Zero();
//    P << 0.1,  0.02, 0,    0,    0,    0,    0,    0,    0,    0,
//            0.02, 0.2,  0,    0,    0,    0,    0,    0,    0,    0,
//            0,    0,    0.1,  0.02, 0,    0,    0,    0,    0,    0,
//            0,    0,    0.02, 0.2,  0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    0.1,  0.02, 0,    0,    0,    0,
//            0,    0,    0,    0,    0.02, 0.2,  0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    0.02, 0.005,0,    0,
//            0,    0,    0,    0,    0,    0,    0.005,0.02, 0,    0,
//            0,    0,    0,    0,    0,    0,    0,    0,    0.1,  0,
//            0,    0,    0,    0,    0,    0,    0,    0,    0,    0.1;
//
//    MatrixXX Q = MatrixXX::Zero();
//    Q << 5e-4, 1e-4, 0,    0,    0,    0,    0,    0,    0,    0,
//            1e-4, 1e-2, 0,    0,    0,    0,    0,    0,    0,    0,
//            0,    0,    5e-4, 1e-4, 0,    0,    0,    0,    0,    0,
//            0,    0,    1e-4, 1e-2, 0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    1e-3, 2e-4, 0,    0,    0,    0,
//            0,    0,    0,    0,    2e-4, 5e-2, 0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    5e-6, 1e-8, 0,    0,
//            0,    0,    0,    0,    0,    0,    1e-8, 5e-6, 0,    0,
//            0,    0,    0,    0,    0,    0,    0,    0,    1e-5, 0,
//            0,    0,    0,    0,    0,    0,    0,    0,    0,    1e-5;
//    // 测量变量顺序: ax, ay, az, tangent, angle_left, angle_right
//    MatrixYY R = MatrixYY::Zero();
//    R << 0.03,  0.01,  0,     0,     0,     0,
//            0.01,  0.03,  0,     0,     0,     0,
//            0,     0,     0.03,  0,     0,     0,
//            0,     0,     0,     0.1,   0,     0,
//            0,     0,     0,     0,     0.0004,  0.00015,
//            0,     0,     0,     0,     0.00015, 0.0004;
//    //R *= 10;
//    R*=0.01;
//    Q*=10;
//    ekf.init(P, Q, R);
    // 状态变量顺序: x, vx, y, vy, theta, omega, r1, r2, z1, z2, ax, ay
    MatrixXX P = MatrixXX::Zero();
    P << 0.2,   0.08,  0,     0,     0,    0,     0,     0,     0,    0,   0.004, 0,
            0.08,  0.4,   0,     0,     0,    0,     0,     0,     0,    0,   0.02,  0,
            0,     0,     0.2,   0.08,  0,    0,     0,     0,     0,    0,   0,     0.004,
            0,     0,     0.08,  0.4,   0,    0,     0,     0,     0,    0,   0,     0.02,
            0,     0,     0,     0,     0.1,  0.02,  0,     0,     0,    0,   0,     0,
            0,     0,     0,     0,     0.02, 0.2,   0,     0,     0,    0,   0,     0,
            0,     0,     0,     0,     0,    0,     0.02,  0.005, 0,    0,   0,     0,
            0,     0,     0,     0,     0,    0,     0.005, 0.02,  0,    0,   0,     0,
            0,     0,     0,     0,     0,    0,     0,     0,     0.1,  0,   0,     0,
            0,     0,     0,     0,     0,    0,     0,     0,     0,    0.1, 0,     0,
            0.004, 0.02,  0,     0,     0,    0,     0,     0,     0,    0,   0.6,   0,
            0,     0,     0.004, 0.02,  0,    0,     0,     0,     0,    0,   0,     0.6;

    MatrixXX Q_val = MatrixXX::Zero();
//    Q << 5e-4, 1e-4, 0,    0,    0,    0,    0,    0,    0,    0,
//         1e-4, 1e-2, 0,    0,    0,    0,    0,    0,    0,    0,
//         0,    0,    5e-4, 1e-4, 0,    0,    0,    0,    0,    0,
//         0,    0,    1e-4, 1e-2, 0,    0,    0,    0,    0,    0,
//         0,    0,    0,    0,    1e-4, 2e-5, 0,    0,    0,    0,
//         0,    0,    0,    0,    2e-5, 5e-3, 0,    0,    0,    0,
//         0,    0,    0,    0,    0,    0,    5e-6, 1e-8, 0,    0,
//         0,    0,    0,    0,    0,    0,    1e-8, 5e-6, 0,    0,
//         0,    0,    0,    0,    0,    0,    0,    0,    1e-4, 0,
//         0,    0,    0,    0,    0,    0,    0,    0,    0,    1e-4;
    //      Ideal Q for one-dimension
    //      Q = σ_a_drift^2 * [[dt^4/20, dt^3/8, dt^2/6 ], [dt^3/8, dt^2/3, dt/2 ], [dt^2/6, dt/2, 1 ]]
    //      x,     vx,    y,     vy,    theta, omega, r1,    r2,    z1,    z2,    ax,    ay
//    const double sigma_x = 0.05; // 5cm
//    const double sigma_y = sigma_x;
//    double sx = sigma_x * sigma_x; // 0.0025
//    double sy = sigma_y * sigma_y; // 0.0025
//    const double est_dt = 0.015;
//
//    Q << 5e-4,  1e-4,  0,     0,     0,     0,     0,     0,     0,     0,     0,     0,      // x
//            1e-4,  1e-2,  0,     0,     0,     0,     0,     0,     0,     0,     2e-3,  0,      // vx
//            0,     0,     5e-4,  1e-4,  0,     0,     0,     0,     0,     0,     0,     0,      // y
//            0,     0,     1e-4,  1e-2,  0,     0,     0,     0,     0,     0,     0,     2e-3,   // vy
//            0,     0,     0,     0,     1e-3,  2e-4,  0,     0,     0,     0,     0,     0,      // theta
//            0,     0,     0,     0,     2e-4,  5e-2,  0,     0,     0,     0,     0,     0,      // omega
//            0,     0,     0,     0,     0,     0,     5e-6,  1e-8,  0,     0,     0,     0,      // r1
//            0,     0,     0,     0,     0,     0,     1e-8,  5e-6,  0,     0,     0,     0,      // r2
//            0,     0,     0,     0,     0,     0,     0,     0,     1e-5,  0,     0,     0,      // z1
//            0,     0,     0,     0,     0,     0,     0,     0,     0,     1e-5,  0,     0,      // z2
//            0,     2e-3,  0,     0,     0,     0,     0,     0,     0,     0,     2e-1,  0,      // ax
//            0,     0,     0,     2e-3,  0,     0,     0,     0,     0,     0,     0,     2e-1;   // ay
    double q_ax_jerk_base_ = 1.5;     // m^2/s^5
    double q_ay_jerk_base_ = 1.5;     // m^2/s^5
    double q_omega_accel_base_ = 0.4; // rad^2/s^3
    double q_r1_drift_base_ = 1e-5;   // m^2/s
    double q_r2_drift_base_ = 1e-5;   // m^2/s
    double q_z1_drift_base_ = 1e-4;   // m^2/s
    double q_z2_drift_base_ = 1e-4;   // m^2/s
    double dt = 0.015; // time step in seconds
    // X, VX, AX (indices 0, 1, 10)
    Eigen::Matrix3d Q_xva = build_Q_PVA(q_ax_jerk_base_, dt);
    Q_val(0,0)   = Q_xva(0,0); Q_val(0,1)   = Q_xva(0,1); Q_val(0,10)  = Q_xva(0,2);
    Q_val(1,0)   = Q_xva(1,0); Q_val(1,1)   = Q_xva(1,1); Q_val(1,10)  = Q_xva(1,2);
    Q_val(10,0)  = Q_xva(2,0); Q_val(10,1)  = Q_xva(2,1); Q_val(10,10) = Q_xva(2,2);

    // Y, VY, AY (indices 2, 3, 11)
    Eigen::Matrix3d Q_yva = build_Q_PVA(q_ay_jerk_base_, dt);
    Q_val(2,2)   = Q_yva(0,0); Q_val(2,3)   = Q_yva(0,1); Q_val(2,11)  = Q_yva(0,2);
    Q_val(3,2)   = Q_yva(1,0); Q_val(3,3)   = Q_yva(1,1); Q_val(3,11)  = Q_yva(1,2);
    Q_val(11,2)  = Q_yva(2,0); Q_val(11,3)  = Q_yva(2,1); Q_val(11,11) = Q_yva(2,2);

    // THETA, OMEGA (indices 4, 5)
    Eigen::Matrix2d Q_to = build_Q_AngleVel(q_omega_accel_base_, dt);
    Q_val(4,4) = Q_to(0,0); Q_val(4,5) = Q_to(0,1);
    Q_val(5,4) = Q_to(1,0); Q_val(5,5) = Q_to(1,1);

    // Parameters: r1, r2, z1, z2 (indices 6, 7, 8, 9)
    Q_val(6,6) = q_r1_drift_base_ * dt;
    Q_val(7,7) = q_r2_drift_base_ * dt;
    Q_val(8,8) = q_z1_drift_base_ * dt;
    Q_val(9,9) = q_z2_drift_base_ * dt;

    // 测量变量顺序: ax, ay, az, tangent, angle_left, angle_right
    // p y d angle_left, angle_right center_angle
    MatrixYY R = MatrixYY::Zero();
    R << 0.005,  0,     0,     0,     0,     0, 0,
         0.,    0.005,  0,     0,     0,     0, 0,
         0,     0,     0.1,   0,     0,     0, 0,
         0,     0,     0,     0.04,   0,     0, 0,
         0,     0,     0,     0,     0.04,  0.015,0,
         0,     0,     0,     0,     0.015, 0.04,0,
         0,     0,     0,     0,     0,     0,   0.002;
    //R *= 10;
    R *= 0.1;
    ekf.init(P, Q_val, R);
}

VectorX MotionModel::getPredictResult(const Time::TimeStamp& timestamp)
{
    // roslog::info("predictResult: {}", ekf.predict(timestamp)[0]);
    // roslog::info("predictResult: {}", ekf.predict(timestamp)[1]);
    // roslog::info("predictResult: {}", ekf.predict(timestamp)[2]);
    // roslog::info("predictResult: {}", ekf.predict(timestamp)[3]);
    return ekf.predict(timestamp);
}

void MotionModel::Update(const VectorY& measure_vec, const Time::TimeStamp& timestamp, int armor_id)
{
    if(firstUpdate)
    {
        initMotionModel();
        firstUpdate = false;
        VectorX first_state = first_state_estimate(measure_vec, armor_id);
        ekf.setX(first_state);
        ekf.setTimeStamp(timestamp);
    }
    else
    {   
        
        VectorY measure_pred;
        measure.setMode(false);
        measure.setId(armor_id);
        measure(ekf.predict(timestamp).data(), measure_pred.data());
        VectorY measure_adjust = measure_vec;

        measure_adjust[0] = std::remainder(measure_vec[0] - measure_pred[0], 2 * M_PI) + measure_pred[0];
        measure_adjust[1] = std::remainder(measure_vec[1] - measure_pred[1], 2 * M_PI) + measure_pred[1];

        measure_adjust[3] = std::remainder(measure_vec[3] - measure_pred[3], 2 * M_PI) + measure_pred[3];
        measure_adjust[4] = std::remainder(measure_vec[4] - measure_pred[4], 2 * M_PI) + measure_pred[4];
        measure_adjust[5] = std::remainder(measure_vec[5] - measure_pred[5], 2 * M_PI) + measure_pred[5];
        measure_adjust[6] = std::remainder(measure_vec[6] - measure_pred[6], 2 * M_PI) + measure_pred[6];


        // auto x = ekf.getX();
        // auto theta = x[4]+ M_PI / 2. * armor_id;
        // auto should_armor_yaw = M_PI/2 - theta;
        //  std::cout<<"should"<<should_armor_yaw<<std::endl;
        //  std::cout<<"measure_should"<<measure_vec[2]<<std::endl;
        // //measure_adjust[2] = std::fmod(measure[2] - should_armor_yaw + M_PI, 2 * M_PI) + should_armor_yaw - M_PI;
        // measure_adjust[2] = std::remainder(measure_vec[2] - should_armor_yaw, 2 * M_PI) + should_armor_yaw;
        // std::cout<<"measure_adjust"<<measure_adjust[2]<<std::endl;
        // std::cout<<"id"<<armor_id<<std::endl;
        //std::cout<<ekf.getP()<<std::endl;
        //std::cout<<"P trace:"<<ekf.getX().trace()<<std::endl;
        //calc residual
        VectorY residual = measure_adjust - measure_pred;
        static auto residual_logger = rclcpp::get_logger("predictor.motion_model");
        RCLCPP_DEBUG(
            residual_logger,
            "residual: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
            residual[0], residual[1], residual[2], residual[3], residual[4], residual[5]);

        // according to residual
        // determine whole car stable & armor stable
       if(residual[0] > 0.5 || residual[1] > 0.5 || residual[2] > 0.5)
       {
           whole_car_stable = false;
           armor_stable = false;
       }
       else if(residual[0] > 0.2 || residual[1] > 0.2 || residual[2] > 0.2)
       {
           whole_car_stable = true;
           armor_stable = false;
       }
       else
       {
           whole_car_stable = true;
           armor_stable = true;
       }
        // whole_car_stable = true;
        // armor_stable = true;
        
        ekf.setTotalId(id1, id2);
        roslog::info("set total id: {}, {}", id1, id2);
        VectorX state = ekf.update(measure_adjust, timestamp, armor_id);
        ekf.resetVisibleId();
        if(state[6] < 0.2) state[6] = 0.2;
        if(state[7] < 0.2) state[7] = 0.2;
        ekf.setX(state);
    }
}
}
