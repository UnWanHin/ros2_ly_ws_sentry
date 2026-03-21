// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <vector>
#include <memory>
#include "solver/solver.hpp"
#include "utils/utils.h"
#include "EKF.hpp"

#define OUTPOST_RADIUS 0.2765    // outpost radius in meters
#define OUTPOST_OMEGA 0.88 * M_PI
namespace PREDICTOR
{
    struct OutpostInformation
    {
        // center position
        Eigen::Vector3d center_position;
        // center velocity
        Eigen::Vector3d center_velocity;
        // radius
        double outpost_radius;
        // theta
        double outpost_theta;
        
        double outpost_omega; // 角速度

        bool is_valid = true; // 是否有效
        
    };

    class OutpostEstimator
    {
    public:
        OutpostEstimator();

        void setDeltaTime(float delta_t)
        {
            this->dt = delta_t;
        }


        bool inputNewMeasurement(SOLVER::ArmorPose &,double);
        void initEstimator(SOLVER::ArmorPose &, int);
        OutpostInformation getInformation(bool);
        void noMeasurementUpdate();
        

    private:
        // (x_c,v_x,y_c,v_y,z_c,v_c,theta,omega) (yaw_a,pitch_a,dis_a,theta_a)
        std::unique_ptr<ExtendedKalman<double, 8, 4>> outpost_state_filter;
        float dt;
        int derection; // 方向：-1为顺时针，1为逆时针，0为未判断
        
        float chi_square_threshold_ = 11.07;
        float chi_square_value_ = 0.0;

        // 卡方检验
        bool ChiSquareTest(
        const Eigen::Matrix<double, 4, 1>& innovation,
        const Eigen::Matrix<double, 4, 4>& innovation_cov
        );
        
        class CenterState2Measure
        {
        public:
            template <typename T>
            void operator()(const T center_state[8], T measure[4])
            {
                // state
                T x_c = center_state[0];
                T y_c = center_state[2];
                T z_c = center_state[4];
                T theta_c = center_state[6];
                T omega_c = center_state[7];

                // measure
                std::reference_wrapper<T> yaw_a = std::ref(measure[0]);
                std::reference_wrapper<T> pitch_a = std::ref(measure[1]);
                std::reference_wrapper<T> dis_a = std::ref(measure[2]);
                std::reference_wrapper<T> theta_a = std::ref(measure[3]);

                T x_a, y_a, z_a;        // 装甲板中心

                x_a = x_c + OUTPOST_RADIUS * sin(theta_c);
                y_a = y_c - OUTPOST_RADIUS * cos(theta_c);
                z_a = z_c;


                // 得到measure
                yaw_a.get() = -atan2(x_a, y_a);
                pitch_a.get() = atan2(z_a, sqrt(x_a * x_a + y_a * y_a));
                dis_a.get() = sqrt(x_a * x_a + y_a * y_a + z_a * z_a);
                theta_a.get() = theta_c;
                // // 把yaw和theta限制在 [-pi, pi]范围内
                // if(measure[0]>M_PI) measure[0]-=2*M_PI;
                // else if(measure[0]<-M_PI) measure[0]+=2*M_PI;

                // if(measure[3]>M_PI) measure[3]-=2*M_PI;
                // else if(measure[3]<-M_PI) measure[3]+=2*M_PI;
            }
        };

        CenterState2Measure center_state_2_measure;

        void setMeasureNoise(float &);
        void setProcessNoise();
        void setTransitionMatrix();


        SOLVER::ArmorPose last_armor_pose; // 上一次装甲板信息
    };

    class OutpostPredictor
    {
    public:
        OutpostPredictor();

        void initPredictor(SOLVER::ArmorPose &, int);
        void resetPredictor();
        OutpostInformation runPredictor(int &, SOLVER::ArmorPose &, bool);
        bool is_initialized;

    private:
        float delta_t;
        std::unique_ptr<OutpostEstimator> outpost_estimator;
    };

} // namespace PREDICTOR