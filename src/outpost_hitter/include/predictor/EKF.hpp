// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include "ceres/jet.h"
#include "eigen3/Eigen/Dense"
#include <eigen3/Eigen/Core>
#include <vector>

namespace PREDICTOR
{
    template <typename T, int x, int y>
    class ExtendedKalman
    {
    public:
        Eigen::Matrix<T, x, y> K; // 卡尔曼增益

        Eigen::Matrix<T, y, 1> Z_k; // 测量值

        Eigen::Matrix<T, x, 1> X_j;   // 先验估计
        Eigen::Matrix<T, y, 1> Z_x_j; // 先验估计测量

        Eigen::Matrix<T, x, x> F;   // 状态转移矩阵
        Eigen::Matrix<T, x, 1> X_k; // 后验估计
        Eigen::Matrix<T, x, x> P;   // 状态估计协方差矩阵
        Eigen::Matrix<T, x, x> Q;   // 过程噪声矩阵
        Eigen::Matrix<T, y, y> R;   // 测量噪声矩阵
        Eigen::Matrix<T, y, x> H;   // 测量矩阵

        Eigen::Matrix<T, y, 1> residual; // 残差向量

        Eigen::Matrix<bool, y, 1> measure_is_radian; // 观测维度是否为弧度

        bool is_residual_matrix_init;

    public:
        ExtendedKalman()
        {
            P = Eigen::Matrix<T, x, x>::Identity();
            H = Eigen::Matrix<T, y, x>::Zero();
            Q = Eigen::Matrix<T, x, x>::Identity();
            X_k = Eigen::Matrix<T, x, 1>::Zero();
            R = Eigen::Matrix<T, y, y>::Identity();

            is_residual_matrix_init = false;
        }

        // 设置测量维度是否为弧度制（为true时为弧度观测）
        void setRadianMeasure(Eigen::Matrix<bool, y, 1> is_radian)
        {
            this->measure_is_radian = is_radian;
        }

        Eigen::Matrix<T, x, 1> predict()
        {
            X_j = F * X_k;

            P = F * P * F.transpose() + Q;

            return X_j;
        }

        template <class Func>
        Eigen::Matrix<T, x, 1> update(Func &&func, const Eigen::Matrix<T, y, 1> &measure_vec)
        {
            ceres::Jet<double, x> Xp_auto_jet[x];
            for (int i = 0; i < x; i++)
            {
                Xp_auto_jet[i].a = X_j[i];
                Xp_auto_jet[i].v[i] = 1;
            }
            ceres::Jet<double, x> Yp_auto_jet[y];
            func(Xp_auto_jet, Yp_auto_jet);
            for (int i = 0; i < y; i++)
            {
                Z_x_j[i] = Yp_auto_jet[i].a;
                H.block(i, 0, 1, x) = Yp_auto_jet[i].v.transpose();
            }

            // 计算原始残差
            residual = measure_vec - Z_x_j;

            // 对所有球面坐标系的角度残差做保护
            for (int i = 0; i < y; ++i)
            {
                if (measure_is_radian[i])
                {
                    if (residual[i] > M_PI)
                    {
                        residual[i] -= 2 * M_PI;
                    }
                    else if (residual[i] < -M_PI)
                    {
                        residual[i] += 2 * M_PI;
                    }
                }
            }

            K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
            X_k = X_j + K * residual;
            P = (Eigen::Matrix<T, x, x>::Identity() - K * H) * P;

            return X_k;
        }
    };
} // namespace PREDICTOR