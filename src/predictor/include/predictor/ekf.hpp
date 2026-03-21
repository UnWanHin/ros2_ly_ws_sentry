// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

// Thanks to xinyang of SJTU.
#pragma once
#include <ceres/jet.h>
#include <Eigen/Dense>

namespace ekf::base{

template<int N_X, int N_Y>
class EKF {
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
    using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;

public:
    explicit EKF(const VectorX &X0 = VectorX::Zero())
            : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity()), R(MatrixYY::Identity()) {}

    template<class Func>
    VectorX predict(Func &&func) {
        ceres::Jet<double, N_X> Xe_auto_jet[N_X];
        for (int i = 0; i < N_X; i++) {
            Xe_auto_jet[i].a = Xe[i];
            Xe_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        func(Xe_auto_jet, Xp_auto_jet);
        for (int i = 0; i < N_X; i++) {
            Xp[i] = Xp_auto_jet[i].a;
            F.block(i, 0, 1, N_X) = Xp_auto_jet[i].v.transpose();
        }
        P = F * P * F.transpose() + Q;
        return Xp;
    }

    template<class Func>
    VectorX update(Func &&func, const VectorY &Y) {
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        for (int i = 0; i < N_X; i++) {
            Xp_auto_jet[i].a = Xp[i];
            Xp_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Yp_auto_jet[N_Y];
        func(Xp_auto_jet, Yp_auto_jet);
        for (int i = 0; i < N_Y; i++) {
            Yp[i] = Yp_auto_jet[i].a;
            H.block(i, 0, 1, N_X) = Yp_auto_jet[i].v.transpose();
        }
        const MatrixYY S = H * P * H.transpose() + R;
        K = P * H.transpose() * S.inverse();
        Xe = Xp + K * (Y - Yp);
        const MatrixXX I = MatrixXX::Identity();
        const MatrixXX I_KH = I - K * H;
        P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
        return Xe;
    }

    VectorX Xe;     // 估计状态变量
    VectorX Xp;     // 预测状态变量
    MatrixXX F;     // 预测雅克比
    MatrixYX H;     // 观测雅克比
    MatrixXX P;     // 状态协方差     *
    MatrixXX Q;     // 预测过程协方差  *   
    MatrixYY R;     // 观测过程协方差  *   
    MatrixXY K;     // 卡尔曼增益
    VectorY Yp;     // 预测观测量
};

template<int N_X, int N_Y1, int N_Y2>
class BMEKF {// Binary Measurement Extended Kalman Filter
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYX1 = Eigen::Matrix<double, N_Y1, N_X>;
    using MatrixXY1 = Eigen::Matrix<double, N_X, N_Y1>;
    using MatrixYY1 = Eigen::Matrix<double, N_Y1, N_Y1>;
    using MatrixYX2 = Eigen::Matrix<double, N_Y2, N_X>;
    using MatrixXY2 = Eigen::Matrix<double, N_X, N_Y2>;
    using MatrixYY2 = Eigen::Matrix<double, N_Y2, N_Y2>;
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY1 = Eigen::Matrix<double, N_Y1, 1>;
    using VectorY2 = Eigen::Matrix<double, N_Y2, 1>;

public:
    explicit BMEKF(const VectorX &X0 = VectorX::Zero())
            : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity())
            , R1(MatrixYY1::Identity()), R2(MatrixYY2::Identity()) {}

    template<class Func>
    VectorX predict(Func &&func) {
        ceres::Jet<double, N_X> Xe_auto_jet[N_X];
        for (int i = 0; i < N_X; i++) {
            Xe_auto_jet[i].a = Xe[i];
            Xe_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        func(Xe_auto_jet, Xp_auto_jet);
        for (int i = 0; i < N_X; i++) {
            Xp[i] = Xp_auto_jet[i].a;
            F.block(i, 0, 1, N_X) = Xp_auto_jet[i].v.transpose();
        }
        P = F * P * F.transpose() + Q;
        return Xp;
    }

    template<class Func>
    VectorX update(Func &&func, const VectorY1 &Y1) {
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        for (int i = 0; i < N_X; i++) {
            Xp_auto_jet[i].a = Xp[i];
            Xp_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Yp_auto_jet[N_Y1];
        func(Xp_auto_jet, Yp_auto_jet);
        for (int i = 0; i < N_Y1; i++) {
            Yp1[i] = Yp_auto_jet[i].a;
            H1.block(i, 0, 1, N_X) = Yp_auto_jet[i].v.transpose();
        }
        const MatrixYY1 S1 = H1 * P * H1.transpose() + R1;
        K1 = P * H1.transpose() * S1.inverse();
        Xe = Xp + K1 * (Y1 - Yp1);
        const MatrixXX I = MatrixXX::Identity();
        const MatrixXX I_KH1 = I - K1 * H1;
        P = I_KH1 * P * I_KH1.transpose() + K1 * R1 * K1.transpose();
        return Xe;
    }

    template<class Func>
    VectorX update(Func &&func, const VectorY2 &Y2) {
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        for (int i = 0; i < N_X; i++) {
            Xp_auto_jet[i].a = Xp[i];
            Xp_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Yp_auto_jet[N_Y2];
        func(Xp_auto_jet, Yp_auto_jet);
        for (int i = 0; i < N_Y2; i++) {
            Yp2[i] = Yp_auto_jet[i].a;
            H2.block(i, 0, 1, N_X) = Yp_auto_jet[i].v.transpose();
        }
        const MatrixYY2 S2 = H2 * P * H2.transpose() + R2;
        K2 = P * H2.transpose() * S2.inverse();
        Xe = Xp + K2 * (Y2 - Yp2);
        const MatrixXX I = MatrixXX::Identity();
        const MatrixXX I_KH2 = I - K2 * H2;
        P = I_KH2 * P * I_KH2.transpose() + K2 * R2 * K2.transpose();
        return Xe;
    }

    VectorX Xe;     // 估计状态变量
    VectorX Xp;     // 预测状态变量
    MatrixXX F;     // 预测雅克比
    MatrixYX1 H1;     // 观测雅克比
    MatrixYX2 H2;     // 观测雅克比
    MatrixXX P;     // 状态协方差     *
    MatrixXX Q;     // 预测过程协方差  *   
    MatrixYY1 R1;     // 观测过程协方差  *   
    MatrixYY2 R2;     // 观测过程协方差  *
    MatrixXY1 K1;     // 卡尔曼增益
    MatrixXY2 K2;     // 卡尔曼增益
    VectorY1 Yp1;     // 预测观测量
    VectorY2 Yp2;     // 预测观测量
};

}// namespace ekf::base
