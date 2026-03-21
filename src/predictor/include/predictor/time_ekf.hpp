// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

// 包装了ekf.hpp，用于更方便的时间序列预测
#pragma once
#include "ekf.hpp"
#include "TimeStamp/TimeStamp.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp> // [ROS 2]
// #include <Logger/Logger.hpp> // 避免頭文件汙染導致衝突

// [ROS 2] 本地簡單適配，防止模板實例化時找不到符號
namespace ekf_log {
    static rclcpp::Logger get_logger() {
        return rclcpp::get_logger("time_ekf");
    }
}

namespace ekf::timeEKF
{
template<class stateTransFunc, class measureFunc,int N_X, int N_Y>
class TimeEKF 
{
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    static constexpr double kDefaultDt = 0.015;
    static constexpr double kMinDt = 1e-3;
    static constexpr double kMaxDt = 0.20;
    public:
    // predict func there will not update inner state
    // which is significant different from so-called predict in EKF
    VectorX predict(const Time::TimeStamp& timestamp)
    {
        VectorX X = ekf.Xe;
        VectorX Xp;
        stateTrans.setDt(previewDt(timestamp));
        stateTrans(X, Xp);
        return Xp;
    }
    VectorX predict(double dt)
    {
        VectorX X = ekf.Xe;
        VectorX Xp;
        stateTrans.setDt(dt);
        //stateTrans(X, Xp);
        stateTrans(X.data(), Xp.data());
        return Xp;
    }
    void setTotalId(int id1, int id2=-1)
    {
        measure.setVisibleId(id1, id2);
    }
    void resetVisibleId()
    {
        measure.resetVisibleId();
    }
    // dont check whether timestamp initialized, just take care of it by yourself 
    VectorX update(const VectorY& Y, const Time::TimeStamp& timestamp, int id)
    {
        stateTrans.setDt(previewDt(timestamp));
        ekf.predict(stateTrans);
        lastTime = timestamp;
        measure.setMode(true);
        measure.setId(id);
        VectorX result = ekf.update(measure, Y);
        //measure.resetVisibleId();
        return result;
    }
    //get P
    MatrixXX getP()
    {
        return ekf.P;
    }
    
    void init(const MatrixXX& P, const MatrixXX& Q, const MatrixYY& R)
    {
        ekf.P = P;
        ekf.Q = Q;
        ekf.R = R;
    }
    void setX(const VectorX& X)
    {
        ekf.Xe = X;
    }
    VectorX getX()
    {
        return ekf.Xe;
    }
    double previewDt(const Time::TimeStamp& timestamp) const
    {
        return sanitizeDt((timestamp - lastTime).seconds());
    }
    void setQ(const MatrixXX& Q)
    {
        ekf.Q = Q;
    }
    void setR(const MatrixYY& R)
    {
        ekf.R = R;
    }
    void setTimeStamp(const Time::TimeStamp& timestamp)
    {
        lastTime = timestamp;
    }
        base::EKF<N_X, N_Y> ekf;
    private:
        static double sanitizeDt(const double dt)
        {
            if (!std::isfinite(dt)) {
                return kDefaultDt;
            }
            return std::clamp(dt, kMinDt, kMaxDt);
        }
        stateTransFunc stateTrans;
        measureFunc measure;
        Time::TimeStamp lastTime;
};

template<class stateTransFunc, class measureFunc1, class measureFunc2, int N_X, int N_Y1, int N_Y2>
class TimeBMEKF
{
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY1 = Eigen::Matrix<double, N_Y1, 1>;
    using VectorY2 = Eigen::Matrix<double, N_Y2, 1>;
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYY1 = Eigen::Matrix<double, N_Y1, N_Y1>;
    using MatrixYY2 = Eigen::Matrix<double, N_Y2, N_Y2>;
    public:
    VectorX predict(const Time::TimeStamp& timestamp)
    {
        VectorX X = ekf.Xe;
        VectorX Xp;
        // [ROS 2] toSeconds -> seconds
        stateTrans.setDt((timestamp - lastTime).seconds());
        stateTrans(X.data(), Xp.data());
        return Xp;
    }
    VectorX predict(double dt)
    {
        VectorX X = ekf.Xe;
        VectorX Xp;
        stateTrans.setDt(dt);
        stateTrans(X.data(), Xp.data());
        return Xp;
    }
    VectorX update(const VectorY1& Y1, const Time::TimeStamp& timestamp, int id)
    {
        stateTrans.setDt((timestamp - lastTime).seconds());
        ekf.predict(stateTrans);
        lastTime = timestamp;
        measure1.setId(id);
        return ekf.update(measure1, Y1);
    }
    VectorX update(const VectorY2& Y2, const Time::TimeStamp& timestamp, int id1, int id2)
    {
        stateTrans.setDt((timestamp - lastTime).seconds());
        ekf.predict(stateTrans);
        lastTime = timestamp;
        measure2.setId(id1,id2);
        return ekf.update(measure2, Y2);
    }

    void init(const MatrixXX& P, const MatrixXX& Q, const MatrixYY1& R1, const MatrixYY2& R2)
    {
        ekf.P = P;
        ekf.Q = Q;
        ekf.R1 = R1;
        ekf.R2 = R2;
    }
    void setX(const VectorX& X)
    {
        ekf.Xe = X;
    }

    private:
        base::BMEKF<N_X, N_Y1, N_Y2> ekf;
        stateTransFunc stateTrans;
        measureFunc1 measure1;
        measureFunc2 measure2;
        Time::TimeStamp lastTime;
};
}
