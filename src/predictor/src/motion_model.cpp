#include "predictor/motion_model.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace ly_auto_aim::inline predictor {
namespace {
constexpr double kDefaultFilterDt = 0.015;
constexpr double kMinFilterDt = 1e-3;
constexpr double kMaxFilterDt = 0.08;
constexpr double kHardOutlierScale = 120.0;
constexpr double kMaxMeasurementScale = 250.0;

double clampFilterDt(const double dt) {
    if (!std::isfinite(dt)) {
        return kDefaultFilterDt;
    }
    return std::clamp(dt, kMinFilterDt, kMaxFilterDt);
}
}  // namespace

inline Eigen::Matrix3d build_Q_PVA(double q_jerk_base, double dt) {
    Eigen::Matrix3d Q_sub;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;

    Q_sub << q_jerk_base * dt5 / 20.0, q_jerk_base * dt4 / 8.0,  q_jerk_base * dt3 / 6.0,
             q_jerk_base * dt4 / 8.0,  q_jerk_base * dt3 / 3.0,  q_jerk_base * dt2 / 2.0,
             q_jerk_base * dt3 / 6.0,  q_jerk_base * dt2 / 2.0,  q_jerk_base * dt;
    return Q_sub;
}

inline Eigen::Matrix2d build_Q_AngleVel(double q_ang_accel_base, double dt) {
    Eigen::Matrix2d Q_sub;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    Q_sub << q_ang_accel_base * dt3 / 3.0, q_ang_accel_base * dt2 / 2.0,
             q_ang_accel_base * dt2 / 2.0, q_ang_accel_base * dt;
    return Q_sub;
}

void MotionModel::initMotionModel()
{
    MatrixXX P = MatrixXX::Zero();
	// step1 卡尔曼里面的状态有什么
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

    // step2 观测的值有哪些
    base_measurement_noise_ = MatrixYY::Zero();
    base_measurement_noise_ << 0.005, 0,     0,     0,     0,     0,     0,
                               0,     0.005, 0,     0,     0,     0,     0,
                               0,     0,     0.1,   0,     0,     0,     0,
                               0,     0,     0,     0.04,  0,     0,     0,
                               0,     0,     0,     0,     0.04,  0.015, 0,
                               0,     0,     0,     0,     0.015, 0.04,  0,
                               0,     0,     0,     0,     0,     0,     0.002;
    base_measurement_noise_ *= 0.25; // step 3 这个噪声怎么取值

    ekf.init(P, buildProcessNoise(kDefaultFilterDt), base_measurement_noise_);
}

MatrixXX MotionModel::buildProcessNoise(double dt) const
{
    dt = clampFilterDt(dt);
    MatrixXX Q_val = MatrixXX::Zero();

    constexpr double q_ax_jerk_base = 1.5;
    constexpr double q_ay_jerk_base = 1.5;
    constexpr double q_omega_accel_base = 0.4;
    constexpr double q_r1_drift_base = 1e-5;
    constexpr double q_r2_drift_base = 1e-5;
    constexpr double q_z1_drift_base = 1e-4;
    constexpr double q_z2_drift_base = 1e-4;

    const Eigen::Matrix3d Q_xva = build_Q_PVA(q_ax_jerk_base, dt);
    Q_val(0,0)   = Q_xva(0,0); Q_val(0,1)   = Q_xva(0,1); Q_val(0,10)  = Q_xva(0,2);
    Q_val(1,0)   = Q_xva(1,0); Q_val(1,1)   = Q_xva(1,1); Q_val(1,10)  = Q_xva(1,2);
    Q_val(10,0)  = Q_xva(2,0); Q_val(10,1)  = Q_xva(2,1); Q_val(10,10) = Q_xva(2,2);

    const Eigen::Matrix3d Q_yva = build_Q_PVA(q_ay_jerk_base, dt);
    Q_val(2,2)   = Q_yva(0,0); Q_val(2,3)   = Q_yva(0,1); Q_val(2,11)  = Q_yva(0,2);
    Q_val(3,2)   = Q_yva(1,0); Q_val(3,3)   = Q_yva(1,1); Q_val(3,11)  = Q_yva(1,2);
    Q_val(11,2)  = Q_yva(2,0); Q_val(11,3)  = Q_yva(2,1); Q_val(11,11) = Q_yva(2,2);

    const Eigen::Matrix2d Q_to = build_Q_AngleVel(q_omega_accel_base, dt);
    Q_val(4,4) = Q_to(0,0); Q_val(4,5) = Q_to(0,1);
    Q_val(5,4) = Q_to(1,0); Q_val(5,5) = Q_to(1,1);

    Q_val(6,6) = q_r1_drift_base * dt;
    Q_val(7,7) = q_r2_drift_base * dt;
    Q_val(8,8) = q_z1_drift_base * dt;
    Q_val(9,9) = q_z2_drift_base * dt;
    return Q_val;
}

void MotionModel::refreshNoiseCovariances(const double dt, const double measurement_scale)
{
    ekf.setQ(buildProcessNoise(dt));
    ekf.setR(base_measurement_noise_ * std::clamp(measurement_scale, 1.0, kMaxMeasurementScale));
}

double MotionModel::computeMeasurementScale(const VectorY& innovation) const
{
    VectorY soft_limit;
    soft_limit << 0.05, 0.05, 0.18, 0.10, 0.10, 0.10, 0.08;
    VectorY hard_limit;
    hard_limit << 0.14, 0.14, 0.45, 0.26, 0.26, 0.26, 0.22;

    const VectorY abs_innovation = innovation.cwiseAbs();
    double scale = 1.0;
    bool has_hard_outlier = false;
    for (int i = 0; i < N_Y; ++i) {
        const double soft = std::max(soft_limit[i], 1e-6);
        const double ratio = abs_innovation[i] / soft;
        if (ratio > 1.0) {
            scale = std::max(scale, 1.0 + 4.0 * ratio * ratio);
        }
        if (abs_innovation[i] > hard_limit[i]) {
            has_hard_outlier = true;
        }
    }
    if (has_hard_outlier) {
        scale = std::max(scale, kHardOutlierScale);
    }
    return std::min(scale, kMaxMeasurementScale);
}

VectorY MotionModel::limitInnovation(const VectorY& innovation) const
{
    VectorY hard_limit;
    hard_limit << 0.14, 0.14, 0.45, 0.26, 0.26, 0.26, 0.22;

    VectorY limited = innovation;
    for (int i = 0; i < N_Y; ++i) {
        limited[i] = std::clamp(limited[i], -hard_limit[i], hard_limit[i]);
    }
    return limited;
}

VectorX MotionModel::getPredictResult(const Time::TimeStamp& timestamp)
{
    return ekf.predict(timestamp);
}

void MotionModel::Update(const VectorY& measure_vec, const Time::TimeStamp& timestamp, int armor_id)
{
    if (firstUpdate) {
        initMotionModel();
        firstUpdate = false;
        VectorX first_state = first_state_estimate(measure_vec, armor_id);
        ekf.setX(first_state);
        ekf.setTimeStamp(timestamp);
        return;
    }

    const double dt = ekf.previewDt(timestamp);
    refreshNoiseCovariances(dt);

    VectorY measure_pred;
    measure.setMode(false);
    measure.setId(armor_id);
    measure(ekf.predict(timestamp).data(), measure_pred.data());
    if (!measure_pred.allFinite()) {
        roslog::warn("Predictor measure prediction became non-finite, reset motion model");
        reset();
        return;
    }

    VectorY measure_adjust = measure_vec;
    measure_adjust[0] = std::remainder(measure_vec[0] - measure_pred[0], 2 * M_PI) + measure_pred[0];
    measure_adjust[1] = std::remainder(measure_vec[1] - measure_pred[1], 2 * M_PI) + measure_pred[1];
    measure_adjust[3] = std::remainder(measure_vec[3] - measure_pred[3], 2 * M_PI) + measure_pred[3];
    measure_adjust[4] = std::remainder(measure_vec[4] - measure_pred[4], 2 * M_PI) + measure_pred[4];
    measure_adjust[5] = std::remainder(measure_vec[5] - measure_pred[5], 2 * M_PI) + measure_pred[5];
    measure_adjust[6] = std::remainder(measure_vec[6] - measure_pred[6], 2 * M_PI) + measure_pred[6];
    if (!measure_adjust.allFinite()) {
        roslog::warn("Predictor adjusted measurement became non-finite, skip this update");
        return;
    }

    VectorY innovation = measure_adjust - measure_pred;
    const double measurement_scale = computeMeasurementScale(innovation);
    innovation = limitInnovation(innovation);
    measure_adjust = measure_pred + innovation;
    refreshNoiseCovariances(dt, measurement_scale);

    static auto residual_logger = rclcpp::get_logger("predictor.motion_model");
    RCLCPP_DEBUG(
        residual_logger,
        "residual: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f], scale=%.2f",
        innovation[0], innovation[1], innovation[2], innovation[3],
        innovation[4], innovation[5], innovation[6], measurement_scale);

    const VectorY innovation_abs = innovation.cwiseAbs();
    if (innovation_abs[0] > 0.5 || innovation_abs[1] > 0.5 || innovation_abs[2] > 0.5) {
        whole_car_stable = false;
        armor_stable = false;
    } else if (innovation_abs[0] > 0.2 || innovation_abs[1] > 0.2 || innovation_abs[2] > 0.2) {
        whole_car_stable = true;
        armor_stable = false;
    } else {
        whole_car_stable = true;
        armor_stable = true;
    }

    ekf.setTotalId(id1, id2);
    roslog::info("set total id: {}, {}", id1, id2);
    VectorX state = ekf.update(measure_adjust, timestamp, armor_id);
    ekf.resetVisibleId();
    if (!state.allFinite()) {
        roslog::warn("Predictor state became non-finite after update, reset motion model");
        reset();
        return;
    }
    if (state[6] < 0.2) state[6] = 0.2;
    if (state[7] < 0.2) state[7] = 0.2;
    ekf.setX(state);
}
}  // namespace ly_auto_aim::predictor
