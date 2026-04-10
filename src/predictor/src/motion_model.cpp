// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "predictor/motion_model.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace ly_auto_aim::inline predictor {
extern rclcpp::Node::SharedPtr global_predictor_node;
namespace {
constexpr double kDefaultFilterDt = 0.015;
constexpr double kMinFilterDt = 1e-3;
constexpr double kMaxFilterDt = 0.20;
constexpr double kNoiseMin = 1e-9;

double clampPositive(const double value, const double fallback) {
    if (!std::isfinite(value) || value <= kNoiseMin) {
        return fallback;
    }
    return value;
}

double clampFilterDt(const double dt) {
    if (!std::isfinite(dt)) {
        return kDefaultFilterDt;
    }
    return std::clamp(dt, kMinFilterDt, kMaxFilterDt);
}

double readDoubleParam(const std::string& key, const double default_value) {
    if (!global_predictor_node) {
        return default_value;
    }
    if (!global_predictor_node->has_parameter(key)) {
        global_predictor_node->declare_parameter(key, default_value);
    }
    double value = default_value;
    global_predictor_node->get_parameter(key, value);
    if (!std::isfinite(value)) {
        RCLCPP_WARN(
            global_predictor_node->get_logger(),
            "Invalid non-finite param %s, fallback to %.6f",
            key.c_str(),
            default_value);
        return default_value;
    }
    return value;
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
    loadTuningParams();

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

    // 平移响应增强：提高速度/加速度不确定性与耦合，缩短速度收敛时间
    // P(0, 1) = P(1, 0) = 0.45;
    // P(2, 3) = P(3, 2) = 0.45;
    // P(1, 1) = 3.2;
    // P(3, 3) = 3.2;
    // P(0, 10) = P(10, 0) = 0.10;
    // P(2, 11) = P(11, 2) = 0.10;
    // P(1, 10) = P(10, 1) = 0.80;
    // P(3, 11) = P(11, 3) = 0.80;
    // P(10, 10) = 10.0;
    // P(11, 11) = 10.0;
         

    base_measurement_noise_ = MatrixYY::Zero();
    base_measurement_noise_ << 0.01,  0,     0,     0,      0,      0,      0,      0,      0,      0,
                               0,     0.01,  0,     0,      0,      0,      0,      0,      0,      0,
                               0,     0,     0.1,   0,      0,      0,      0,      0,      0,      0,
                               0,     0,     0,     0.04,   0,      0,      0,      0,      0,      0,
                               0,     0,     0,     0,      0.04,   0.015,  0.01,   0,      0,      0,
                               0,     0,     0,     0,      0.015,  0.04,   0.01,   0,      0,      0,
                               0,     0,     0,     0,      0.01,   0.01,   0.02,   0,      0,      0,
                               0,     0,     0,     0,      0,      0,      0,      0.03,   0.01,   0.015,
                               0,     0,     0,     0,      0,      0,      0,      0.01,   0.03,   0.015,
                               0,     0,     0,     0,      0,      0,      0,      0.015,  0.015,  0.03;
    base_measurement_noise_ *= measurement_noise_scale_;
    yaw_boundary_var_ema_ = std::max(
        kNoiseMin,
        (base_measurement_noise_(4, 4) + base_measurement_noise_(5, 5) + base_measurement_noise_(6, 6)) / 3.0);
    pitch_boundary_var_ema_ = std::max(
        kNoiseMin,
        (base_measurement_noise_(7, 7) + base_measurement_noise_(8, 8) + base_measurement_noise_(9, 9)) / 3.0);

    ekf.init(P, buildProcessNoise(kDefaultFilterDt), base_measurement_noise_);
}

void MotionModel::loadTuningParams()
{
    const double default_q_ax_jerk = q_ax_jerk_base_;
    const double default_q_ay_jerk = q_ay_jerk_base_;
    const double default_q_omega_accel = q_omega_accel_base_;
    const double default_q_r1_drift = q_r1_drift_base_;
    const double default_q_r2_drift = q_r2_drift_base_;
    const double default_q_z1_drift = q_z1_drift_base_;
    const double default_q_z2_drift = q_z2_drift_base_;
    const double default_r_scale = measurement_noise_scale_;
    const double default_boundary_alpha = boundary_fit_alpha_;
    const double default_boundary_scale_min = boundary_scale_min_;
    const double default_boundary_scale_max = boundary_scale_max_;

    q_ax_jerk_base_ = clampPositive(
        readDoubleParam("motion_model.q_ax_jerk_base", default_q_ax_jerk),
        default_q_ax_jerk);
    q_ay_jerk_base_ = clampPositive(
        readDoubleParam("motion_model.q_ay_jerk_base", default_q_ay_jerk),
        default_q_ay_jerk);
    q_omega_accel_base_ = clampPositive(
        readDoubleParam("motion_model.q_omega_accel_base", default_q_omega_accel),
        default_q_omega_accel);
    q_r1_drift_base_ = clampPositive(
        readDoubleParam("motion_model.q_r1_drift_base", default_q_r1_drift),
        default_q_r1_drift);
    q_r2_drift_base_ = clampPositive(
        readDoubleParam("motion_model.q_r2_drift_base", default_q_r2_drift),
        default_q_r2_drift);
    q_z1_drift_base_ = clampPositive(
        readDoubleParam("motion_model.q_z1_drift_base", default_q_z1_drift),
        default_q_z1_drift);
    q_z2_drift_base_ = clampPositive(
        readDoubleParam("motion_model.q_z2_drift_base", default_q_z2_drift),
        default_q_z2_drift);
    measurement_noise_scale_ = clampPositive(
        readDoubleParam("motion_model.r_scale", default_r_scale),
        default_r_scale);
    boundary_fit_alpha_ = std::clamp(
        readDoubleParam("motion_model.boundary_fit_alpha", default_boundary_alpha),
        0.01,
        1.0);
    boundary_scale_min_ = clampPositive(
        readDoubleParam("motion_model.boundary_scale_min", default_boundary_scale_min),
        default_boundary_scale_min);
    boundary_scale_max_ = clampPositive(
        readDoubleParam("motion_model.boundary_scale_max", default_boundary_scale_max),
        default_boundary_scale_max);
    if (boundary_scale_max_ < boundary_scale_min_) {
        std::swap(boundary_scale_min_, boundary_scale_max_);
    }
}

MatrixXX MotionModel::buildProcessNoise(double dt) const
{
    dt = clampFilterDt(dt);
    MatrixXX Q_val = MatrixXX::Zero();

    const Eigen::Matrix3d Q_xva = build_Q_PVA(q_ax_jerk_base_, dt);
    Q_val(0,0)   = Q_xva(0,0); Q_val(0,1)   = Q_xva(0,1); Q_val(0,10)  = Q_xva(0,2);
    Q_val(1,0)   = Q_xva(1,0); Q_val(1,1)   = Q_xva(1,1); Q_val(1,10)  = Q_xva(1,2);
    Q_val(10,0)  = Q_xva(2,0); Q_val(10,1)  = Q_xva(2,1); Q_val(10,10) = Q_xva(2,2);

    const Eigen::Matrix3d Q_yva = build_Q_PVA(q_ay_jerk_base_, dt);
    Q_val(2,2)   = Q_yva(0,0); Q_val(2,3)   = Q_yva(0,1); Q_val(2,11)  = Q_yva(0,2);
    Q_val(3,2)   = Q_yva(1,0); Q_val(3,3)   = Q_yva(1,1); Q_val(3,11)  = Q_yva(1,2);
    Q_val(11,2)  = Q_yva(2,0); Q_val(11,3)  = Q_yva(2,1); Q_val(11,11) = Q_yva(2,2);

    const Eigen::Matrix2d Q_to = build_Q_AngleVel(q_omega_accel_base_, dt);
    Q_val(4,4) = Q_to(0,0); Q_val(4,5) = Q_to(0,1);
    Q_val(5,4) = Q_to(1,0); Q_val(5,5) = Q_to(1,1);

    Q_val(6,6) = q_r1_drift_base_ * dt;
    Q_val(7,7) = q_r2_drift_base_ * dt;
    Q_val(8,8) = q_z1_drift_base_ * dt;
    Q_val(9,9) = q_z2_drift_base_ * dt;
    return Q_val;
}

void MotionModel::refreshNoiseCovariances(const double dt)
{
    ekf.setQ(buildProcessNoise(dt));
    ekf.setR(base_measurement_noise_);
}

void MotionModel::fitBoundaryErrorAndUpdateR(const VectorY& residual)
{
    const double yaw_var_now = (residual[4] * residual[4] + residual[5] * residual[5] + residual[6] * residual[6]) / 3.0;
    const double pitch_var_now =
        (residual[7] * residual[7] + residual[8] * residual[8] + residual[9] * residual[9]) / 3.0;

    yaw_boundary_var_ema_ =
        (1.0 - boundary_fit_alpha_) * yaw_boundary_var_ema_ + boundary_fit_alpha_ * std::max(yaw_var_now, kNoiseMin);
    pitch_boundary_var_ema_ = (1.0 - boundary_fit_alpha_) * pitch_boundary_var_ema_ +
                              boundary_fit_alpha_ * std::max(pitch_var_now, kNoiseMin);

    const double yaw_base_var =
        std::max(kNoiseMin, (base_measurement_noise_(4, 4) + base_measurement_noise_(5, 5) + base_measurement_noise_(6, 6)) / 3.0);
    const double pitch_base_var =
        std::max(kNoiseMin, (base_measurement_noise_(7, 7) + base_measurement_noise_(8, 8) + base_measurement_noise_(9, 9)) / 3.0);

    const double yaw_scale = std::clamp(yaw_boundary_var_ema_ / yaw_base_var, boundary_scale_min_, boundary_scale_max_);
    const double pitch_scale =
        std::clamp(pitch_boundary_var_ema_ / pitch_base_var, boundary_scale_min_, boundary_scale_max_);

    MatrixYY adaptive_r = base_measurement_noise_;
    for (int i = 4; i <= 6; ++i) {
        for (int j = 4; j <= 6; ++j) {
            adaptive_r(i, j) = base_measurement_noise_(i, j) * yaw_scale;
        }
    }
    for (int i = 7; i <= 9; ++i) {
        for (int j = 7; j <= 9; ++j) {
            adaptive_r(i, j) = base_measurement_noise_(i, j) * pitch_scale;
        }
    }
    ekf.setR(adaptive_r);
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
    measure_adjust[7] = std::remainder(measure_vec[7] - measure_pred[7], 2 * M_PI) + measure_pred[7];
    measure_adjust[8] = std::remainder(measure_vec[8] - measure_pred[8], 2 * M_PI) + measure_pred[8];
    measure_adjust[9] = std::remainder(measure_vec[9] - measure_pred[9], 2 * M_PI) + measure_pred[9];
    if (!measure_adjust.allFinite()) {
        roslog::warn("Predictor adjusted measurement became non-finite, skip this update");
        return;
    }

    const VectorY innovation = measure_adjust - measure_pred;

    static auto residual_logger = rclcpp::get_logger("predictor.motion_model");
    RCLCPP_DEBUG(
        residual_logger,
        "residual: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        innovation[0], innovation[1], innovation[2], innovation[3],
        innovation[4], innovation[5], innovation[6], innovation[7],
        innovation[8], innovation[9]);
    fitBoundaryErrorAndUpdateR(innovation);

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
    // roslog::info("set total id: {}, {}", id1, id2);
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
