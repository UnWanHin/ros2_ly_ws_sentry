// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <RosTools/RosTools.hpp>
#include <Logger/Logger.hpp>
#include "auto_aim_common/Location.hpp"
#include "auto_aim_common/SolverType.hpp"
#include "auto_aim_common/TrackerType.hpp"
#include <rclcpp/rclcpp.hpp> // [ROS 2]

namespace ly_auto_aim::inline solver {
using tracker::ArmorXYV;

// [ROS 2] 聲明全局節點指針
extern rclcpp::Node::SharedPtr global_tracker_solver_node;

struct CameraIntrinsicsParameterPack {
    float FocalLength[2]{1331.1f, 1330.1f};
    float PrincipalPoint[2]{624.5817f, 526.3662f};
    float RadialDistortion[3]{-0.1059f, -0.3427f, 1.4125f};
    float TangentialDistortion[2]{0.0072f, 0};

    Eigen::Matrix3d GetCameraMatrix() const;

    Eigen::Vector5d GetDistortionCoefficients() const;
};

/**
 * @brief 完整名称其实是pnp_solver，使用了pnp算法来计算距离，但是使用线性拟合的办法计算全局的yaw和pitch
 * @note 后面可以把pitch和yaw的的计算优化一下
 */
class Solver : public BaseSolver {
private:
    Eigen::Matrix3d cameraIntrinsicMatrix;
    Eigen::Vector3d cameraOffset;
    Eigen::Vector5d distorationCoefficients; // k1,k2,p1,p2,k3
    Eigen::Vector3d cameraRotation;
    Eigen::Matrix3d cameraRotationMatrix;
    double f_x, f_y, c_x, c_y;

public:
    // [ROS 2] 構造函數只聲明 (實現移至 .cpp)
    explicit Solver();

    inline PYD XYZ2PYD(const XYZ &in) const override {
        double distance = sqrt(in.x * in.x + in.y * in.y + in.z * in.z);
        double pitch = asin(in.z / distance);
        double yaw = atan2(in.y, in.x);
        return PYD(pitch, yaw, distance);
    }
    inline XYZ PYD2XYZ(const PYD &in) const override {
        XYZ out;
        out.x = in.distance * cos(in.pitch) * cos(in.yaw);
        out.y = in.distance * cos(in.pitch) * sin(in.yaw);
        out.z = in.distance * sin(in.pitch);
        return out;
    }
	inline XYZ CXYD2XYZ(const CXYD& in) const override
	{
		//eliminate distortion
        XYZ out;
        out.x = in.k;
        out.y = (c_x - in.cx) * in.k / f_x;
        out.z = (c_y - in.cy) * in.k / f_y;
		return out;
	}

    // [關鍵修復] 參數類型改為 const XYZ& (原本錯誤寫成 const CXYD&)
	inline CXYD XYZ2CXYD(const XYZ& in) const override
	{
        double cx = c_x - in.y / in.x * f_x;
        double cy = c_y - in.z / in.x * f_y;
        
        CXYD out;
        out.cx = cx;
        out.cy = cy;
        out.k = in.x;
		return out;
	}

    inline XYZ camera2world(const XYZ& in, const PYD& imuData) const override
	{
        double imu_yaw = imuData.yaw;
        double imu_pitch = imuData.pitch;
        Eigen::Vector3d in_eigen = Eigen::Vector3d(in.x, in.y, in.z);
        Eigen::Vector3d gimbal = cameraRotationMatrix * in_eigen + cameraOffset;
        Eigen::Vector3d world = Eigen::AngleAxisd(-imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                                 * Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                                            * gimbal;
        return XYZ(world(0), world(1), world(2));
	}
	inline XYZ world2camera(const XYZ& in, const PYD& imuData) const override
	{
        double imu_yaw = imuData.yaw;
        double imu_pitch = imuData.pitch;
        Eigen::Vector3d in_eigen = Eigen::Vector3d(in.x, in.y, in.z);
        // Keep this as the exact inverse of camera2world():
        // world = Ry(-pitch) * Rz(yaw) * gimbal
        // gimbal = Rz(-yaw) * Ry(pitch) * world
        Eigen::Vector3d gimbal = Eigen::AngleAxisd(-imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                                 * Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                                            * in_eigen;
        Eigen::Vector3d camera = cameraRotationMatrix.inverse() * (gimbal - cameraOffset);

        return XYZ(camera(0), camera(1), camera(2));
	}

    std::pair<XYZ, double> camera2world(
        const ArmorXYV &trackResult, const GimbalAngleType &gimbalAngle_deg, bool isLarge);
    std::pair<XYZ, double> camera2worldWithWholeCar(const ArmorXYV &trackResult, const GimbalAngleType &gimbalAngle_deg,
        const cv::Rect &bounding_rect, bool isLarge);

    void solve_all( std::pair<std::vector<TrackResult>, std::vector<CarTrackResult>>& trackResults, 
                    GimbalAngleType& gimbalAngle_deg);

    void setCameraIntrinsicMatrix(const Eigen::Matrix3d &cameraIntrinsicMatrix);
    void setCameraOffset(const Eigen::Vector3d &cameraOffset);
    void setDistorationCoefficients(const Eigen::Vector5d &distorationCoefficients);
};
std::shared_ptr<Solver> createSolver();
} // namespace ly_auto_aim::inline solver
