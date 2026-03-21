// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.


namespace ly_auto_aim{
    struct CameraIntrinsicsParameterPack
    {
        float FocalLength[2]{1331.1f, 1330.1f};
        float PrincipalPoint[2]{624.5817f, 526.3662f};
        float RadialDistortion[3]{-0.1059f, -0.3427f, 1.4125f};
        float TangentialDistortion[2]{0.0072f, 0};

        void GetCameraMatrix(cv::Mat& matrix) const;

        void GetDistortionCoefficients(cv::Mat& matrix) const;
    };
}