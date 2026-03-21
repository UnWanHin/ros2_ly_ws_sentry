// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <solver/solver.hpp>
#include <predictor/predictor.hpp>
#include <controller/controller.hpp>

namespace CONTROLLER
{
    class MuzzleSolver
    {
    public:
        MuzzleSolver(Eigen::Vector3d);
        ~MuzzleSolver();
        BoardInformations solveMuzzle(PREDICTOR::OutpostInformation &outpost_information);
        void setBulletSpeed(float bullet_speed)
        {
            this->bullet_speed = bullet_speed;
        }

    private:
        float bulletModel(float dis, float angle);
        Eigen::Vector3d gun_muzzle_offset;
        float bullet_speed;
        float time_delay;
    };
} // namespace CONTROLLER