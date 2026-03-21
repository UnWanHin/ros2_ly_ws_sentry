// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include "solver/solver.hpp"

namespace PREDICTOR
{
    class TopFilter
    {
    public:
        TopFilter();
        ~TopFilter();

        SOLVER::ArmorPoses filterTopArmor(const SOLVER::ArmorPoses &armor_poses);

    private:
    };
} // namespace PREDICTOR