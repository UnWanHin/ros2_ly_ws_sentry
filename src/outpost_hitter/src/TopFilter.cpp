// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "predictor/TopFilter.hpp"

using namespace SOLVER;
namespace PREDICTOR
{
    TopFilter::TopFilter()
    {
    }

    TopFilter::~TopFilter()
    {
    }

    ArmorPoses TopFilter::filterTopArmor(const ArmorPoses &armor_poses)
    {
        return armor_poses;
    }
} // namespace PREDICTOR