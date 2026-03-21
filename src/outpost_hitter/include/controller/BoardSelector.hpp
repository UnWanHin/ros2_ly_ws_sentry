// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <vector>
#include <deque>
#include <controller/controller.hpp>

namespace CONTROLLER
{
    class BoardSelector
    {
    public:
        BoardSelector();
        ~BoardSelector();

        BoardInformation selectBestBoard(BoardInformations &board_info);

    private:
    };
} // namespace CONTROLLER