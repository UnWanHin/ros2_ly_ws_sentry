// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "controller/BoardSelector.hpp"

namespace CONTROLLER
{
    BoardSelector::BoardSelector()
    {
        // Constructor implementation
    }

    BoardSelector::~BoardSelector()
    {
        // Destructor implementation
    }

    BoardInformation BoardSelector::selectBestBoard(BoardInformations &board_info)
    {
        // // Implement the logic to select the best board based on the board information and IMU flag.
        // BoardInformation best_board;
        // double min_cos = M_PI/2;
        // // 遍历board_info，返回face_cos最小的板子
        // for (auto &board : board_info)
        // {
        //     if (board.face_cos < min_cos)
        //     {
        //         best_board = board;
        //         min_cos = board.face_cos;
        //     }
        // }

        // if(board_info[0].face_cos < M_PI/4){
        //     return board_info[0];
        // }else{
        //     return board_info[1];
        // }
        // return best_board;
        return board_info[0];
    }

} // namespace CONTROLLER