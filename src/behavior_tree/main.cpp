/*
 * behavior_tree_node 入口
 *
 * 说明：
 * - 负责创建 Application 并进入决策主循环。
 * - BT 内部会等待比赛开始信号，再执行策略 tick。
 */
#include "include/Application.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) try {
    // 1. [ROS 2] 必須先初始化 ROS 上下文
    rclcpp::init(argc, argv);

    // 2. [ROS 2] 日誌打印
    // 由於此時還沒有 node 對象，我們創建一個名為 "main" 的臨時 logger
    RCLCPP_INFO(rclcpp::get_logger("main"), "main: running %s", BehaviorTree::Application::nodeName);

    // 3. 創建應用實例
    // 這會觸發 Application.cpp 的構造函數，完成所有初始化 (路徑、Logger、訂閱等)
    BehaviorTree::Application app(argc, argv);

    // 4. 運行主循環
    app.Run();

    // 5. [ROS 2] 清理資源
    rclcpp::shutdown();
    return 0;
}
catch (const std::exception &ex) {
    std::cerr << "[Exception] " << ex.what() << std::endl;
    // 確保發生異常時也能正確關閉 ROS
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 1;
}
