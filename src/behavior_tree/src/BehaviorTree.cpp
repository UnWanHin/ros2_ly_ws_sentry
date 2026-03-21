// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include "../include/BTNodes.hpp"
#include <cstdlib>

using namespace LangYa;

namespace BehaviorTree {

namespace {
bool EnvEnabled(const char* key, bool default_value) {
    const char* v = std::getenv(key);
    if (!v || !*v) return default_value;
    const std::string s(v);
    return s == "1" || s == "true" || s == "TRUE" || s == "on" || s == "ON";
}
} // namespace

bool Application::LoadBehaviorTree() noexcept {
    try {
        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }
        if (!TickBlackboard_) {
            TickBlackboard_ = BT::Blackboard::create();
        }
        GlobalBlackboard_->set("TickBlackboard", TickBlackboard_);

        BTree = Factory.createTreeFromFile(behavior_tree_file_, GlobalBlackboard_);

        // 稳定性优先：默认关闭 BT 调试附加器，避免在实机环境引入额外崩溃面。
        // 如需开启，导出环境变量：
        //   BT_ENABLE_FILE_LOGGER=1
        //   BT_ENABLE_GROOT=1
        if (EnvEnabled("BT_ENABLE_FILE_LOGGER", false)) {
            try {
                btFileLogger_ = std::make_unique<BT::FileLogger2>(BTree, "behavior_tree_trace.fbl");
                LoggerPtr->Info("BT file logger enabled.");
            } catch (const std::exception& ex) {
                LoggerPtr->Warning("BT file logger init failed: {}", ex.what());
            }
        }
        if (EnvEnabled("BT_ENABLE_GROOT", false)) {
            try {
                btGrootPublisher_ = std::make_unique<BT::Groot2Publisher>(BTree, 1667);
                LoggerPtr->Info("BT Groot publisher enabled on port 1667.");
            } catch (const std::exception& ex) {
                LoggerPtr->Warning("BT Groot publisher init failed: {}", ex.what());
            }
        }

        return true;
    }
    catch (const std::exception& ex) {
        LoggerPtr->Error("BehaviorTree Script Load Failed, location: {}", behavior_tree_file_);
        LoggerPtr->Error("Error Message: {}", ex.what());
        return false;
    }
}

/**
 * @brief 注册行为树节点 \n
 */
bool Application::RegisterTreeNodes() {
    try {
#define REGISTER_APP_NODE(NodeType, NodeID)                                                 \
    Factory.registerBuilder<NodeType>(                                                      \
        NodeID,                                                                             \
        [this](const std::string& name, const BT::NodeConfig& config) {                    \
            return std::make_unique<NodeType>(name, config, this);                         \
        })

        REGISTER_APP_NODE(UpdateGlobalData, "UpdateGlobalData");
        REGISTER_APP_NODE(SelectAimModeNode, "SelectAimMode");
        REGISTER_APP_NODE(SelectStrategyModeNode, "SelectStrategyMode");
        REGISTER_APP_NODE(CheckNeedRecoveryNode, "CheckNeedRecovery");
        REGISTER_APP_NODE(IsAimModeBuffNode, "IsAimModeBuff");
        REGISTER_APP_NODE(IsAimModeOutpostNode, "IsAimModeOutpost");
        REGISTER_APP_NODE(IsStrategyHitSentryNode, "IsStrategyHitSentry");
        REGISTER_APP_NODE(IsStrategyHitHeroNode, "IsStrategyHitHero");
        REGISTER_APP_NODE(IsStrategyProtectedNode, "IsStrategyProtected");
        REGISTER_APP_NODE(IsStrategyNaviTestNode, "IsStrategyNaviTest");
        REGISTER_APP_NODE(IsStrategyLeagueSimpleNode, "IsStrategyLeagueSimple");
        REGISTER_APP_NODE(ExecuteHitSentryStrategyNode, "ExecuteHitSentryStrategy");
        REGISTER_APP_NODE(ExecuteHitHeroStrategyNode, "ExecuteHitHeroStrategy");
        REGISTER_APP_NODE(ExecuteProtectedStrategyNode, "ExecuteProtectedStrategy");
        REGISTER_APP_NODE(ExecuteNaviTestStrategyNode, "ExecuteNaviTestStrategy");
        REGISTER_APP_NODE(ExecuteLeagueSimpleStrategyNode, "ExecuteLeagueSimpleStrategy");
        REGISTER_APP_NODE(PreprocessDataNode, "PreprocessData");
        REGISTER_APP_NODE(SelectAimTargetNode, "SelectAimTarget");
        REGISTER_APP_NODE(SelectPostureNode, "SelectPosture");
        REGISTER_APP_NODE(PublishAllNode, "PublishAll");

#undef REGISTER_APP_NODE

        // 保留旧节点注册，保证旧 XML 不会因为重构直接失效
        Factory.registerNodeType<SetAimTargetFromAim>("SetAimTargetFromAim");
        Factory.registerNodeType<SetNaviPosition>("SetNaviPosition");

        return true;
    }
    catch (const std::exception& ex) {
        LoggerPtr->Error("RegisterTreeNodes Failed: {}", ex.what());
        return false;
    }
}

} // namespace BehaviorTree
