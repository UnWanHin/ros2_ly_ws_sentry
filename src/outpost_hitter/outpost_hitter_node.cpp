// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

// 改用SOLVER::global_solver_node = my_node_ptr;
/*
 * outpost_hitter_node（打前哨）
 *
 * 功能：
 * - 输入 /ly/outpost/armors 检测结果
 * - 执行姿态估计、方向判定与弹道求解
 * - 输出 /ly/outpost/target（供上层决策或外部桥接）
 */

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <initializer_list>

// [ROS 2] 引入你的工具庫和消息頭文件
#include "RosTools/RosTools.hpp"
#include "solver/PoseSolver.hpp"
#include <auto_aim_common/DetectionType.hpp>
#include <auto_aim_common/msg/target.hpp>
#include <auto_aim_common/msg/armor.hpp>
#include <auto_aim_common/msg/armors.hpp>
#include <std_msgs/msg/bool.hpp>

// [引入你的模塊] 確保這些頭文件路徑正確
#include "solver/solver.hpp"
#include "detector/detector.hpp"
#include "predictor/predictor.hpp"
#include "predictor/OutpostPredictor.hpp"
#include "predictor/DerectionJudger.hpp"
#include "predictor/TopFilter.hpp"
#include "controller/MuzzleSolver.hpp"
#include "controller/BoardSelector.hpp"
#include "utils/utils.h"

using namespace LangYa;
using namespace LY_UTILS;
using namespace ly_auto_aim;
using namespace std;

#define MAX_LOSS_COUNT 10
#define USE_MEASURE_UPDATE true
#define USE_ITERATION_UPDATE false

namespace {
    // [ROS 2] 修正 Topic 定義: 加上 ::msg::
    LY_DEF_ROS_TOPIC(ly_outpost_armors, "/ly/outpost/armors", auto_aim_common::msg::Armors);
    LY_DEF_ROS_TOPIC(ly_outpost_enable, "/ly/outpost/enable", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_outpost_target, "/ly/outpost/target", auto_aim_common::msg::Target);

    constexpr const char AppName[] = "outpost_hitter_node";

    class OutpostHitterNode
    {
    public:
        // [ROS 2] 構造函數修改：移除 argc/argv，因為初始化在 main 裡做
        OutpostHitterNode() : node() 
        {
            // [ROS 2] node.Initialize 已經不需要了，刪除

            // [ROS 2] 修正訂閱者回調
            // 參數類型明確為 ConstSharedPtr (不帶 &)
            node.GenSubscriber<ly_outpost_armors>([this](const auto_aim_common::msg::Armors::ConstSharedPtr msg) { 
                outpost_detection_callback(msg); 
            });

            // 如果需要 enable 回调，可以解开注释
            // node.GenSubscriber<ly_outpost_enable>([this](const std_msgs::msg::Bool::ConstSharedPtr msg) { outpost_enable_callback(msg); });

            // 初始化其餘模塊（PoseSolver 需要在 main 設置 global_solver_node 後再初始化）
            outpost_predictor = std::make_unique<PREDICTOR::OutpostPredictor>();
            derection_judger = std::make_unique<PREDICTOR::DirectionJudger>();
            top_filter = std::make_unique<PREDICTOR::TopFilter>();
            muzzle_solver = std::make_unique<CONTROLLER::MuzzleSolver>(Eigen::Vector3d(0.0, 0.0, 0.0));
            board_selector = std::make_unique<CONTROLLER::BoardSelector>();
            loadOutpostConfig();

            // 初始化時間戳
            last_time_stamp = node.now();

            RCLCPP_INFO(node.get_logger(), "OutpostHitterNode> Initialized base modules.");
        }

        ~OutpostHitterNode() = default;

        bool InitializeSolver() {
            if (solver) return true;
            try {
                solver = std::make_unique<SOLVER::PoseSolver>();
                RCLCPP_INFO(node.get_logger(), "OutpostHitterNode> PoseSolver initialized.");
                return true;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node.get_logger(), "OutpostHitterNode> PoseSolver init failed: %s", e.what());
                return false;
            }
        }

        template<typename T>
        bool tryGetParamCompat(const std::initializer_list<const char*>& keys, T& value) {
            for (const auto* key : keys) {
                if (node.has_parameter(key) && node.get_parameter(key, value)) {
                    return true;
                }
            }
            return false;
        }

        template<typename T>
        void loadParamCompat(const std::initializer_list<const char*>& keys, T& value, const T& default_value) {
            if (tryGetParamCompat(keys, value)) {
                return;
            }
            const auto* primary = *keys.begin();
            if (!node.has_parameter(primary)) {
                node.declare_parameter<T>(primary, default_value);
            }
            node.get_parameter(primary, value);
        }

        void loadOutpostConfig() {
            loadParamCompat<double>(
                {"outpost_config.fire.bullet_speed", "outpost_config/fire/bullet_speed"},
                outpost_bullet_speed_,
                23.0);
            loadParamCompat<double>(
                {"outpost_config.fire.pitch_bias_deg", "outpost_config/fire/pitch_bias_deg"},
                outpost_pitch_bias_deg_,
                2.0);
            loadParamCompat<double>(
                {"outpost_config.fire.max_yaw_delta_deg", "outpost_config/fire/max_yaw_delta_deg"},
                outpost_max_yaw_delta_deg_,
                80.0);
            loadParamCompat<double>(
                {"outpost_config.fire.time_delay_sec", "outpost_config/fire/time_delay_sec"},
                outpost_time_delay_sec_,
                0.10);

            if (!std::isfinite(outpost_bullet_speed_) || outpost_bullet_speed_ <= 0.0) {
                outpost_bullet_speed_ = 23.0;
            }
            if (!std::isfinite(outpost_pitch_bias_deg_)) {
                outpost_pitch_bias_deg_ = 2.0;
            }
            if (!std::isfinite(outpost_max_yaw_delta_deg_) || outpost_max_yaw_delta_deg_ <= 0.0) {
                outpost_max_yaw_delta_deg_ = 80.0;
            }
            if (!std::isfinite(outpost_time_delay_sec_) || outpost_time_delay_sec_ < 0.0) {
                outpost_time_delay_sec_ = 0.10;
            }

            muzzle_solver->setBulletSpeed(static_cast<float>(outpost_bullet_speed_));
            muzzle_solver->setTimeDelay(static_cast<float>(outpost_time_delay_sec_));
            RCLCPP_INFO(
                node.get_logger(),
                "Outpost fire config: bullet_speed=%.3f pitch_bias_deg=%.3f max_yaw_delta_deg=%.3f time_delay_sec=%.3f",
                outpost_bullet_speed_,
                outpost_pitch_bias_deg_,
                outpost_max_yaw_delta_deg_,
                outpost_time_delay_sec_);
        }

        // [ROS 2] Callback 修正：類型為 ConstSharedPtr
        void outpost_detection_callback(const auto_aim_common::msg::Armors::ConstSharedPtr msg) {
            // if(!outpost_enable.load()) return;

            if (!solver) {
                RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000, "OutpostHitterNode> PoseSolver is not ready, skip frame.");
                return;
            }

            RCLCPP_DEBUG(node.get_logger(), "OutpostHitterNode> Received outpost detection message.");

            // ** 步驟一：detection的獲取 **
            rclcpp::Time stamp = msg->header.stamp;
            float yaw_now = msg->yaw;
            float pitch_now = msg->pitch; 
            /// debug (已注釋，使用真實雲台角度)
            // yaw_now = 1000.0f;
            // pitch_now = 10.0f;

            Detections detections;
            convertToDetections(msg, detections);
            RCLCPP_DEBUG(node.get_logger(), "OutpostHitterNode> Corners converted.");
            
            if(detections.empty()) {
                RCLCPP_DEBUG_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "OutpostHitterNode> outpost not found.");
                return;
            }

            // ** 步驟二: 解算 **
            SOLVER::ArmorPoses armor_poses = solver->solveArmorPoses(detections, yaw_now, pitch_now);
            RCLCPP_DEBUG(node.get_logger(), "OutpostHitterNode> Armor poses solved.");

            // ** 步驟三： 預測 **
            /// 取出最近的裝甲板
            auto min_pitch_armor = std::min_element(armor_poses.begin(), armor_poses.end(),[](const SOLVER::ArmorPose &a, const SOLVER::ArmorPose &b)
                                                    { return a.pyd.distance < b.pyd.distance; });

            if (min_pitch_armor == armor_poses.end()) {
                RCLCPP_DEBUG_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "OutpostHitterNode> No valid armor poses found.");
                return;
            }
            PREDICTOR::OutpostInformation outpost_info;
            /// 判斷方向
            if(!derection_judger->isDerectionJudged()){
                RCLCPP_DEBUG_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "OutpostHitterNode> Direction judging.");
                /// 更新世界坐標系下的yaw角和距離
                derection_judger->updateWorldPYD(min_pitch_armor->pyd.pitch, min_pitch_armor->pyd.yaw, min_pitch_armor->pyd.distance);
            }
            if(derection_judger->isDerectionJudged()){
                RCLCPP_DEBUG(node.get_logger(), "OutpostHitterNode> Direction judged.");
                if(!outpost_predictor->is_initialized) { // 初始化
                    RCLCPP_INFO(node.get_logger(), "OutpostHitterNode> Initializing outpost predictor.");
                    outpost_predictor->initPredictor(*(min_pitch_armor), derection_judger->getDirection());
                    last_time_stamp = stamp;
                } else { // 量測更新
                    RCLCPP_DEBUG(node.get_logger(), "OutpostHitterNode> Running outpost predictor.");
                    
                    // [修復] seconds() 返回 double，cast 成 int 會截斷幀間隔（~13ms → 0）
                    // 改用毫秒整數，符合 OutpostPredictor::runPredictor 的 dt 單位
                    int delta_time = static_cast<int>((stamp - last_time_stamp).seconds() * 1000.0);
                    last_time_stamp = stamp;
                    outpost_info = outpost_predictor->runPredictor(delta_time, *(min_pitch_armor), USE_MEASURE_UPDATE);

                    RCLCPP_DEBUG(node.get_logger(), "OutpostHitterNode> outpost_theta: %.4f, outpost_omega: %.4f", outpost_info.outpost_theta, outpost_info.outpost_omega);
                    
                    if (!outpost_info.is_valid) {
                        RCLCPP_DEBUG_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "OutpostHitterNode> Outpost info is not valid.");
                        return;
                    }
                }
            } else {
                RCLCPP_DEBUG_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "OutpostHitterNode> Direction judging...");
                return;
            }

            // ** 步驟四：計算角度 **
            CONTROLLER::BoardInformations board_info = muzzle_solver->solveMuzzle(outpost_info);

            if(board_info.size()==0) return ;

            auto best_board = board_selector->selectBestBoard(board_info);
            double pitch_setpoint = best_board.aim_pitch;
            pitch_setpoint *= 180 / M_PI;
            pitch_setpoint -= outpost_pitch_bias_deg_;
            double yaw_setpoint = best_board.aim_yaw;
            yaw_setpoint *= 180 / M_PI;
            yaw_setpoint = (float)(yaw_setpoint + std::round((yaw_now - yaw_setpoint) / 360.0) * 360.0);
            if (std::abs(yaw_setpoint - yaw_now) > outpost_max_yaw_delta_deg_) {
                // roslog::warn("OutpostHitterNode> Yaw setpoint is too far from current yaw, skipping.");
                return;
            }

            // ** 步驟五: 發布消息 **
            auto_aim_common::msg::Target target_msg;
            target_msg.header.stamp = stamp;
            target_msg.status = true;
            target_msg.buff_follow = false;
            target_msg.yaw = yaw_setpoint;
            target_msg.pitch = pitch_setpoint;
            
            // [ROS 2] 修正 publish 調用方式: 指針 -> 用箭頭
            node.Publisher<ly_outpost_target>()->publish(target_msg);
        }

        // [關鍵] 公開 node 成員，以便 main 函數可以獲取它來進行 spin
        // 因為 LangYa::ROSNode 繼承自 rclcpp::Node
        LangYa::ROSNode<AppName> node;

    private:
        // [ROS 2] 修正參數類型為 ConstSharedPtr
        void convertToDetections(const auto_aim_common::msg::Armors::ConstSharedPtr msg, Detections& detections) {
            detections.clear();
            detections.reserve(msg->armors.size());
            for (const auto& armor : msg->armors) {
                if(armor.type != 7) continue; /// 需要判斷這個是否已經被過濾了
                /// 需要進一步判斷是否對應的上，感覺對不上的樣子
                double angle1 = atan2(armor.corners_y[3] - armor.corners_y[1], armor.corners_x[0] - armor.corners_x[3]);
                double angle2 = atan2(armor.corners_y[2] - armor.corners_y[0], armor.corners_x[1] - armor.corners_x[2]);
                if(abs(angle1 - M_PI/2) >= M_PI/6 || abs(angle2 - M_PI/2) >= M_PI/6) continue;
                detections.emplace_back(Detection{
                    .tag_id = armor.type,  // C++20 
                    .corners = {
                        {armor.corners_x[0], armor.corners_y[0]},
                        {armor.corners_x[1], armor.corners_y[1]},
                        {armor.corners_x[2], armor.corners_y[2]},
                        {armor.corners_x[3], armor.corners_y[3]}
                    }
                });
            }
        }

        std::unique_ptr<SOLVER::PoseSolver> solver;
        std::unique_ptr<PREDICTOR::OutpostPredictor> outpost_predictor;
        std::unique_ptr<PREDICTOR::DirectionJudger> derection_judger;
        std::unique_ptr<PREDICTOR::TopFilter> top_filter;
        std::unique_ptr<CONTROLLER::MuzzleSolver> muzzle_solver;
        std::unique_ptr<CONTROLLER::BoardSelector> board_selector;
        rclcpp::Time last_time_stamp;
        double outpost_bullet_speed_{23.0};
        double outpost_pitch_bias_deg_{2.0};
        double outpost_max_yaw_delta_deg_{80.0};
        double outpost_time_delay_sec_{0.10};
    };
} // namespace

int main(int argc, char** argv){
    // 1. 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 2. 創建我們封裝的業務類
    // 注意：OutpostHitterNode 的構造函數裡初始化了 LangYa::ROSNode (繼承自 rclcpp::Node)
    auto app = std::make_shared<OutpostHitterNode>();

    // 3. [關鍵] 獲取底層的 rclcpp::Node 指針
    // 這樣我們可以把它傳給 SOLVER 全局變量，也可以用來 spin
    // 這裡使用 "空刪除器 (Aliasing Constructor)" 技巧，創建一個指向成員變量的 shared_ptr
    std::shared_ptr<rclcpp::Node> node_ptr(&app->node, [](auto*){});

    // 4. [必做] 設置 SOLVER 的全局指針
    // 確保 PoseSolver::PoseSolver() 能拿到節點
    SOLVER::global_solver_node = node_ptr;

    // 5. 在全局節點就緒後再初始化 PoseSolver，避免啟動時序崩潰
    if (!app->InitializeSolver()) {
        RCLCPP_FATAL(node_ptr->get_logger(), "OutpostHitterNode> Fatal: PoseSolver initialization failed.");
        rclcpp::shutdown();
        return 1;
    }

    // 6. 運行
    rclcpp::spin(node_ptr);
    
    rclcpp::shutdown();
    return 0;
}
