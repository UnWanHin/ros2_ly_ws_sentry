// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <map>
#include <memory>
#include <functional>
#include <mutex>
#include <string>

#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

namespace LangYa::roslog
{
    // [不變] 獲取全局 Logger
    inline rclcpp::Logger get_logger() {
        static rclcpp::Logger logger = rclcpp::get_logger("LangYa");
        return logger;
    }

#define LY_DEF_ROS_LOG(name, rosName)\
    template<typename ...Args> \
    void name(const auto fmt, Args &&... args) \
    { \
        /* 保持 fmt::format 接口，內部轉為 C 字符串給 ROS 2 */ \
        RCLCPP_##rosName(get_logger(), "%s", fmt::format(fmt, std::forward<Args>(args)...).c_str()); \
    } \

    LY_DEF_ROS_LOG(error, ERROR);
    LY_DEF_ROS_LOG(warn, WARN);
    LY_DEF_ROS_LOG(info, INFO);
    LY_DEF_ROS_LOG(debug, DEBUG);
#undef LY_DEF_ROS_LOG
}

namespace LangYa
{
    template<typename TGlobal>
    class MultiCallback
    {
        std::mutex Mutex{};
        TGlobal Global;

    public:
        MultiCallback(TGlobal g) noexcept : Global(g) {}

        template<typename TTopic>
        std::function<void(typename TTopic::CallbackArg)> Generate(auto modifier) noexcept
        {
            return [this, modifier](typename TTopic::CallbackArg const& arg)
            {
                std::lock_guard lock{ Mutex };
                modifier(Global, arg);
            };
        }
    };

    template<const char* TName, typename TMessage>
    struct ROSTopic
    {
        static constexpr auto Name = TName;
        using Msg = TMessage;
        // ROS 2 習慣用 SharedPtr，這可能需要你在回調函數參數裡微調一下 (ConstPtr -> SharedPtr)
        // 但調用邏輯基本兼容
        using CallbackArg = typename Msg::SharedPtr; 
    };

#define LY_DEF_ROS_TOPIC(varname, topic, type)\
    inline static constexpr const char varname##_topic_name[] = topic;\
    using varname = LangYa::ROSTopic<varname##_topic_name, type>;

    template<const char* TName>
    class ROSNode
    {
        std::shared_ptr<rclcpp::Node> NodePtr{};

        // 內部依然用指針存儲（為了多態）
        std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> Publishers{};
        std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> Subscribers{};

    public:
        ROSNode() noexcept = default;

        bool Initialize(int argc, char** argv) noexcept try
        {
            if (!rclcpp::ok()) {
                rclcpp::init(argc, argv);
            }
            NodePtr = std::make_shared<rclcpp::Node>(TName);
            roslog::info("Node [{}] Initialized", TName);
            return true;
        }
        catch (const std::exception& ex)
        {
            roslog::error("ROSNode::Initialize: {}", ex.what());
            return false;
        }
        
        // 獲取原始節點指針 (新增接口，不影響舊代碼)
        std::shared_ptr<rclcpp::Node> GetNode() const { return NodePtr; }

        // [核心修改] 返回引用 (&)，完美保留接口格式！
        template<typename TTopic>
        rclcpp::Publisher<typename TTopic::Msg>& Publisher()
        {
            using MsgType = typename TTopic::Msg;
            std::string name{ TTopic::Name };

            // 1. 如果已存在，取出 -> 轉型 -> 解引用
            if (Publishers.contains(name)) {
                auto base_ptr = Publishers[name];
                auto concrete_ptr = std::dynamic_pointer_cast<rclcpp::Publisher<MsgType>>(base_ptr);
                return *concrete_ptr; // 返回引用，用戶可以用 .publish()
            }

            // 2. 如果不存在，創建 -> 存入 -> 解引用
            auto pub = NodePtr->template create_publisher<MsgType>(name, 3);
            Publishers.insert({ name, pub });
            return *pub; // 返回引用
        }

        template<typename TTopic, typename TFunc>
        void GenSubscriber(TFunc&& callback)
        {
            using MsgType = typename TTopic::Msg;
            std::string name{ TTopic::Name };
            
            if (Subscribers.contains(name)) return;

            auto sub = NodePtr->template create_subscription<MsgType>(
                name, 
                rclcpp::QoS(2),
                std::forward<TFunc>(callback)
            );

            Subscribers.insert({ name, sub });
        }

        // 手動觸發回調 (ROS 2 需要)
        void SpinSome() {
            if(NodePtr) rclcpp::spin_some(NodePtr);
        }
    };
}


//拿數據的接口?
