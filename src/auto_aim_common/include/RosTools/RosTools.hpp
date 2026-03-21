// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include "Logger/Logger.hpp"

namespace LangYa {

template<typename TGlobal>
class MultiCallback {
    std::shared_ptr<TGlobal> global_;
    std::mutex mutex_;

public:
    explicit MultiCallback(std::shared_ptr<TGlobal> global)
        : global_(std::move(global)) {}

    template<typename TTopic>
    auto Generate(auto modifier) noexcept {
        return [this, modifier](const typename TTopic::Msg& msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            modifier(*global_, msg);
        };
    }
};

// -------------------------------------------------------------
// [修改 1] 將模板參數從數組引用改為 const char*
// 這樣可以兼容任意長度的字串，不用改接口
// -------------------------------------------------------------
template<const char* TName, typename TMsg>
struct ROSTopic {
    static constexpr const char* TopicName = TName;
    using Msg = TMsg;
};


#define LY_DEF_ROS_TOPIC(name, str, type) \
    inline constexpr char name##_topic_name[] = str; \
    using name = LangYa::ROSTopic<name##_topic_name, type>;

template<const char* TName>
class ROSNode : public rclcpp::Node {
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> pubs_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subs_;
    std::mutex pub_mutex_, sub_mutex_;

public:

    ROSNode()
        : rclcpp::Node(
              TName,
              rclcpp::NodeOptions()
                  .allow_undeclared_parameters(true)
                  .automatically_declare_parameters_from_overrides(true)) {
        // 保留了 roslog，沒有改動
        roslog::info("ROSNode [{}] created", TName);
    }

    template<typename TTopic>
    auto Publisher() {
        const std::string topic_name = TTopic::TopicName;
        std::lock_guard<std::mutex> lock(pub_mutex_);
        if (pubs_.count(topic_name) == 0) {
            pubs_[topic_name] = this->create_publisher<typename TTopic::Msg>(topic_name, 10);
        }
        return std::static_pointer_cast<rclcpp::Publisher<typename TTopic::Msg>>(pubs_[topic_name]);
    }

    template<typename TTopic, typename F>
    void GenSubscriber(F&& cb) {
        const std::string topic_name = TTopic::TopicName;
        std::lock_guard<std::mutex> lock(sub_mutex_);
        if (subs_.count(topic_name) == 0) {
            subs_[topic_name] = this->create_subscription<typename TTopic::Msg>(
                topic_name, 10, std::forward<F>(cb));
        }
    }

    template<typename T>
    bool GetParam(const std::string& name, T& value, const T& default_value = T{}) {
        if (this->has_parameter(name)) {
            this->get_parameter(name, value);
            return true;
        }
        // 為了防止參數未聲明報錯，加個 try-catch 或者保留你原本的邏輯
        // 這裡保持你原本的邏輯
        this->declare_parameter(name, default_value);
        this->get_parameter(name, value);
        return true;
    }
};

} // namespace LangYa
