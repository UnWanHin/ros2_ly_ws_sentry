// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include "../Logger/Logger.hpp"

namespace LangYa {

template<typename TGlobal>
class MultiCallback {
    std::function<void(const TGlobal&)> PostEvent;
    mutable std::mutex Mutex{};
    TGlobal Global{};

public:
    explicit MultiCallback(std::function<void(const TGlobal&)> postEvent = nullptr)
        : PostEvent(std::move(postEvent)) {}

    template<typename TTopic>
    auto Generate(auto modifier) noexcept {
        return [this, modifier](const typename TTopic::Msg& arg) {
            std::lock_guard lock{Mutex};
            modifier(Global, arg);
            if (PostEvent) PostEvent(Global);
        };
    }

    TGlobal GetCopy() const noexcept {
        std::lock_guard lock{Mutex};
        return Global;
    }

    void SetPostEvent(std::function<void(const TGlobal&)> cb) { PostEvent = std::move(cb); }
};

template<const char* TName, typename TMessage>
struct ROSTopic {
    static constexpr const char* TopicName = TName;  // 自動轉換為 const char*
    using Msg = TMessage;
};

#define LY_DEF_ROS_TOPIC(varname, topic_str, type) \
    inline constexpr char varname##_topic_name[] = topic_str; \
    using varname = LangYa::ROSTopic<varname##_topic_name, type>;

template<const char* TName>
class ROSNode : public rclcpp::Node {
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> Publishers;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> Subscribers;
    std::mutex pub_mutex, sub_mutex;
public:
    ROSNode() : rclcpp::Node(TName) {
        roslog::info("ROSNode [{}] created", TName);
    }
    template<typename TTopic>
    auto Publisher() {
        const std::string n = TTopic::TopicName;
        std::lock_guard l(pub_mutex);
        if (!Publishers.count(n))
            Publishers[n] = this->create_publisher<typename TTopic::Msg>(n, 10);
        return std::static_pointer_cast<rclcpp::Publisher<typename TTopic::Msg>>(Publishers[n]);
    }
    template<typename TTopic, typename F>
    void GenSubscriber(F&& cb) {
        const std::string n = TTopic::TopicName;
        std::lock_guard l(sub_mutex);
        if (Subscribers.count(n)) return;
        Subscribers[n] = this->create_subscription<typename TTopic::Msg>(n, 10, std::forward<F>(cb));
    }
    template<typename T>
    bool GetParam(const std::string& n, T& v, const T& d = T{}) {
        if (this->has_parameter(n)) return this->get_parameter(n, v);
        this->declare_parameter(n, d);
        return this->get_parameter(n, v);
    }
};

} // namespace LangYa