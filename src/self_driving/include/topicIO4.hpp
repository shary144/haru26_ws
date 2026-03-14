#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <optional>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

namespace topic_registry
{

// =======================
// 抽象基底: TopicConcept
// =======================

struct TopicConcept
{
    virtual ~TopicConcept() = default;

    virtual void send_field_erased(std::function<void(void*)> f) = 0;

    virtual void set_listener_erased(std::function<void(void*)> f) = 0;

    virtual void use_latest_msg_erased(std::function<void(void*)> f) = 0;
};

// =======================
// 具象: TopicModel<MsgT>
// =======================

template<class MsgT>
class TopicModel : public TopicConcept
{
public:
    TopicModel(const rclcpp::Node::SharedPtr& node,
               const std::string& topic_name,
               const rclcpp::QoS& qos = rclcpp::QoS(10))
    : node_(node),
      topic_name_(topic_name),
      qos_(qos)
    {
        pub_ = node_->create_publisher<MsgT>(topic_name_, qos_);
    }

    // ---- send_field ----
    void send_field_erased(std::function<void(void*)> f) override
    {
        MsgT msg{};
        f(static_cast<void*>(&msg));   // auto& msg に入る
        pub_->publish(msg);
    }

    // ---- listener ----
    void set_listener_erased(std::function<void(void*)> f) override
    {
        sub_ = node_->create_subscription<MsgT>(
            topic_name_, qos_,
            [this, f](const MsgT& msg)
            {
                {
                    std::lock_guard<std::mutex> lock(latest_mutex_);
                    latest_msg_ = msg;
                }
                f((void*)&msg);   // auto& msg に入る
            }
        );
    }

    // ---- use_latest_msg ----
    void use_latest_msg_erased(std::function<void(void*)> f) override
    {
        std::optional<MsgT> copy;
        {
            std::lock_guard<std::mutex> lock(latest_mutex_);
            copy = latest_msg_;
        }
        f((void*)&copy);   // auto& opt に入る
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string topic_name_;
    rclcpp::QoS qos_;

    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
    typename rclcpp::Subscription<MsgT>::SharedPtr sub_;

    std::optional<MsgT> latest_msg_;
    std::mutex latest_mutex_;
};

// =======================
// TopicHandle: ユーザが触るハンドル
// =======================

class TopicHandle
{
public:
    explicit TopicHandle(TopicConcept* impl)
    : impl_(impl)
    {}

    // ---- send_field ----
    template<class F>
    void send_field(F&& f)
    {
        impl_->send_field_erased(
            [f](void* erased){
                f(*static_cast<decltype(auto)>(erased));  
            }
        );
    }

    // ---- listener ----
    template<class F>
    void listener(F&& f)
    {
        impl_->set_listener_erased(
            [f](void* erased){
                f(*static_cast<decltype(auto)>(erased));
            }
        );
    }

    // ---- use_latest_msg ----
    template<class F>
    void use_latest_msg(F&& f)
    {
        impl_->use_latest_msg_erased(
            [f](void* erased){
                f(*static_cast<decltype(auto)>(erased));
            }
        );
    }

private:
    TopicConcept* impl_;
};

// =======================
// TopicRegistry
// =======================

class TopicRegistry
{
public:
    explicit TopicRegistry(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    {}

    template<class MsgT>
    void declare(const std::string& topic_name,
                 const rclcpp::QoS& qos = rclcpp::QoS(10))
    {
        topics_[topic_name] =
            std::make_unique<TopicModel<MsgT>>(node_, topic_name, qos);
    }

    TopicHandle operator[](const std::string& topic_name)
    {
        return TopicHandle(topics_.at(topic_name).get());
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::unordered_map<std::string, std::unique_ptr<TopicConcept>> topics_;
};

} // namespace topic_registry