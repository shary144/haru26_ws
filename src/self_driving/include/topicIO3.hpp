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
// function_traits: auto& に対応したジェネリックラムダ対応版
// =======================

template<typename T>
struct function_traits;

// 通常のラムダ（非テンプレート operator()）
template<typename C, typename R, typename Arg>
struct function_traits<R(C::*)(Arg) const>
{
    using arg_type = std::decay_t<Arg>;
};

// ジェネリックラムダ（テンプレート operator()）
template<typename F>
struct function_traits
{
private:
    template<typename C, typename R, typename Arg>
    static Arg test(R(C::*)(Arg) const);

    template<typename C, typename R, typename Arg>
    static Arg test(R(C::*)(Arg));

    template<typename C, typename R, typename... Args>
    static void test(R(C::*)(Args...) const);

public:
    using arg_type = std::decay_t<decltype(test(&F::operator()))>;
};

// =======================
// 抽象基底: TopicConcept
// =======================

struct TopicConcept
{
    virtual ~TopicConcept() = default;

    virtual void send_erased(const void* msg) = 0;

    virtual void set_listener_erased(std::function<void(const void*)> cb) = 0;

    virtual void use_latest_msg_erased(std::function<void(const void*)> cb) = 0;
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

    void send_erased(const void* msg) override
    {
        const auto* typed = static_cast<const MsgT*>(msg);
        pub_->publish(*typed);
    }

    void set_listener_erased(std::function<void(const void*)> cb) override
    {
        sub_ = node_->create_subscription<MsgT>(
            topic_name_, qos_,
            [this, cb](const MsgT& msg)
            {
                {
                    std::lock_guard<std::mutex> lock(latest_mutex_);
                    latest_msg_ = msg;
                }
                cb(&msg);
            }
        );
    }

    void use_latest_msg_erased(std::function<void(const void*)> cb) override
    {
        std::optional<MsgT> copy;
        {
            std::lock_guard<std::mutex> lock(latest_mutex_);
            copy = latest_msg_;
        }
        cb(&copy);
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

    template<class F>
    void send_field(F&& f)
    {
        using MsgT = typename function_traits<F>::arg_type;
        MsgT msg{};
        f(msg);
        impl_->send_erased(&msg);
    }

    template<class F>
    void listener(F&& f)
    {
        using MsgT = typename function_traits<F>::arg_type;
        impl_->set_listener_erased(
            [f](const void* erased)
            {
                const auto* typed = static_cast<const MsgT*>(erased);
                f(*typed);
            }
        );
    }

    template<class F>
    void use_latest_msg(F&& f)
    {
        using OptT = typename function_traits<F>::arg_type;
        impl_->use_latest_msg_erased(
            [f](const void* erased)
            {
                const auto* opt = static_cast<const OptT*>(erased);
                f(*opt);
            }
        );
    }

private:
    TopicConcept* impl_;
};

// =======================
// TopicRegistry: トピック名 → 型消去モデル
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