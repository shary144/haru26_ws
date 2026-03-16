#pragma once
#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <functional>
#include <mutex>

enum class TopicMode {
    Publisher,
    Subscriber
};

template<class MsgT>
class Topic {
public:
    Topic(rclcpp::Node* node,
          const std::string& name,
          TopicMode mode,
          size_t qos = 10)
        : node_(node), name_(name), mode_(mode)
    {
        if (mode_ == TopicMode::Publisher) {
            pub_ = node_->create_publisher<MsgT>(name_, qos);
        } else {
            sub_ = node_->create_subscription<MsgT>(
                name_, qos,
                [this](const typename MsgT::SharedPtr msg){
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        latest_ = *msg;  // コピー（MsgT が軽い前提）
                    }
                    if (sub_callback_) sub_callback_(*msg);
                }
            );
        }
    }

    // ---- publisher hook ----
    void hook(std::function<void(MsgT&)> f) {
        if (mode_ == TopicMode::Publisher)
            pub_callback_ = f;
    }

    // ---- subscriber hook ----
    void hook(std::function<void(const MsgT&)> f) {
        if (mode_ == TopicMode::Subscriber)
            sub_callback_ = f;
    }

    // ---- publish ----
    void publish() {
        if (mode_ != TopicMode::Publisher) return;
        MsgT msg{};
        if (pub_callback_) pub_callback_(msg);
        pub_->publish(msg);
    }

    // ---- latest ----
    std::optional<MsgT> get() {
        if (mode_ == TopicMode::Publisher) {
            return MsgT{};  // publisher は最新値を持たない
        }
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }

private:
    rclcpp::Node* node_;
    std::string name_;
    TopicMode mode_;

    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
    typename rclcpp::Subscription<MsgT>::SharedPtr sub_;

    std::function<void(MsgT&)> pub_callback_;
    std::function<void(const MsgT&)> sub_callback_;

    std::optional<MsgT> latest_;
    std::mutex mutex_;
};