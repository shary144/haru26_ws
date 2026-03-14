#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <functional>
#include <string>
#include <mutex>
#include <optional>

template<class MsgT>
class Topic {
public:
    Topic(rclcpp::Node* node, const std::string& name, size_t qos = 10)
    : node_(node), name_(name)
    {
        pub_ = node_->create_publisher<MsgT>(name_, qos);

        sub_ = node_->create_subscription<MsgT>(
            name_, qos,
            [this](const MsgT& msg){
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_ = msg;
                }
                for (auto& cb : listeners_) {
                    cb(msg);
                }
            }
        );
    }

    // ---- publish ----
    void publish(std::function<void(MsgT&)> f) {
        MsgT msg{};
        f(msg);
        pub_->publish(msg);
    }

    // ---- listener ----
    void listen(std::function<void(const MsgT&)> f) {
        listeners_.push_back(f);
    }

    // ---- latest を返す ----
    std::optional<MsgT> use_latest() {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;  // コピーされるので安全
    }

private:
    rclcpp::Node* node_;
    std::string name_;

    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
    typename rclcpp::Subscription<MsgT>::SharedPtr sub_;

    std::optional<MsgT> latest_;
    std::mutex mutex_;

    std::vector<std::function<void(const MsgT&)>> listeners_;
};