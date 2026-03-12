#pragma once
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <memory>
#include <functional>
#include <string>
#include <mutex>
#include <optional>

namespace topicIO {

// =========================
// BaseTopicIO（型消去用）
// =========================
class BaseTopicIO {
public:
    virtual ~BaseTopicIO() = default;
    virtual void init(rclcpp::Node* node) = 0;
    virtual std::string topic() const = 0;
};

// =========================
// TopicIO<MsgT>
// =========================
template<typename MsgT>
class TopicIO : public BaseTopicIO {
public:
    TopicIO(const std::string& topic,
            std::function<void(const MsgT&)> user_cb = nullptr)
        : topic_(topic), user_cb_(user_cb) {}

    void init(rclcpp::Node* node) override {
        pub_ = node->create_publisher<MsgT>(topic_, 10);

        sub_ = node->create_subscription<MsgT>(
            topic_, 10,
            [this](const MsgT& msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_ = msg;
                has_value_ = true;
                if (user_cb_) user_cb_(msg);
            }
        );
    }

    template<typename T = MsgT>
    void send(const T& msg) {
        pub_->publish(msg);
    }

    template<typename T = MsgT>
    std::optional<T> get() const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_value_) return std::nullopt;
        return latest_;
    }

    std::string topic() const override { return topic_; }

private:
    mutable std::mutex mtx_;
    std::string topic_;
    std::function<void(const MsgT&)> user_cb_;

    rclcpp::Publisher<MsgT>::SharedPtr pub_;
    rclcpp::Subscription<MsgT>::SharedPtr sub_;

    MsgT latest_;
    bool has_value_ = false;
};

// =========================
// IncludeClass（TopicManager）
// =========================
class IncludeClass {
protected:
    std::map<std::string, std::shared_ptr<BaseTopicIO>> topicMC;

    template<typename MsgT>
    void add_topicIO(const std::string& topic,
                     std::function<void(const MsgT&)> cb = nullptr)
    {
        topicMC[topic] = std::make_shared<TopicIO<MsgT>>(topic, cb);
    }

    void init_all(rclcpp::Node* node) {
        for (auto& [name, io] : topicMC) {
            io->init(node);
        }
    }
};

} // namespace topicIO