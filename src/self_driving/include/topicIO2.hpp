#pragma once
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <memory>
#include <functional>
#include <string>
#include <mutex>
#include <optional>
#include <typeinfo>

namespace topicIO {

// =========================
// BaseTopicIO（型消去の箱）
// =========================
class BaseTopicIO {
public:
    virtual ~BaseTopicIO() = default;

    virtual void init(rclcpp::Node* node) = 0;
    virtual std::string topic() const = 0;

    virtual const std::type_info& type() const = 0;
};

// =========================
// TopicIO<MsgT>（実処理）
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

    void send(const MsgT& msg) {
        pub_->publish(msg);
    }

    template<typename F>
    void send(F&& builder) {
        MsgT msg{};
        builder(msg);
        pub_->publish(msg);
    }

    std::optional<MsgT> get() const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_value_) return std::nullopt;
        return latest_;
    }

    std::string topic() const override { return topic_; }
    const std::type_info& type() const override { return typeid(MsgT); }

private:
    mutable std::mutex mtx_;
    std::string topic_;
    std::function<void(const MsgT&)> user_cb_;

    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
    typename rclcpp::Subscription<MsgT>::SharedPtr sub_;

    MsgT latest_;
    bool has_value_ = false;
};

// =========================
// TopicNode（Node + Manager）
// =========================
class TopicNode : public rclcpp::Node {
public:
    TopicNode(const std::string& name)
        : rclcpp::Node(name) {}

    // ---- 登録 ----
    template<typename MsgT>
    void add_topic(const std::string& topic,
                   std::function<void(const MsgT&)> cb = nullptr)
    {
        topicMC_[topic] = std::make_shared<TopicIO<MsgT>>(topic, cb);
    }

    // ---- 初期化 ----
    void init_topics() {
        for (auto& [name, io] : topicMC_) {
            io->init(this);
        }
    }

    // ---- 型安全 send ----
    template<typename MsgT>
    void send(const std::string& topic, const MsgT& msg) {
        auto ptr = get_ptr<MsgT>(topic);
        ptr->send(msg);
    }
    template<typename MsgT, typename F>
    void send(const std::string& topic, F&& builder) {
        auto ptr = get_ptr<MsgT>(topic);
        ptr->send(std::forward<F>(builder));
    }

    // ---- 型安全 get ----
    template<typename MsgT>
    std::optional<MsgT> get(const std::string& topic) {
        auto ptr = get_ptr<MsgT>(topic);
        return ptr->get();
    }

private:
    std::map<std::string, std::shared_ptr<BaseTopicIO>> topicMC_;

    template<typename MsgT>
    std::shared_ptr<TopicIO<MsgT>> get_ptr(const std::string& topic) {
        auto it = topicMC_.find(topic);
        if (it == topicMC_.end())
            throw std::runtime_error("Topic not found: " + topic);

        auto ptr = std::dynamic_pointer_cast<TopicIO<MsgT>>(it->second);
        if (!ptr)
            throw std::runtime_error("Type mismatch on topic: " + topic);

        return ptr;
    }
};

} // namespace topicIO