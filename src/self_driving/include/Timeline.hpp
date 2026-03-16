#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <functional>

class Timeline {
public:
    explicit Timeline(rclcpp::Node* node)
    : node_(node), t_(0.0), running_(false), index_(0)
    {}

    // t秒後に実行するイベントを登録
    Timeline& at(double time_sec, std::function<void()> cb) {
        events_.push_back({time_sec, cb});
        return *this;
    }

    // タイムライン開始
    void start() {
        t_ = 0.0;
        index_ = 0;
        running_ = true;
        last_update_ = node_->get_clock()->now();
    }

    // 一時停止
    void pause() {
        running_ = false;
    }

    // 再開
    void resume() {
        running_ = true;
        last_update_ = node_->get_clock()->now();
    }

    // 毎フレーム呼ぶ
    void update() {
        if (!running_) return;

        rclcpp::Time now = node_->get_clock()->now();
        double dt = (now - last_update_).seconds();
        last_update_ = now;

        t_ += dt;  // 経過時間を加算

        // イベント実行
        while (index_ < events_.size() && t_ >= events_[index_].first) {
            events_[index_].second();  // コールバック実行
            index_++;
        }
    }

    double time() const { return t_; }
    bool finished() const { return index_ >= events_.size(); }

private:
    rclcpp::Node* node_;

    double t_;                     // 経過時間（秒）
    bool running_;
    size_t index_;
    rclcpp::Time last_update_;

    std::vector<std::pair<double, std::function<void()>>> events_;
};