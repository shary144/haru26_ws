#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <array>
#include <map>
#include <functional>
#include <chrono>
#include <cmath>

#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode()
    : Node("manager_node")
    {
        route_index_ = 0;
        waypoint_index_ = 0;
        promise_index_ = 0;

        sub_status_ = create_subscription<self_driving::msg::TargetStatus>(
            "pursuit/status", 10,
            std::bind(&ManagerNode::update_status, this, std::placeholders::_1)
        );

        pub_pose_ = this->create_publisher<self_driving::msg::Target>(
            "target_pose", 10
        );

        sub_pose_icp_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "robot_pose_icp", 10,
            std::bind(&ManagerNode::pose_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ManagerNode::run_promise_chain, this)
        );

        // 例: 最初に init ルートを展開しておく
        unfold_route("init", false);
    }

private:
    // ==========================
    // ROS I/F
    // ==========================
    rclcpp::Subscription<self_driving::msg::TargetStatus>::SharedPtr sub_status_;
    rclcpp::Publisher<self_driving::msg::Target>::SharedPtr           pub_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_icp_;
    rclcpp::TimerBase::SharedPtr                                      timer_;

    // ==========================
    // 状態・ルート関連
    // ==========================
    double wallDistance_ = 0.35;

    std::map<std::string, std::vector<double>> nav_point_ = {
        {"blue",      {1.362, 3.5}},
        {"yellow",    {1.8,   1.85}},
        {"red",       {0.924, 5.112}},
        {"notezone_", {0.35,  5.888}}
    };

    // 制御順番を管理するための変数
    std::vector<std::function<bool()>> promise_chain_; // 制御全体
    size_t promise_index_;                             // promise_chain のどこまで実行したか
    size_t waypoint_index_;                            // waypoint の通し番号
    size_t route_index_;                               // ルート内インデックス（必要なら使用）

    self_driving::msg::TargetStatus status_msg_;

    std::map<std::string, bool> colors_iscompleted_ = {
        {"blue",   false},
        {"yellow", false},
        {"red",    false}
    };

    // ==========================
    // コールバック
    // ==========================
    void pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr /*msg*/)
    {
        // 必要に応じて ICP からの自己位置を処理
    }

    void update_status(const self_driving::msg::TargetStatus::SharedPtr msg)
    {
        status_msg_ = *msg;
    }

    // ==========================
    // ルート生成
    // ==========================
    std::vector<std::array<double, 3>> route_seg(const std::string &kind, bool back = false)
    {
        std::vector<std::array<double, 3>> route;

        if (kind == "init") {
            route.push_back({nav_point_["notezone_"][0],
                             nav_point_["notezone_"][1],
                             M_PI / 2.0});
            return route;
        }

        if (back) {
            route.push_back({wallDistance_,               nav_point_[kind][1],       M_PI / 2.0});
            route.push_back({wallDistance_,               nav_point_["notezone_"][1],M_PI / 2.0});
            route.push_back({nav_point_["notezone_"][0],  nav_point_["notezone_"][1],M_PI / 2.0});
        } else {
            route.push_back({wallDistance_,               nav_point_["notezone_"][1], M_PI / 2.0});
            route.push_back({nav_point_["notezone_"][0],  nav_point_["notezone_"][1],-M_PI / 2.0});
            route.push_back({wallDistance_,               nav_point_[kind][1],       -M_PI / 2.0});
            route.push_back({nav_point_[kind][0],         nav_point_[kind][1],        0.0});
        }

        return route;
    }

    // ==========================
    // pursuit 本体
    // ==========================
    bool pursuit(const std::array<double, 3> &waypoint)
    {
        // 毎回 publish（status に関係なく）
        publish_target(waypoint[0], waypoint[1], waypoint[2]);

        // status が true なら次へ進む
        if (status_msg_.status) {
            status_msg_.status = false;
            waypoint_index_++;
            return true;
        }

        // 進まないが、publish は継続される
        return false;
    }


    // ==========================
    // ルート展開
    // ==========================
    bool unfold_route(const std::string &kind, bool isback)
    {

        auto route = route_seg(kind, isback);

        // 開始ログ
        promise_chain_.insert(
            promise_chain_.begin() + promise_index_,
            [this, kind, isback]() {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Route '%s/%s' start",
                    kind.c_str(),
                    isback ? "back" : "forward"
                );
                return true;
            }
        );
        //promise_index_++;

        // waypoint 群を挿入
        for (const auto &wp : route) {
            promise_chain_.insert(
                promise_chain_.begin() + promise_index_,
                [this, wp]() {
                    return this->pursuit(wp);
                }
            );
        }

        // 終了ログ
        promise_chain_.insert(
            promise_chain_.begin() + promise_index_,
            [this, kind, isback]() {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Route '%s/%s' end",
                    kind.c_str(),
                    isback ? "back" : "forward"
                );
                return true;
            }
        );
        printf("%d",promise_chain_.size());

        return true;
    }

    // ==========================
    // promise_chain 実行
    // ==========================
    void run_promise_chain()
    {
        if (promise_index_ >= promise_chain_.size()) {
            printf("エラーにより詰み\n");
            return;
        }

        bool done = promise_chain_[promise_index_]();
        if (done) {
            promise_index_++;
        }
    }

    // ==========================
    // 目標 publish
    // ==========================
    void publish_target(double x, double y, double yaw)
    {
        self_driving::msg::Target msg;
        msg.index = static_cast<int32_t>(waypoint_index_);
        msg.mode  = 0; // pure_pursuit
        msg.x     = x;
        msg.y     = y;
        msg.yaw   = yaw;
        pub_pose_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManagerNode>());
    rclcpp::shutdown();
    return 0;
}