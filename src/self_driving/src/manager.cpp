// route_before_shoot_(kind,back=True) -> (<aftershoot>? 1:0)
// kind_<status>
// || .route_<status>
//    .route_index_<status>
//ひとまず画像認識ノードの情報から次の姿勢を出すためのノード
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode()
    : Node("manager_node")
    {
        sub_status_ = create_subscription<self_driving::msg::TargetStatus>(
            "pursuit/status", 10,
            std::bind(&ManagerNode::callback, this, std::placeholders::_1)
        );
        pub_pose_ = this->create_publisher<self_driving::msg::Target>(
            "target_pose", 10
        );//QoSは10でいいのか？要検討
    }
private:
    //ここはパラメータの宣言と取得。yamlの数値ををここで同名のメンバ変数として定義している。
    std::map<std::string, std::vector<std::array<double, 3>>> routes_{
        {"route1", {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 1.57}}},
        {"route2", {{0.0, 0.0, 0.0}, {1.0, -1.0, -1.57}, {2.0, -1.0, -1.57}}}
    };
    std::vector<std::array<double, 3>> route_;
    int route_index_ = 0;
    color_queue_ = std::queue<std::string>();
    void set_route(const std::string &kind);
    void publish_target(double x, double y, double yaw)
    {
        self_driving::msg::Target msg;
        msg.x=x;
        msg.y=y;
        msg.yaw=yaw;
        pub_pose_->publish(msg);
    }

    // ==========================
    // pursuit/status コールバック
    // ==========================
    void callback(const self_driving::msg::TargetStatus::SharedPtr msg){
        msg.index
        if (msg.status) {
            
        } else
    }

        // ==========================
    // ルート設定
    // ==========================
    void set_route(const std::string &kind)
    {
        route_ = routes_[kind];
        route_index_ = 0;
        RCLCPP_INFO(get_logger(), "Route set: %s", kind.c_str());
    }
    // ==========================
    // ナビゲーション本体
    // ==========================
    void navigate(double x, double y, double yaw)
    {
        this->sub_status_->
        // ルート完了判定
        if (route_index_ >= route_.size()) {
            RCLCPP_INFO(get_logger(), "Route finished!");
            publish_stop();
            return;
        }
    }


}