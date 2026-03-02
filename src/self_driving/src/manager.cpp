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
        /*sub_status_ = create_subscription<self_driving::msg::TargetStatus>(
            "pursuit/status", 10,
            std::bind(&ManagerNode::callback, this, std::placeholders::_1)
        );*/
        pub_pose_ = this->create_publisher<self_driving::msg::Target>(
            "target_pose", 10
        );//QoSは10でいいのか？要検討

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ManagerNode::publish_once, this)
        );
    }

private:
    void publish_once() {
        //目標値をpublishして挙動を確認
        self_driving::msg::Target msg;
        msg.index=0;
        msg.x=1.0;
        msg.y=2.0;
        msg.yaw=0.5;
        pub_pose_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published target pose: index=%d, x=%.2f, y=%.2f, yaw=%.2f", 
                    msg.index, msg.x, msg.y, msg.yaw);
    }
/*
    //ここはパラメータの宣言と取得。yamlの数値ををここで同名のメンバ変数として定義している。
    std::vector<std::array<double, 3>> route_;
    int route_index_ = 0;
    double wallDistance = 0.35;
    std::vector<std::vector<double>> blue_nav_point_ = {1.362, 3.5};
    std::vector<std::vector<double>> yellow_nav_point_ = {1.8, 1.85};
    std::vector<std::vector<double>> red_nav_point_ = {0.924, 5.112};
    std::vector<std::vector<double>> notezone_entrance = {0.35, 5.888};
    std::queue<std::string> color_queue_; // 残りの色を管理するキュー
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

*/
    /*rclcpp::Subscription<self_driving::msg::TargetStatus>::SharedPtr sub_status_;*/
    rclcpp::Publisher<self_driving::msg::Target>::SharedPtr pub_pose_;
    rclcpp::TimerBase::SharedPtr timer_;   // ← これが必要！
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManagerNode>());
  rclcpp::shutdown();
  return 0;
}