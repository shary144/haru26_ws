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
#include <cmath>

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode()
    : Node("manager_node")
    {
        route_index_ = 0;
        sub_status_ = create_subscription<self_driving::msg::TargetStatus>(
            "pursuit/status", 10,
            std::bind(&ManagerNode::callback, this, std::placeholders::_1)
        );
        pub_pose_ = this->create_publisher<self_driving::msg::Target>(
            "target_pose", 10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ManagerNode::publish_once, this)
        );
    }

private:
    //ルート定義系
    double wallDistance = 0.35;
    std::map<std::string, std::vector<double>> nav_point_ = {
        {"blue", {1.362, 3.5}},
        {"yellow", {1.8, 1.85}},
        {"red", {0.924, 5.112}},
        {"notezone_", {0.35, 5.888}}
    };
    std::vector<std::array<double, 3>> main_route;
    //制御順番を管理するための変数
    std::vector<std::function<void()>> callback_chain;
    int waypoint_id_; //単純にpublishした分連番でidを振る(クラス変数)
    self_driving::msg::TargetStatus status_msg;
    
    //色ピックのステータス
    std::map<std::string, bool> colors_isleft = {
        {"blue", true},
        {"yellow", true},
        {"red", true}
    };
    std::vector<std::vector<double>> route_seg(std::string kind, bool back=false) {
        //ルート生成関数。kindは画像認識ノードからの情報をもとに、どのルートを通るかを決めるための引数。
        if (kind=="init") 
            return {
                {nav_point_["notezone_"].at(0), nav_point_["notezone_"].at(1),M_PI/2}
            };
        if (back) {
            return {
                {wallDistance,nav_point_[kind].at(1),M_PI/2},
                {wallDistance,nav_point_["notezone_"].at(1),M_PI/2},
                {nav_point_["notezone_"].at(0), nav_point_["notezone_"].at(1), M_PI/2}
            };
        } else {
            return {
                {wallDistance,nav_point_["notezone_"].at(1), M_PI/2},
                {nav_point_["notezone_"].at(0), nav_point_["notezone_"].at(1), -M_PI/2},
                {wallDistance,nav_point_[kind].at(1),-M_PI/2},
                {nav_point_[kind].at(0), nav_point_[kind].at(1), 0.0}
            };
        }
    }
    bool pursuit() {
        //追従の完了を待って、次の目標値をpublishする関数
        //pursuit_nodeから到達を検知したら終了→trueを返す
        callback_msg
        if  return true;
    }

    void publish_once() {
        //目標値をpublishして挙動を確認(テスト用)
        self_driving::msg::Target msg;
        msg.index=0;
        msg.x=1.0;
        msg.y=2.0;
        msg.yaw=0.5;
        pub_pose_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published target pose: index=%d, x=%.2f, y=%.2f, yaw=%.2f", 
                    msg.index, msg.x, msg.y, msg.yaw);
    }
    //
    void update_route_status(self_driving::msg::TargetStatus)
    bool follow_waypoint(std::string kind) {
        //ルートの達成を状態として持たせる
        auto route_segment = this->route_seg(kind);
        //pursuit/statusをみて、追従が終わったら次の目標値をpublishする、という流れを作る
        for (const auto& point : route_segment) {
            this->publish_target(point[0], point[1], point[2]);
        }
    }

    void publish_target(double x, double y, double yaw)
    {
        self_driving::msg::Target msg;
        msg.index = waypoint_id_;
        msg.x=x;
        msg.y=y;
        msg.yaw=yaw;
        pub_pose_->publish(msg);
    }

    // ==========================
    // pursuit/status コールバック
    // ==========================
    void callback(const self_driving::msg::TargetStatus::SharedPtr msg){
        status_msg = *msg;
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