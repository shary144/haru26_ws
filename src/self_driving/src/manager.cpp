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
template <typename MsgT>
class LatestValueSubscriber {
public:
    LatestValueSubscriber(rclcpp::Node* node, const std::string& topic)
    {
        sub_ = node->create_subscription<MsgT>(
            topic, 10,
            [this](const typename MsgT::SharedPtr msg) {
                latest_ = *msg;
                has_value_ = true;
            }
        );
    }

    bool has_value() const { return has_value_; }

    MsgT get() const { return latest_; }

private:
    typename MsgT::SharedPtr sub_;
    MsgT latest_;
    bool has_value_ = false;
};


class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode()
    : Node("manager_node")
    {
        route_index_ = 0;
        sub_status_ = create_subscription<self_driving::msg::TargetStatus>(
            "pursuit/status", 10,
            std::bind(&ManagerNode::update_status, this, std::placeholders::_1)
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
    rclcpp::Subscription<self_driving::msg::TargetStatus>::SharedPtr sub_status_;
    rclcpp::Publisher<self_driving::msg::Target>::SharedPtr pub_pose_;
    void run_promise_chain() //主制御のコールバック
    void update_status();

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
    std::vector<std::function<bool()>> promise_chain; //制御全体
    size_t promise_index_; //promise_chainのどこまで実行したか
    size_t waypoint_index_; //単純にpursuitした分連番でidを振る(クラス変数)
    self_driving::msg::TargetStatus status_msg;
    bool follow_waypoint(std::string kind, bool back=false);
    bool follow_route();/*waypoint列をそのまま引数にもてる*/
    bool catch_front_target()
    
    //unfold:promise_chainで回ってきたときに自分のインデックスの次にpursuit列とフッターをpromise_chainに挿入する処理
    //これでpursuitを疑似非同期的に実行できる

    void run_promise_chain() {
        auto [func, arg] = promise_chain[promise_index];
        if (func(arg)) promise_index++;
    }

    //色ピックのステータス
    std::map<std::string, bool> colors_iscompleted = {
        {"blue", false},
        {"yellow", false},
        {"red", false}
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
    //リアルタイムでカメラ補正:targetがずれることになるので参照渡し
    bool pursuit(std::array<double, 3> waypoint) {
        //追従の完了を待って、次の目標値をpublishする関数
        //pursuit_nodeから到達を検知したら終了→trueを返す
        this->publish_target(waypoint[0],waypoint[1], waypoint[2]);
        //ここでpursuit_nodeからの到達を検知する必要がある
        if (sub_status_) {
            this->waypoint_index_++;
            this->promise_index_++;
            return true;
        } else {
            return false;
        }
    }

    bool unfold_route(const std::string &kind, bool isback) {
        promise_chain.insert(promise_chain.begin()+this->promise_index_, [] {
            RCLCPP_INFO(get_logger(), "Route '%s/%s' start", kind.c_str(),isback?"back":"foreward"); return true;});
        auto route = route_seg(kind);
        for(int i=1;i<route.size+1;i++){
            promise_chain.insert(
                promise_chain.begin()+this->promise_index_+i,
                std::bind(pursuit,route[i])
            );
        }
        promise_chain.insert(route.size+2, [] {
            RCLCPP_INFO(get_logger(), "Route '%s/%s' end", kind.c_str(),isback?"back":"foreward"); return true;})
        return true;
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
    void update_status(const self_driving::msg::TargetStatus::SharedPtr msg){
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

};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManagerNode>());
  rclcpp::shutdown();
  return 0;
}