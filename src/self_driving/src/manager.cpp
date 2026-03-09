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
#include "robomas_interfaces/msg/"
#include "my_tf.hpp"
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

struct BallChache{
    int color_id
    double global_x;
    double global_y;
    bool state;
};

struct BallLayout{
    BallLayout(void):{};
    double threshold_r = 0.3
    std::vector<ballChache> ball_layout_;
    void insert(std::vector<self_driving::msg::Ball> ball_array){
        for (BallChache& ball_chache: ball_layout_){
            for (self_driving::msg::Ball& ball: ball_layout_){
                double d=std::sqrt(std::pow(ball_chache.global_x-ball.robot_x, 2)
                 + std::pow(ball_chache.global_y-ball.robot_y, 2))
                if d<thereshol
            }
        }
    }
}


//Mytf tf(x=0,y=0,yaw=0)
//tf.sub_rot(lidar_yaw).sub_trans(lidar_x,lidar_y)
//Mytf tf2();
//tf3 = tf2.set_value(icp_x,icp_y,icp_yaw).apply(tf).add_trans(ball_x,ball_y)
//tf3.x,tf3.yとして値を引き出す

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode()
    : Node("manager_node")
    {
        //内部的に更新がかかる構造体としてメッセージを購読
        LatestValueSubscriber<self_driving::msg::TargetStatus> status_msg(this,"pursuit/status");
        LatestValueSubscriber<self_driving::msg::BallArray> ball_msg(this,"ball_array")

        route_index_ = 0;
        // ICP 推定結果の購読
        sub_pose_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "robot_pose_icp", 10,
            std::bind(&NavNode::run_promise_chain, this, std::placeholders::_1));
        
        pub_pose_ = this->create_publisher<self_driving::msg::Target>(
            "target_pose", 10
        );
    }

private:
    LatestValueSubscriber<self_driving::msg::TargetStatus> status_msg;
    LatestValueSubscriber<self_driving::msg::BallArray> ball_msg;
    rclcpp::Subscription<self_driving::msg::TargetStatus>::SharedPtr sub_status_;
    rclcpp::Publisher<self_driving::msg::Target>::SharedPtr pub_pose_;
    
    //座標変換用の定数
    double lidar_offset_x = 0.286;
    double lidar_offset_y = -0.165;
    double lidar_offset_yaw = -1.047;
    double nav_radius = 0.3;
    std::vector<Ball_chache> ball_chache: //{{color,x,y,onstage}}

    void run_promise_chain() //主制御のコールバック
    void unfold_pick()

    bool pick_ball(int id){
        //座標変換系(lidarグローバル座標→ロボットグローバル座標)
        Mytf recipe_lidar_offset();
        recipe_lidar_offset
            .sub_rot(lidar_offset_yaw)
            .sub_trans(lidar_offset_x, lidar_offset_y);
        //ボールグローバル座標の計算
        Mytf tf2();
        ball_msg.get()
        tf2.set_value(icp_x,icp_y,icp_yaw);
            .apply(recipe_lidar_offset)
            .add_trans(ball_msg.get().data.,ball_msg.get().data);
        publish_target(tf2.x(),tf2.y(),tf2.yaw());   
    }


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
    std::vector<std::function<bool()>> promise_chain = {
        std::bind(&Manager::unfold_route,"init",false),
        &Manager::ball_pick,
        std::bind(&Manager::unfold_route,kinds[0],true),
        &Manager::shoot,
        std::bind(&Manager::unfold_route,kinds[0],false),
    };

    size_t promise_index_; //promise_chainのどこまで実行したか
    size_t waypoint_index_; //単純にpursuitした分連番でidを振る(クラス変数)
    bool follow_waypoint(std::string kind, bool back=false);
    bool pursuit();/*waypoint列をそのまま引数にもてる*/
    
    //unfold:promise_chainで回ってきたときに自分のインデックスの次にpursuit列とフッターをpromise_chainに挿入する処理
    //これでpursuitを疑似非同期的に実行できる

    void run_promise_chain(std_msgs::msg::Float32MultiArray msg) {
        this->icp_msg = *msg;
        auto func = this->promise_chain[this->promise_index];
        if (func()) this->promise_index++;
    }

    //色ピックのステータス
    std::map<std::string, int> colors_shot = {
        {"blue", 0},
        {"yellow", 0},
        {"red", 0}
    };
    
    std::vector<self_driving::msg::Ball> balls = ball_msg.get()->data.BallArray;
    //1.一番左のボールをピック。
    //2.同じ色のボールを探してピック。(インデックスを保存)
    std::array<std::vector<int>,3> closest_ind;
    for(int i=0;i<ball_array.size();i++){
        auto closest_ball = balls[closest_ind[balls[i].color_id][i]];
        auto ith_ball = balls[i];
        double d_min = closest_ball.robot_x*closest_ball.robot_x+closest_ball.robot_y*closest_ball.robot_y;
        double d = ith_ball.robot_x*ith_ball.robot_x+ith_ball.robot_y*ith_ball.robot_y;
    }
    
    balls[closest_ind[kind]].x
    

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
        if (status_msg->status) {
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
    bool unfold_shoot(){
        auto motor_msg = robomas_interfaces::msg::RobomasPacket();
        auto motor_feedback = robomas_interfaces::msg::RobomasFrame();
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.motor_id = 4;
        cmd.mode = 2;
        //位置制御(mode2)
        //robomas_interfaces/msg/RobomasFrame.msgのangle属性で基盤起動から現在までの累積角度を取り出せて、
        //targetは目標累積角度を指定
        //8*19*360°で一周のはずなので
        if <=motor_feedback->angle[3]<=
        cmd->target-
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