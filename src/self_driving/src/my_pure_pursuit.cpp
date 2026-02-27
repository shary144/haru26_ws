#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include <vector>
#include <array>
#include <deque>
#include <map>
#include <cmath>
// route_before_shoot_(kind,back=True) -> (<aftershoot>? 1:0)
// kind_<status>
// || .route_<status>
//    .route_index_<status>

// [callback]pursuit(
// lookup_x
// lookup_y
// est_x
// est_y
// est_yaw
// ) ~> "/robomas/cmd"
class NavNode : public rclcpp::Node
{
public:
    NavNode()
    : Node("nav_node"),
      e_(0.3),
      pgain_(1.5),
      route_index_(0)
    {
        // ICP 推定結果の購読
        sub_pose_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot_pose_icp", 10,
            std::bind(&NavNode::pose_callback, this, std::placeholders::_1)
        );

        // publisher_cmd
        pub_cmd_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);

        // ルート定義
        routes_ = {
            {"blue", {
                {0.35, 5.888},
                {0.35, 3.5},
                {1.362, 3.5},//ここで射出
                {0.35, 3.5},
                {0.35, 5.888}
            }},
            {"yellow", {
                {0.35, 5.888},
                {0.35, 1.85},
                {1.8 , 1.85},//ここで射出
                {0.35, 1.85},
                {0.35, 5.888}
            }},
            {"red", {
                {0.35, 5.888},
                {0.35 , 5.112},
                {0.924, 5.112},//ここで射出
                {0.35 , 5.112},
                {0.35, 5.888}
            }},
            {"init",{
                {0.35,5.888}
            }}
        };

        // 今回は red を例として使用
        set_route("red");
    }

private:
    // ==========================
    // ICP callback
    // ==========================
    void pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3) return;

        double x   = msg->data[0];
        double y   = msg->data[1];
        double yaw = msg->data[2];

        //あんま自信ないけどlidarの位置と向きを考慮してロボットの位置と向きを逆算
        double robot_x = x + offset_r * std::cos(yaw-offset_theta);
        double robot_y = y + offset_r * std::sin(yaw-offset_theta);
        double robot_yaw = yaw - offset_yaw;

        // ---- 平均化したい場合はここを ON ----
        // add_history(x, y, yaw);
        // auto [fx, fy, fyaw] = filtered_pose();
        // navigate(fx, fy, fyaw);
        // -------------------------------------

        // 平均化しない場合はこちら
        navigate(robot_x, robot_y, robot_yaw);
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
        // ルート完了判定
        if (route_index_ >= route_.size()) {
            RCLCPP_INFO(get_logger(), "Route finished!");
            publish_stop();
            return;
        }

        auto target = route_[route_index_];
        double tx = target[0];
        double ty = target[1];
        
        double dx = tx - x;
        double dy = ty - y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // 到達判定
        if (dist < e_) {
            RCLCPP_INFO(get_logger(), "Reached waypoint %zu at %s", route_index_, kind.c_str());
            route_index_++;
            if (route_index_ >= route_.size()) {
                RCLCPP_INFO(get_logger(), "Route finished!");
                set_route("");
                return;
            }
            return;
        }

        // 目標方向
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalize_angle(target_yaw - yaw);

        // 簡易 P 制御
        double wz = pgain_ * yaw_error;   // 角速度
        double vx = 0.3;               // 前進速度（固定）

        // 角度が大きくズレているときは回転優先
        if (std::fabs(yaw_error) > 0.5) {
            vx = 0.0;
        }

        publish_cmd(vx, wz);
    }

    // ==========================
    // 速度 publish
    // ==========================
    void publish_cmd(double vx, double wz)
    {
        auto msg = robomas_interfaces::msg::RobomasPacket();
        
        float cos = vx * std::cos(wz); //左スティックX
        float sin = vx * std::sin(wz); //左スティックY
        
        //motor1(足回り)
        robomas_interfaces::msg::MotorCommand cmd1;
        cmd1.motor_id = 1;
        cmd1.mode = 1;
        cmd1.target = (-0.5f * cos + 0.866f * sin) * 1000.0f;
        msg.motors.push_back(cmd1);

        //motor2(足回り)
        robomas_interfaces::msg::MotorCommand cmd2;
        cmd2.motor_id = 2;
        cmd2.mode = 1;
        cmd2.target = (-0.5f * cos - 0.866f * sin) * 1000.0f;
        msg.motors.push_back(cmd2);

        //motor3(足回り)
        robomas_interfaces::msg::MotorCommand cmd3;
        cmd3.motor_id = 3;
        cmd3.mode = 1;
        cmd3.target = cos * 1000.0f;
        msg.motors.push_back(cmd3);
        
        pub_cmd_->publish(msg);
    }

    void publish_stop()
    {
        publish_cmd(0.0, 0.0);
    }

    // ==========================
    // 角度正規化
    // ==========================
    double normalize_angle(double a)
    {
        while (a > M_PI)  a -= 2*M_PI;
        while (a < -M_PI) a += 2*M_PI;
        return a;
    }

    // ==========================
    // 平均化（任意）
    // ==========================
    void add_history(double x, double y, double yaw)
    {
        history_.push_back({x, y, yaw});
        if (history_.size() > 10) history_.pop_front();//ここははみだしを消す処理
    }

    std::array<double,3> filtered_pose()
    {
        double sx=0, sy=0, syaw=0;
        for (auto &p : history_) {
            sx += p[0];
            sy += p[1];
            syaw += p[2];
        }
        size_t n = history_.size();
        return {sx/n, sy/n, syaw/n};
    }

    // ==========================
    // メンバ変数
    // ==========================
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_;
    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr pub_cmd_;

    std::map<std::string, std::vector<std::array<double,2>>> routes_;
    std::vector<std::array<double,2>> route_;
    size_t route_index_;

    double e_;  // waypoint 到達判定の半径
    double pgain_; // P 制御のゲイン

    double offset_r = 0.33f; //lidarの取付位置のロボット座標原点からの距離
    double offset_theta = 4/3*M_PI; //lidarの取付位置のロボット正面を基準にした角度
    double offset_yaw = 4/3*M_PI; //lidarの取り付け向きのロボット正面からの角度

    std::deque<std::array<double,3>> history_; // 平均化用
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavNode>());
    rclcpp::shutdown();
    return 0;
}