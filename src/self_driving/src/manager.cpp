#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"
#include "self_driving/msg/ball_array.hpp"
#include "self_driving/msg/ball.hpp"
#include "BallCache.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <array>
#include <cmath>
#include "robomas_interfaces/msg/robomas_frame.hpp"

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode()
    : Node("manager_node")
    {
        sub_status_ = create_subscription<self_driving::msg::TargetStatus>(
            "pursuit/status", 10,
            std::bind(&ManagerNode::update_status, this, std::placeholders::_1)
        );

        pub_pose_ = this->create_publisher<self_driving::msg::Target>(
            "target_pose", 10
        );

        sub_pose_icp_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "robot_pose_icp", 10,
            std::bind(&ManagerNode::automaton, this, std::placeholders::_1)
        );

        sub_ball_ = create_subscription<self_driving::msg::BallArray>(
            "ball_array", 10,
            std::bind(&ManagerNode::update_ball, this, std::placeholders::_1)
        );
        can_pub_ = this->create_publisher<robomas_interfaces::msg::CanFrame>("/robomas/can_tx", 10);

        color_cache = {0,0,0};
    }

private:
    // ROS I/F
    rclcpp::Subscription<self_driving::msg::TargetStatus>::SharedPtr sub_status_;
    rclcpp::Publisher<self_driving::msg::Target>::SharedPtr          pub_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_icp_;
    rclcpp::Subscription<self_driving::msg::BallArray>::SharedPtr    sub_ball_;
    rclcpp::Publisher<robomas_interfaces::msg::CanFrame>SharedPtr     can_pub_;

    self_driving::msg::TargetStatus status_msg_;
    int phase = 0;

    std::array<int,3> color_cache; // 色ごとの取得数
    ballcache::BallCache ball_cache_;
    double last_robot_x_ = 0.0;
    double last_robot_y_ = 0.0;

    ballcache::Ball chosen_ball{0,0.0f,0.0f,false,0};

    double note_front_zone[3][2] =
    {
        {2.9, 1.5+0.35},   // yellow
        {2.46, 3.5},  // blue
        {2.024, 7-(1.5+0.35)}  // red
    };

    // fallback 用の仮のデフォルト位置（適当に置いてるので要調整）
    double default_pos[3][2] =
    {
        {0.5, 0.8},  // yellow.x
        {1.1, 1.4},  // blue.x;
        {1.7, 1.8}   // red.x
    };

    int color_id = 0;

    // ==========================
    // オートマトン
    // ==========================
    void automaton(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        auto msg_can = robomas_interfaces::msg::CanFrame();
        msg_can.id = 0x100;
        msg_can.dlc = 8;
        msg_can.data = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};

        can_pub_->publish(msg_can);
        if (msg->data.size() >= 2) {
            last_robot_x_ = msg->data[0];
            last_robot_y_ = msg->data[1];
        }

        printf("phase:%d", phase);

        switch (phase) {
        case -1:
            printf("grip_setup\n");//把持を開いた状態で.
            if(grip_setup()) {
                phase = 0;
            }
            break;
        case 0:
            printf("start(120度でnotezone入る).\n");
            if (pursuit({0.35, 6.2, M_PI/2})) {
                phase = 1;
            }
            break;

        case 1:
            printf("なぎ倒すために把持を開く.\n");
            if (grip_down_tosweep()) {
                phase = 2;
            }
            break;

        case 2:
            printf("なぎ倒すように. \n");
            if (pursuit({3.15, 6.2, M_PI/2})) {
                phase = 3;
            }
            break;
        case 3:
            printf("ちょっと回転. \n");
            if(pursuit({3.15 ,6.2, M_PI/3})) {
                phase = 4;
            }
            break;
        case 4:
            printf("スタートに戻ったらここは完了。\n");
            if (pursuit({0.35, 3.5, 0})) {
                phase = 5;
            }
            break;
        case 5:
            printf("箱の間を通る:1.\n");
            if (pursuit({0.35, 3.5-0.912, 0})) {
                phase = 6;
            }
            break;
        case 6:
            printf("箱の間を通る:2\n");
            if (pursuit({3.5-0.35, 3.5-0.912,0})) {
                phase = 7;
            }
            break;
        case 7:
            printf("リバース定位置へ行く");
            if (pursuit({3.5-0.35, 0.262+0.038+0.4, -M_PI/2})) {
                phase = 8;
            }
            break;
        case 8:
            printf("[%d]色のボール一つ目の前に行く\n",color_id);
            if (pursuit({3.5-default_pos[color_id][0],0.262+0.038+0.4,-M_PI/2})) {
                phase = 9;
            }
            break;
        case 9:
            printf("一個目を取る\n",color_id);
            if (grip()) {
                phase = 10;
            }
            break;
        case 10:
            printf("[%d]色のボール二つ目の前に行く\n",color_id);
            if (pursuit({3.5-default_pos[color_id][1],0.262+0.038+0.4,-M_PI/2})) {
                phase = 11;
            }
            break;
        case 11:
            printf("二個目を取る\n",color_id);
            if (grip2()) {
                phase = 12;
            }
            break;
        case 12:
            printf("リバース定位置へ行く\n");
            if (pursuit({3.5-0.35, 0.262+0.038+0.4, -M_PI/2})) {
                phase = 13;
            }
            break;
        case 13:
            printf("(壁際,ノーツy)に行く\n");
            if (pursuit({3.5-0.35, note_front_zone[color_id][1], 0})) {
                phase = 14;
            }
            break;
        case 14:
            printf("(ノード手前x,ノードy)に行く\n");
            if (pursuit({note_front_zone[color_id][0],note_front_zone[color_id][1],0})) {
                phase = 15;
            }
            break;
        case 15:
            printf("一個目を射出\n");
            if (shoot()) {
                phase = 16;
            }
            break;
        case 16:
            printf("昇降を上げる\n");
            if (onlyup()) {
                phase = 17;
            }
            break;
        case 17:
            printf("一個目を射出\n");
            if (shoot()) {
                phase = 18;
            }
            break;
        case 18:
            printf("(壁際、ノーツy)に戻る\n");
            if (pursuit({3.5-0.35,note_front_zone[color_id][1],0})) {
                phase = 19;
            }
            break;
        case 19:
            printf("色によって変わるが\n");
            if (color_id>=3) {
                printf("fanfare!");
            } else {
                printf("phase 7へ戻る");
                color_id++;
                phase = 7;
            }
            break;
        }
    }

    // ==========================
    // Pure Pursuit への指示
    // ==========================
    bool pursuit(const std::array<double, 3> &wp)
    {
        publish_target(wp[0], wp[1], wp[2]);

        if (status_msg_.status) {
            status_msg_.status = false;
            return true;
        }
        return false;
    }

    void publish_target(double x, double y, double yaw)
    {
        self_driving::msg::Target msg;
        msg.index = 0;
        msg.mode  = 0;
        msg.x = x;
        msg.y = y;
        msg.yaw = yaw;
        pub_pose_->publish(msg);
    }

    // ==========================
    // 射出
    // ==========================
    bool shoot()
    {
        self_driving::msg::Target msg;
        msg.index = 0;
        msg.mode  = 1;
        pub_pose_->publish(msg);

        if (status_msg_.status) {
            status_msg_.status = false;
            return true;
        }
        return false;
    }

    // 把持
    bool grip()
    {
        self_driving::msg::Target msg;
        msg.index = 0;
        msg.mode  = 2;
        pub_pose_->publish(msg);

        if (status_msg_.status) {
            std::cout << "grip end!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            status_msg_.status = false;
            return true;
        }
        return false;
    }

    bool grip_setup()
    {
        self_driving::msg::Target msg;
        msg.index = 0;
        msg.mode  = 3;
        pub_pose_->publish(msg);

        if (status_msg_.status) {
            std::cout << "setup end!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            status_msg_.status = false;
            return true;
        }
        return false;
    }
    bool grip_down_tosweep()
    {
        self_driving::msg::Target msg;
        msg.index = 0;
        msg.mode  = 4;
        pub_pose_->publish(msg);

        if (status_msg_.status) {
            std::cout << "grip_down_tosweep end!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            status_msg_.status = false;
            return true;
        }
        return false;
    }
    bool grip2()
    {
        self_driving::msg::Target msg;
        msg.index = 0;
        msg.mode = 5;

        if (status_msg_.status) {
            std::cout << "grip2 end!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            status_msg_.status = false;
            return true;
        }
        return false;
    }

    bool onlyup()
    {
        self_driving::msg::Target msg;
        msg.index = 0;
        msg.mode = 6;

        if (status_msg_.status) {
            std::cout << "onlyup end!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            status_msg_.status = false;
            return true;
        }
        return false;
    }

    // ==========================
    // ボール選択
    // ==========================
    void choose_ball()
    {
        auto &balls = ball_cache_.ball_array;

        double rx = last_robot_x_;
        double ry = last_robot_y_;

        double best_dist = 1e9;
        bool found = false;

        ballcache::Ball best_ball;

        for (auto &b : balls) {

            if (b.color_id < 0 || b.color_id > 2) continue;

            if (color_cache[b.color_id] >= 2)
                continue;

            if (!b.onstage)
                continue;

            double d = std::hypot(b.x - rx, b.y - ry);

            if (d < best_dist) {
                best_dist = d;
                best_ball = b;
                found = true;
            }
        }

        if (found) {
            chosen_ball = best_ball;
            printf("choose_ball: color=%d, pos=(%.3f, %.3f)\n",
                   chosen_ball.color_id, chosen_ball.x, chosen_ball.y);
            return;
        }

        printf("choose_ball: no visible ball, fallback\n");

        for (int cid = 0; cid < 3; cid++) {
            if (color_cache[cid] < 2) {
                chosen_ball.color_id = cid;
                chosen_ball.x = default_pos[cid][0];
                chosen_ball.y = default_pos[cid][1];
                return;
            }
        }
    }

    void update_status(const self_driving::msg::TargetStatus::SharedPtr msg)
    {
        std::cout << "UPDATE_STATUS REACH!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        status_msg_ = *msg;
    }

    void update_ball(const self_driving::msg::BallArray::SharedPtr msg)
    {
        std::vector<ballcache::Ball> new_ball_array;
        new_ball_array.reserve(msg->ballarray.size());

        for (const auto &ball : msg->ballarray) {
            ballcache::Ball cache;
            cache.color_id = ball.color_id;
            cache.x        = ball.robot_x;
            cache.y        = ball.robot_y;
            cache.onstage  = true;
            cache.N        = 0; // enroll_ball_array 内で1にされる
            new_ball_array.push_back(cache);
        }

        ball_cache_.enroll_ball_array(new_ball_array);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManagerNode>());
    rclcpp::shutdown();
    return 0;
}