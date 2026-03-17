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

        color_cache = {0,0,0};
    }

private:
    // ROS I/F
    rclcpp::Subscription<self_driving::msg::TargetStatus>::SharedPtr sub_status_;
    rclcpp::Publisher<self_driving::msg::Target>::SharedPtr          pub_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_icp_;
    rclcpp::Subscription<self_driving::msg::BallArray>::SharedPtr    sub_ball_;

    self_driving::msg::TargetStatus status_msg_;
    int inner_order = 0;
    int phase = -1;

    std::array<int,3> color_cache; // 色ごとの取得数
    ballcache::BallCache ball_cache_;
    double last_robot_x_ = 0.0;
    double last_robot_y_ = 0.0;

    ballcache::Ball chosen_ball{0,0.0f,0.0f,false,0};

    double note_front_zone[3][2] =
    {
        {1.362, 3.5},   // blue
        {1.8,   1.85},  // yellow
        {0.924, 5.112}  // red
    };

    // fallback 用の仮のデフォルト位置（適当に置いてるので要調整）
    double default_pos[3][2] =
    {
        {0.35, 5.5},  // blue
        {0.60, 5.5},  // yellow
        {0.85, 5.5}   // red
    };

    // ==========================
    // オートマトン
    // ==========================
    void automaton(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            last_robot_x_ = msg->data[0];
            last_robot_y_ = msg->data[1];
        }

        printf("phase:%d, inner_order:%d\n", phase, inner_order);

        switch (phase) {
        case -1:
            printf("grip_setup\n");
            if(grip_setup()) {
                phase = 0;
            }
            break;

        case 0:
            printf("start\n");
            if (pursuit({0.35, 5.888, M_PI/2})) {
                phase = 1;
                inner_order = 3;
            }
            break;

        case 1:
            printf("catch_ball\n");

            // if (inner_order == 0) {
            //     choose_ball();
            //     inner_order = 1;
            // }

            // if (inner_order == 1 &&
            //     pursuit({chosen_ball.x, 5.888, M_PI/2})) {
            //     inner_order = 2;
            // }

            // if (inner_order == 2 &&
            //     pursuit({chosen_ball.x, chosen_ball.y - 0.33, M_PI/2})) {
            //     inner_order = 3;
            // }

            if (inner_order == 3 && grip()) {
                color_cache[chosen_ball.color_id]++;

                if (color_cache[chosen_ball.color_id] < 2) {
                    inner_order = 0;
                } else {
                    phase = 2;
                    inner_order = 0;
                }
            }
            break;

        case 2:
            printf("return_and_shoot\n");

            if (inner_order == 0 &&
                pursuit({0.35, note_front_zone[chosen_ball.color_id][1], M_PI})) {
                inner_order = 1;
            }

            if (inner_order == 1 &&
                pursuit({note_front_zone[chosen_ball.color_id][0],
                         note_front_zone[chosen_ball.color_id][1], M_PI})) {
                inner_order = 2;
            }

            if (inner_order == 2 && shoot()) {
                inner_order = 3;
            }

            if (inner_order == 3) {
                int total = color_cache[0] + color_cache[1] + color_cache[2];
                if (total >= 3) {
                    puts("fanfare");
                } else {
                    inner_order = 0;
                    phase = 0;
                }
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