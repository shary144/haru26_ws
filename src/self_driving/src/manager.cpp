#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"
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

        color_cache = {0,0,0};
    }

private:
    // ROS I/F
    rclcpp::Subscription<self_driving::msg::TargetStatus>::SharedPtr sub_status_;
    rclcpp::Publisher<self_driving::msg::Target>::SharedPtr          pub_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_icp_;

    self_driving::msg::TargetStatus status_msg_;
    uint8_t inner_order = 0;
    int phase = 0;

    std::array<int,3> color_cache; // 色ごとの取得数

    ballcache::Ball chosen_ball;   // ← BallCache.hpp の Ball を使う

    double note_front_zone[3][2] =
    {
        {1.362, 3.5},   // blue
        {1.8,   1.85},  // yellow
        {0.924, 5.112}  // red
    };

    // ==========================
    // オートマトン
    // ==========================
    void automaton(const std_msgs::msg::Float32MultiArray::SharedPtr /*msg*/)
    {
        switch (phase) {

        case 0:
            printf("start\n");
            if (pursuit({0.35, 5.888, M_PI/2})) {
                phase = 1;
                inner_order = 0;
            }
            break;

        case 1:
            printf("catch_ball\n");

            if (inner_order == 0) {
                choose_ball();
                inner_order = 1;
            }

            if (inner_order == 1 &&
                pursuit({chosen_ball.x, 5.888, M_PI/2})) {
                inner_order = 2;
            }

            if (inner_order == 2 &&
                pursuit({chosen_ball.x, chosen_ball.y - 0.33, M_PI/2})) {

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

    // ==========================
    // ボール選択
    // ==========================
    void choose_ball()
    {
        // TODO: BallCache から選ぶ
        chosen_ball.x = 0.35;
        chosen_ball.y = 5.5;
        chosen_ball.color_id = 0;
    }

    void update_status(const self_driving::msg::TargetStatus::SharedPtr msg)
    {
        status_msg_ = *msg;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManagerNode>());
    rclcpp::shutdown();
    return 0;
}