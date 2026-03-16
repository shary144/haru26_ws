#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"
#include <cmath>
#include <cassert>
#include "topic.hpp"

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode()
    : Node("test_node"),
      target_pose(this, "target_pose", TopicMode::Publisher)   // ★ 新API：mode 必須
    {
        // 500msごとに publish
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ManagerNode::publish_once, this)
        );

        // publisher の hook（メッセージを埋める処理）
        target_pose.hook([this](auto& msg){
            msg.index = 0;
            msg.mode = 0;
            msg.x = 1.0;
            msg.y = 2.0;
            msg.yaw = 0.5;

            RCLCPP_INFO(this->get_logger(),
                "Publishing target: index=%d x=%.2f y=%.2f yaw=%.2f",
                msg.index, msg.x, msg.y, msg.yaw);
        });
    }

private:
    void publish_once()
    {
        // ★ 新API：publish() は引数なし
        target_pose.publish();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    Topic<self_driving::msg::Target> target_pose;  // ★ 新API：Publisher
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManagerNode>());
    rclcpp::shutdown();
    return 0;
}