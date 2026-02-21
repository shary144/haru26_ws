#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "ball_detector/msg/ball.hpp"

class BallColorMarkerNode : public rclcpp::Node {
public:
    BallColorMarkerNode() : Node("ball_color_marker_node")
    {
        sub_ = this->create_subscription<ball_detector::msg::Ball>(
            "/ball_position", 10,
            std::bind(&BallColorMarkerNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/ball_marker", 10);
        RCLCPP_INFO(this->get_logger(), "Ball Color Marker Node started.");
    }

private:
    void callback(const ball_detector::msg::Ball::SharedPtr msg)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = msg->header.frame_id;   // "robot_base"
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "ball_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // --- 座標 ---
        marker.pose.position.x = msg->position.x;
        marker.pose.position.y = msg->position.y;
        marker.pose.position.z = msg->position.z;
        marker.pose.orientation.w = 1.0;

        // --- 球のサイズ（直径0.30m ＝ 半径15cm） ---
        marker.scale.x = 0.30;
        marker.scale.y = 0.30;
        marker.scale.z = 0.30;

        // --- 色を反映 ---
        if (msg->color == "red") {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.9;
        } else if (msg->color == "blue") {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.9;
        } else if (msg->color == "yellow") {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.9;
        } else {
            // 不明色 → 白
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.9;
        }

        pub_->publish(marker);
    }

    rclcpp::Subscription<ball_detector::msg::Ball>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallColorMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
