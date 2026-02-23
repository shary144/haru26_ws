#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "box_data.hpp"

using namespace std::chrono_literals;

class BoxMarkerPublisher : public rclcpp::Node
{
public:
    BoxMarkerPublisher()
        : rclcpp::Node("box_marker_publisher", rclcpp::NodeOptions{})
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "world_markers", 10);

        timer_ = this->create_wall_timer(
            1s, std::bind(&BoxMarkerPublisher::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "BoxMarkerPublisher started");
    }

private:
    void on_timer()
    {
        visualization_msgs::msg::MarkerArray msg;

        const std::string frame_id = "map";
        const rclcpp::Time now = this->get_clock()->now();

        int id = 0;
        for (const auto & b : BOX_LIST)
        {
            visualization_msgs::msg::Marker m;
            m.header.stamp = now;
            m.header.frame_id = frame_id;

            m.ns  = "boxes";
            m.id  = id++;
            m.type   = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;

            // 位置
            m.pose.position.x = b.x;
            m.pose.position.y = b.y;
            m.pose.position.z = b.z;

            // 姿勢
            m.pose.orientation.x = b.qx;
            m.pose.orientation.y = b.qy;
            m.pose.orientation.z = b.qz;
            m.pose.orientation.w = b.qw;

            // 大きさ
            m.scale.x = b.length_x;
            m.scale.y = b.length_y;
            m.scale.z = b.length_z;

            // 色
            m.color.r = b.r;
            m.color.g = b.g;
            m.color.b = b.b;
            m.color.a = b.a;

            // 消えない Marker（timer で上書きされる）
            m.lifetime = rclcpp::Duration(0, 0);

            msg.markers.push_back(m);
        }

        // RCLCPP_INFO(this->get_logger(), "Publishing %zu markers", msg.markers.size());
        marker_pub_->publish(msg);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxMarkerPublisher>());
    rclcpp::shutdown();
    return 0;
}
