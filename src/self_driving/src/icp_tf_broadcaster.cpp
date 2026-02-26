// pure_pursuitのベースがtf2だったのでtopicとの橋渡しをさせるためのノード
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class IcpTfBroadcaster : public rclcpp::Node
{
public:
  IcpTfBroadcaster()
  : Node("icp_tf_broadcaster")
  {
    broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/robot_pose_icp", 10,
      std::bind(&IcpTfBroadcaster::callback,
                this, std::placeholders::_1));
  }

private:
  void callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) return;

    double x   = msg->data[0];
    double y   = msg->data[1];
    double yaw = msg->data[2];

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "map";
    t.child_frame_id  = "livox_frame";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}