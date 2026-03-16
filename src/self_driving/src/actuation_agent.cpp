#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/robomas_frame.hpp"
#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"
#include "my_tf2.hpp"
#include "topic.hpp"   // ← 新設計版 Topic

#include <vector>
#include <array>
#include <deque>
#include <map>
#include <cmath>
#include <algorithm>

class NavNode : public rclcpp::Node {
public:
  NavNode()
  : Node("actuation_agent",
    rclcpp::NodeOptions().allow_undeclared_parameters(true)),
    // Topic（新設計版）
    target_pose(this, "target_pose", TopicMode::Subscriber),
    robot_pose_icp(this, "robot_pose_icp", TopicMode::Subscriber),
    motor_cmd_array(this, "/robomas/motor_cmd_array", TopicMode::Publisher),
    pursuit_status(this, "/pursuit/status", TopicMode::Publisher),
    feedback(this, "/robomas/feedback", TopicMode::Subscriber),
    // 座標系
    base_link("base_link"),
    lidar("lidar_frame"),
    map("map"),
    motor1("motor1"),
    motor2("motor2"),
    motor3("motor3")
  {
    // ---- target_pose の受信 ----
    target_pose.hook([this](const self_driving::msg::Target& msg){
      RCLCPP_INFO(this->get_logger(), "target received: index=%d mode=%d",
                  msg.index, msg.mode);
      // ここで後で move_to や set_base を戻す
    });

    // ---- 座標系の初期化 ----
    lidar.set_base(base_link(
      lidar_offset_x,
      lidar_offset_y,
      lidar_offset_yaw));

    std::array<my_tf2::Frame*,3> motors = {&motor1, &motor2, &motor3};
    for (int i=0;i<3;i++){
      double th = M_PI*(1+i*2)/3;
      motors[i]->set_base(base_link(
        L*std::cos(th),
        L*std::sin(th),
        th+M_PI/2)
      );
    }

    RCLCPP_INFO(get_logger(), "actuation_agent started (new Topic)");
  }

private:
  // ---- 新 Topic ----
  Topic<self_driving::msg::Target> target_pose;
  Topic<std_msgs::msg::Float32MultiArray> robot_pose_icp;
  Topic<robomas_interfaces::msg::RobomasPacket> motor_cmd_array;
  Topic<self_driving::msg::TargetStatus> pursuit_status;
  Topic<robomas_interfaces::msg::RobomasFrame> feedback;

  // ---- 座標系 ----
  my_tf2::Frame base_link;
  my_tf2::Frame lidar;
  my_tf2::Frame map;
  my_tf2::Frame motor1;
  my_tf2::Frame motor2;
  my_tf2::Frame motor3;

  // ---- パラメータ ----
  double lidar_offset_x = 0.286;
  double lidar_offset_y = -0.165;
  double lidar_offset_yaw = -1.047;
  double x_threshold = 0.3;
  double y_threshold = 0.3;
  double yaw_threshold = 0.1;
  double pgain_x = 0.5;
  double pgain_y = 1.0;
  double pgain_theta = 1.0;
  double L= 0.33;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavNode>());
  rclcpp::shutdown();
  return 0;
}