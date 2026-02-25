/////先輩のコード
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <vector>

class PurePursuitNode : public rclcpp::Node {
private:
  // ROS
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // State
  nav_msgs::msg::Path path_;
  bool path_received_{false};

  // Parameters
  double lookahead_distance_;
  double linear_velocity_;
  double goal_tolerance_;
  int path_counter_ = -1;
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
    this->path_counter_ = 0;
    path_received_ = true;

    RCLCPP_INFO(this->get_logger(), "path recieved");
  }

  void controlLoop() {
    geometry_msgs::msg::Twist cmd;

    if (!path_received_ || path_.poses.empty() || this->path_counter_ < 0) {
      cmd_pub_->publish(cmd);
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(path_.header.frame_id, "map",
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", ex.what());
      cmd_pub_->publish(cmd);
      return;
    }

    auto tf_map_to_base =
        tf_buffer_.lookupTransform("livox_frame", "map", tf2::TimePointZero);

    double robot_x = tf_map_to_base.transform.translation.x;
    double robot_y = tf_map_to_base.transform.translation.y;
    tf2::Quaternion q(tf_map_to_base.transform.rotation.x,
                      tf_map_to_base.transform.rotation.y,
                      tf_map_to_base.transform.rotation.z,
                      tf_map_to_base.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(), "[%d] %lf, %lf, %lf", path_counter_, robot_x, robot_y, yaw);


    // --- lookahead point search ---
    // geometry_msgs::msg::PoseStamped target;
    // bool found = false;

    // for (const auto &pose : path_.poses) {
    //   double dx = pose.pose.position.x - robot_x;
    //   double dy = pose.pose.position.y - robot_y;
    //   double dist = std::hypot(dx, dy);

    //   if (dist >= lookahead_distance_) {
    //     target = pose;
    //     found = true;
    //     break;
    //   }
    // }

    // // --- goal check ---
    // const auto &goal_pose = path_.poses.back().pose.position;

    // double dx_goal = goal_pose.x - robot_x;
    // double dy_goal = goal_pose.y - robot_y;
    // double dist_to_goal = std::hypot(dx_goal, dy_goal);

    // if (dist_to_goal < goal_tolerance_) {
    //   geometry_msgs::msg::Twist stop_cmd;
    //   cmd_pub_->publish(stop_cmd);
    //   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    //                        "Goal reached (dist=%.3f)", dist_to_goal);
    //   return;
    // }

    // if (!found) {
    //   cmd_pub_->publish(cmd);
    //   return;
    // }

    // --- transform target to base_link frame ---
    // geometry_msgs::msg::PoseStamped target_bl;
    // tf2::doTransform(target, target_bl, tf);

    // double x = target_bl.pose.position.x;
    // double y = target_bl.pose.position.y;

    // if (x <= 0.0) {
    //   cmd_pub_->publish(cmd);
    //   return;
    // }

    // // --- Pure Pursuit ---
    // double curvature = 2.0 * y / (x * x + y * y);
    // double angular_velocity = linear_velocity_ * curvature;

    // cmd.linear.x = linear_velocity_;
    // cmd.angular.z = angular_velocity;

    // cmd_pub_->publish(cmd);



    std::vector<std::vector<double>> path_list({
      {-1.8, 0.8, 1.55},
      {-3, 0.8, 1.55},
      {-3, 2.5, 1.55},
      {-1.8, 2.5, 1.55},
      {-1.8, 0.8, 1.55}
    });

    double target_x = path_list[path_counter_][0];
    double target_y = path_list[path_counter_][1];
    double target_yaw = path_list[path_counter_][2];

    double dx = (robot_x - target_x);
    double dy = (robot_y - target_y);
    double dyaw = (yaw - target_yaw);

    cmd.linear.x = 5000 * dx;
    cmd.linear.y = 5000 * dy;
    cmd.angular.z = 3500 * dyaw;

    if(abs(cmd.linear.x) > 3000)
    {
      cmd.linear.x = cmd.linear.x < 0 ? -3000 : 3000;
    }

    if(abs(cmd.linear.y) > 3000)
    {
      cmd.linear.y = cmd.linear.y < 0 ? -3000 : 3000;
    }

    if(abs(cmd.angular.z) > 2000)
    {
      cmd.angular.z = cmd.angular.z < 0 ? -2000 : 2000;
    }

    cmd_pub_->publish(cmd);

    if(sqrt(dx * dx + dy * dy) < 0.5)
      this->path_counter_++;

    if(this->path_counter_ >= path_list.size() )
      this->path_counter_ = -1;
  }

public:
  PurePursuitNode()
      : Node("pure_pursuit_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    declare_parameter("lookahead_distance", 0.5);
    declare_parameter("linear_velocity", 0.3);
    declare_parameter("control_frequency", 20.0);
    declare_parameter("goal_tolerance", 0.15);
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();

    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    linear_velocity_ = get_parameter("linear_velocity").as_double();
    double freq = get_parameter("control_frequency").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "path", 10,
        std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / freq)),
        std::bind(&PurePursuitNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Pure Pursuit follower started");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
