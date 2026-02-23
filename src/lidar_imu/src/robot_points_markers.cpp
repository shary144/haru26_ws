#include <memory>
#include <cmath>
#include <mutex>
#include <functional>   // ★追加（std::bind / placeholders）

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "builtin_interfaces/msg/duration.hpp"

using namespace std::chrono_literals;

static double yaw_from_quat(double qw, double qx, double qy, double qz)
{
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

class LidarAndRobotVizNode : public rclcpp::Node
{
public:
  LidarAndRobotVizNode()
  : rclcpp::Node("lidar_and_robot_viz_node"),
    last_points_pub_time_(this->now()),
    min_points_period_sec_(1.0 / 30.0),   // 点群は最大30Hz
    x_(0.0), y_(0.0), z_(0.0), th_(0.0),
    qw_(1.0), qx_(0.0), qy_(0.0), qz_(0.0),
    has_pose_(false)
  {
    // robot_pose: [x,y,z,qw,qx,qy,qz]
    sub_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "robot_pose", 10,
      std::bind(&LidarAndRobotVizNode::poseCallback, this, std::placeholders::_1));

    // lidar_points: Float32MultiArray (rows=N, cols=2) 想定
    sub_points_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "lidar_points", 10,
      std::bind(&LidarAndRobotVizNode::pointsCallback, this, std::placeholders::_1));

    pub_points_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "lidar_points_marker", 10);

    pub_robot_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "robot_marker", 10);

    // ロボットMarkerはタイマーで30Hz
    timer_ = this->create_wall_timer(
      33ms, std::bind(&LidarAndRobotVizNode::onTimerRobotMarker, this));

    RCLCPP_INFO(this->get_logger(), "LidarAndRobotVizNode started.");
  }

private:
  void poseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // ★7要素必要
    if (msg->data.size() < 7) {
      RCLCPP_WARN(this->get_logger(), "robot_pose data size < 7 (need [x,y,z,qw,qx,qy,qz])");
      return;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    x_  = msg->data[0];
    y_  = msg->data[1];
    z_  = msg->data[2];
    qw_ = msg->data[3];
    qx_ = msg->data[4];
    qy_ = msg->data[5];
    qz_ = msg->data[6];

    // ★変数名修正
    th_ = yaw_from_quat(qw_, qx_, qy_, qz_);
    has_pose_ = true;
  }

  void pointsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const rclcpp::Time now = this->now();

    // 点群は最大30Hzに制限
    if ((now - last_points_pub_time_).seconds() < min_points_period_sec_) {
      return;
    }
    last_points_pub_time_ = now;

    const std::size_t n = msg->data.size();
    if (n < 2) return;

    // rows, cols を layout から取得（なければ cols=2 とみなす）
    int rows = 0;
    int cols = 0;
    if (msg->layout.dim.size() >= 2) {
      rows = static_cast<int>(msg->layout.dim[0].size);
      cols = static_cast<int>(msg->layout.dim[1].size);
    } else {
      cols = 2;
      rows = static_cast<int>(n / 2);
    }

    if (cols < 2) {
      RCLCPP_WARN(this->get_logger(), "lidar_points cols (%d) < 2", cols);
      return;
    }
    if (rows * cols > static_cast<int>(n)) {
      RCLCPP_WARN(this->get_logger(), "layout rows*cols (%d) > data size (%zu)", rows * cols, n);
      return;
    }

    double x, y, th;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_pose_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "No robot_pose received yet, skip lidar visualization");
        return;
      }
      x  = x_;
      y  = y_;
      th = th_;
    }

    const double c = std::cos(th);
    const double s = std::sin(th);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now;
    marker.header.frame_id = "map";
    marker.ns = "lidar_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = builtin_interfaces::msg::Duration(); // 0 -> 消えない
    marker.points.reserve(rows);

    for (int i = 0; i < rows; ++i) {
      const int base = i * cols;
      const double xr = msg->data[base + 0];
      const double yr = msg->data[base + 1];

      const double xw = x + c * xr - s * yr;
      const double yw = y + s * xr + c * yr;

      geometry_msgs::msg::Point p;
      p.x = xw;
      p.y = yw;
      p.z = 0.1;
      marker.points.push_back(p);
    }

    pub_points_marker_->publish(marker);
  }

  void onTimerRobotMarker()
  {
    double x, y, z, qw, qx, qy, qz;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_pose_) return;
      x  = x_;
      y  = y_;
      z  = z_;
      qw = qw_;
      qx = qx_;
      qy = qy_;
      qz = qz_;
    }

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->get_clock()->now();
    m.ns = "robot";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    // ★ローカル変数で統一（姿勢が混ざらない）
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;

    m.pose.orientation.x = qx;
    m.pose.orientation.y = qy;
    m.pose.orientation.z = qz;
    m.pose.orientation.w = qw;

    m.scale.x = 0.08;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    m.color.r = 0.0f;
    m.color.g = 0.8f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;

    pub_robot_marker_->publish(m);
  }

  std::mutex mtx_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_points_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr     pub_points_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr     pub_robot_marker_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ★これが無いとコンストラクタ初期化でコンパイルエラー
  rclcpp::Time last_points_pub_time_;
  double min_points_period_sec_;

  double x_, y_, z_;
  double th_;
  double qw_, qx_, qy_, qz_;
  bool has_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarAndRobotVizNode>());
  rclcpp::shutdown();
  return 0;
}
