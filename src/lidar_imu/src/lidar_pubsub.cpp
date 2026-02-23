#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class LidarScanToPointsNode : public rclcpp::Node
{
public:
  LidarScanToPointsNode()
  : rclcpp::Node("lidar_scan_to_points_node")
  {
    // パラメータ（必要ならコード直書きでもよいが、最低限ここだけ用意）
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("out_topic",  "lidar_points");
    this->declare_parameter<double>("range_min_override", -1.0); // <0ならLaserScanのrange_minを使う
    this->declare_parameter<double>("range_max_override", -1.0); // <0ならLaserScanのrange_maxを使う

    scan_topic_ = this->get_parameter("scan_topic").as_string();
    out_topic_  = this->get_parameter("out_topic").as_string();
    range_min_override_ = this->get_parameter("range_min_override").as_double();
    range_max_override_ = this->get_parameter("range_max_override").as_double();

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarScanToPointsNode::scanCallback, this, std::placeholders::_1));

    pub_points_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(out_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
      "Subscribe: %s (sensor_msgs/LaserScan), Publish: %s (Float32MultiArray)",
      scan_topic_.c_str(), out_topic_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double range_min = (range_min_override_ >= 0.0) ? range_min_override_ : msg->range_min;
    const double range_max = (range_max_override_ >= 0.0) ? range_max_override_ : msg->range_max;

    // 有効点数を先に数える（NaN/Infや範囲外を除外）
    int valid = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < range_min || r > range_max) continue;
      valid++;
    }
    if (valid == 0) {
      return;
    }

    std_msgs::msg::Float32MultiArray out;
    out.layout.dim.resize(2);
    out.layout.dim[0].label  = "rows";
    out.layout.dim[0].size   = static_cast<uint32_t>(valid);
    out.layout.dim[0].stride = static_cast<uint32_t>(valid * 2);

    out.layout.dim[1].label  = "cols";
    out.layout.dim[1].size   = 2;
    out.layout.dim[1].stride = 2;

    out.data.resize(static_cast<size_t>(valid) * 2);

    // LaserScan -> (x,y) 変換（laser座標系）
    // angle = angle_min + i * angle_increment
    int idx = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < range_min || r > range_max) continue;

      const double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const double x = static_cast<double>(r) * std::cos(-a);
      // const double y = static_cast<double>(r) * std::sin(a);


      // ===== ここが修正点 =====
      // 元: const double y = static_cast<double>(r) * std::sin(a);
      const double y = static_cast<double>(r) * std::sin(-a);  // CHANGED: LiDAR逆さま対応で y を反転
      // =======================


      out.data[idx++] = static_cast<float>(x);
      out.data[idx++] = static_cast<float>(y);
    }

    pub_points_->publish(out);
  }

  std::string scan_topic_;
  std::string out_topic_;
  double range_min_override_{-1.0};
  double range_max_override_{-1.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_points_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarScanToPointsNode>());
  rclcpp::shutdown();
  return 0;
}
