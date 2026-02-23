#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>   // M_PI
#include <memory>  // shared_ptr
#include <functional>

class ImuRepublisher : public rclcpp::Node
{
public:
  ImuRepublisher()
  : Node("imu_republisher"), count_(0)
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&ImuRepublisher::imu_callback, this, std::placeholders::_1));

    array_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("imu_array", 10);

    RCLCPP_INFO(this->get_logger(), "IMU Republisher started (use 1/3).");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // ---- 1/3 に間引く（3回に1回だけ使う）----
    ++count_;
    if (count_ % 3 != 0) {
      return;
    }

    std_msgs::msg::Float32MultiArray array_msg;
    array_msg.data.resize(6);

    // 加速度（m/s^2)
    array_msg.data[0] = static_cast<float>(-9.80665 * msg->linear_acceleration.y + 0.02);
    array_msg.data[1] = static_cast<float>( 9.80665 * msg->linear_acceleration.x + 0.02);
    array_msg.data[2] = static_cast<float>( 9.80665 * msg->linear_acceleration.z - 0.05);

    // 角速度（rad/s ）
    array_msg.data[3] = static_cast<float>(-(msg->angular_velocity.y + 1.0)    * M_PI / 180.0);
    array_msg.data[4] = static_cast<float>(( msg->angular_velocity.x + 0.6555) * M_PI / 180.0);
    array_msg.data[5] = static_cast<float>( (msg->angular_velocity.z - 2.366 ) * M_PI / 180.0);

    array_pub_->publish(array_msg);

    // RCLCPP_INFO(this->get_logger(),
    //   "Published (1/3) [ax=%8.3f, ay=%8.3f, az=%8.3f, gx=%8.3f, gy=%8.3f, gz=%8.3f]",
    //   array_msg.data[0], array_msg.data[1], array_msg.data[2],
    //   array_msg.data[3], array_msg.data[4], array_msg.data[5]);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr array_pub_;
  std::size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRepublisher>());
  rclcpp::shutdown();
  return 0;
}
