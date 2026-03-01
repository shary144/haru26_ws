#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"

#include <vector>
#include <array>
#include <deque>
#include <map>
#include <cmath>
#include <algorithm>

#define INIT_PARAM(name) \
  this->declare_parameter<decltype(name)>(#name, name); \
  this->get_parameter(#name, name);

class NavNode : public rclcpp::Node
{
public:
  NavNode()
  : Node("my_pure_pursuit_node", 
    rclcpp::NodeOptions().allow_undeclared_parameters(true)
  )
  {

    // ICP 推定結果の購読
    sub_pose_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/robot_pose_icp", 10,
      std::bind(&NavNode::pose_callback, this, std::placeholders::_1));

    // 目標値の購読
    sub_target_ = create_subscription<self_driving::msg::Target>(
      "/target_pose", 10,
      std::bind(&NavNode::target_callback, this, std::placeholders::_1));

    // publisher_cmd
    pub_cmd_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>(
      "/robomas/cmd", 10);

    // 到達ステータスをpublish
    pub_status_ = this->create_publisher<self_driving::msg::TargetStatus>(
      "/pursuit/status", 10);
  }

private:
  // クラス要素一覧 ======================
  double lidar_offset_x = 0.286;
  double lidar_offset_y = -0.165;
  double lidar_offset_yaw = -1.047;
  double nav_radius = 0.3;
  double pgain_x = 0.5;
  double pgain_y = 1.0;
  double pgain_theta = 1.0;
  double L= 0.33;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_;
  rclcpp::Subscription<self_driving::msg::Target>::SharedPtr sub_target_;
  rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr pub_cmd_;
  rclcpp::Publisher<self_driving::msg::TargetStatus>::SharedPtr pub_status_;

  std::deque<std::array<double, 3>> history_; // 履歴用
  self_driving::msg::Target latest_target_;   // 最新の目標値
  bool has_target_ = false;                   // 目標が来たかどうか

  // ==========================
  // ICP callback
  // ==========================
  void pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) return;

    double x = msg->data[0];
    double y = msg->data[1];
    double yaw = msg->data[2];

    // ---- 平均化したい場合はここを ON ----
    // add_history(x, y, yaw);
    // auto [fx, fy, fyaw] = filtered_pose();
    // -------------------------------------

    // LiDARの位置と向きを考慮してロボットの位置と向きを逆算
    double robot_x = x + this->lidar_offset_x * std::cos(yaw) - this->lidar_offset_y * std::sin(yaw);
    double robot_y = y + this->lidar_offset_x * std::sin(yaw) + this->lidar_offset_y * std::cos(yaw);
    double robot_yaw = yaw - this->lidar_offset_yaw;

    // まだ一度も目標を受信していない場合は何もしない
    if (!has_target_) return;

    double tx = this->latest_target_.x;   // 目標値
    double ty = this->latest_target_.y;
    double tyaw = this->latest_target_.yaw;

    navigate(robot_x, robot_y, robot_yaw, tx, ty, tyaw);
  }

  void target_callback(const self_driving::msg::Target::SharedPtr msg)
  {
    latest_target_ = *msg;
    has_target_ = true;
  }

  // ==========================
  // ナビゲーション本体
  // ==========================
  void navigate(double x, double y, double yaw,
                double target_x, double target_y, double target_yaw)
  {
    // 世界座標での誤差
    double ex = target_x - x;
    double ey = target_y - y;
    double e_yaw = normalize_angle(target_yaw - yaw);

    // ロボット座標系へ変換
    double ex_b = std::cos(yaw) * ex + std::sin(yaw) * ey;
    double ey_b = -std::sin(yaw) * ex + std::cos(yaw) * ey;

    // 制御入力（P制御）
    double vx = this->pgain_x * ex_b;
    double vy = this->pgain_y * ey_b;
    double wz = this->pgain_theta * e_yaw;

    // 安定化のための制限
    vx = std::clamp(vx, -0.3, 0.3);
    vy = std::clamp(vy, -0.3, 0.3);
    wz = std::clamp(wz, -1.5, 1.5);

    // 到達判定
    double dist = std::sqrt(ex * ex + ey * ey);
    if (dist < nav_radius) {
      publish_stop();

      // ここで TargetStatus を publish してもいい
      self_driving::msg::TargetStatus status;
      status.status= 1; // 到達
      pub_status_->publish(status);

      return;
    }

    publish_cmd(vx, vy, wz);
  }

  // ==========================
  // 速度 publish
  // ==========================
  void publish_cmd(double vx, double vy, double wz)
  {
    double vw1 = -0.5 * vx + 0.866 * vy + this->L * wz;  // 前左（+60°）
    double vw2 = -0.5 * vx - 0.866 * vy + this->L * wz;  // 前右（-60°）
    double vw3 = 1.0 * vx + this->L * wz;                // 後ろ（180°）

    double max_w = std::max({std::abs(vw1), std::abs(vw2), std::abs(vw3)});
    if (max_w == 0.0) max_w = 1.0; // ゼロ割防止

    if (max_w > 1000.0f) {
      double scale = 1000.0f / max_w;
      vw1 *= scale;
      vw2 *= scale;
      vw3 *= scale;
    }

    auto msg = robomas_interfaces::msg::RobomasPacket();

    // motor1(足回り)
    robomas_interfaces::msg::MotorCommand cmd1;
    cmd1.motor_id = 1;
    cmd1.mode = 1;
    cmd1.target = vw1;
    msg.motors.push_back(cmd1);

    // motor2(足回り)
    robomas_interfaces::msg::MotorCommand cmd2;
    cmd2.motor_id = 2;
    cmd2.mode = 1;
    cmd2.target = vw2;
    msg.motors.push_back(cmd2);

    // motor3(足回り)
    robomas_interfaces::msg::MotorCommand cmd3;
    cmd3.motor_id = 3;
    cmd3.mode = 1;
    cmd3.target = vw3;
    msg.motors.push_back(cmd3);

    pub_cmd_->publish(msg);
  }

  void publish_stop()
  {
    publish_cmd(0.0, 0.0, 0.0);
  }

  // ==========================
  // 角度正規化
  // ==========================
  double normalize_angle(double a)
  {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  // ==========================
  // 平均化（任意）
  // ==========================
  void add_history(double x, double y, double yaw)
  {
    history_.push_back({x, y, yaw});
    if (history_.size() > 10) history_.pop_front();//ここははみだしを消す処理
  }

  std::array<double, 3> filtered_pose()
  {
    double sx = 0, sy = 0, syaw = 0;
    for (auto &p : history_) {
      sx += p[0];
      sy += p[1];
      syaw += p[2];
    }
    size_t n = history_.size();
    return {sx / n, sy / n, syaw / n};
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavNode>());
  rclcpp::shutdown();
  return 0;
}