#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/robomas_frame.hpp"
#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"
#include "Shoot.hpp"

#include <vector>
#include <array>
#include <deque>
#include <cmath>
#include <algorithm>

class NavNode : public rclcpp::Node
{
public:
  NavNode()
  : Node("my_pure_pursuit_node",
         rclcpp::NodeOptions().allow_undeclared_parameters(true))
  {
    sub_pose_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "robot_pose_icp", 10,
      std::bind(&NavNode::pose_callback, this, std::placeholders::_1));

    sub_target_ = create_subscription<self_driving::msg::Target>(
      "/target_pose", 10,
      std::bind(&NavNode::target_callback, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>(
      "/robomas/cmd", 10);

    pub_status_ = this->create_publisher<self_driving::msg::TargetStatus>(
      "/pursuit/status", 10);

    sub_feedback_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
      "/robomas/feedback", 10,
      std::bind(&NavNode::update_feedback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "my_pure_pursuit_node started");
  }

private:
  // パラメータ・状態
  double lidar_offset_x   = 0.286;
  double lidar_offset_y   = -0.165;
  double lidar_offset_yaw = -2.094;
  double nav_radius       = 0.1;
  double pgain_x          = 1.0;
  double pgain_y          = 1.0;
  double pgain_theta      = 1.0;
  double L                = 0.33;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_;
  rclcpp::Subscription<self_driving::msg::Target>::SharedPtr        sub_target_;
  rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr pub_cmd_;
  rclcpp::Publisher<self_driving::msg::TargetStatus>::SharedPtr     pub_status_;
  rclcpp::Subscription<robomas_interfaces::msg::RobomasFrame>::SharedPtr sub_feedback_;

  std::deque<std::array<double, 3>> history_;
  self_driving::msg::Target         latest_target_;
  robomas_interfaces::msg::RobomasFrame feedback_msg;
  bool   has_target_ = false;
  int    prev_mode   = -1;
  bool   mode_init_  = true;

  angleControl shooter_;

  // 把持・昇降用状態
  double motor5_th_now = 0.0; // 昇降
  double motor6_th_now = 0.0; // 把持

  bool closed  = true;
  bool closing = false;
  bool opened  = false;
  bool opening = false;
  double m6_th_tgt = 0.0;

  bool downed  = true;
  bool downing = false;
  bool uped    = false;
  bool uping   = false;
  double m5_th_tgt = 199.0;

  // ==========================
  // ICP callback
  // ==========================
  void pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!has_target_) return;

    // mode による分岐
    if (latest_target_.mode == 1) {
      shoot();
      return;
    } else if (latest_target_.mode == 2) {
      grip();
      return;
    }
    else {
      grip_setup();
      return;
    }

    if (msg->data.size() < 3) return;

    double x   = msg->data[0];
    double y   = msg->data[1];
    double yaw = msg->data[2];

    double robot_x   = x - 0.3;
    double robot_y   = y ;
    double robot_yaw = yaw + 2.0/3.0 * M_PI;

    printf("robot_body_pose:%lf,%lf,%lf\n", robot_x, robot_y, robot_yaw);
    printf("lidar_pose:%lf,%lf,%lf\n", x, y, yaw);

    double tx   = latest_target_.x;
    double ty   = latest_target_.y;
    double tyaw = latest_target_.yaw;

    navigate(robot_x, robot_y, robot_yaw, tx, ty, tyaw);
  }

  void target_callback(const self_driving::msg::Target::SharedPtr msg)
  {
    mode_init_ = (prev_mode != msg->mode);
    prev_mode  = msg->mode;

    latest_target_ = *msg;
    has_target_    = true;
  }

  void update_feedback(const robomas_interfaces::msg::RobomasFrame::SharedPtr msg)
  {
    feedback_msg = *msg;
    // 角度インデックスは環境に合わせて調整
    // ここでは仮に angle[4] が motor5, angle[5] が motor6 とする
    if (feedback_msg.angle.size() > 5) {
      motor5_th_now = feedback_msg.angle[4];
      motor6_th_now = feedback_msg.angle[5];
    }
  }

  // ==========================
  // ナビゲーション本体
  // ==========================
  void navigate(double robot_x, double robot_y, double robot_yaw,
                double target_x, double target_y, double target_yaw)
  {
    double ex    = target_x - robot_x;
    double ey    = target_y - robot_y;
    double e_yaw = normalize_angle(target_yaw - robot_yaw);

    printf("e: %lf,%lf,%lf\n", ex, ey, e_yaw);

    double ex_b = std::cos(robot_yaw) * ex + std::sin(robot_yaw) * ey;
    double ey_b = -std::sin(robot_yaw) * ex + std::cos(robot_yaw) * ey;

    printf("robot_yaw:%lf\n", robot_yaw);

    double th_margin = 0.2;
    double er        = std::sqrt(ex * ex + ey * ey);
    double vx        = pgain_x * ex_b;
    double vy        = pgain_y * ey_b;
    double wz        = pgain_theta * e_yaw;

    self_driving::msg::TargetStatus status;
    status.index = latest_target_.index;

    if (er < nav_radius) vx = vy = 0.0;
    if (std::abs(e_yaw) <= th_margin) wz = 0.0;

    if (er < nav_radius && std::abs(e_yaw) <= th_margin) {
      status.status = true;
      printf("END\n");
      // 田巻
      publish_cmd(0., 0., 0.);
      has_target_ = false;
      // 田巻
      pub_status_->publish(status);
      printf("vx,vy,wz=%lf,%lf,%lf\n", vx, vy, wz);
    } else {
      printf("vx,vy,wz=%lf,%lf,%lf\n", vx, vy, wz);
      publish_cmd(vx, vy, wz);  // status.status = false;
    }
    // 田巻
    // pub_status_->publish(status);
  }

  // ==========================
  // 速度 publish
  // ==========================
  void publish_cmd(double vx, double vy, double wz)
  {
    double vw1 = -(0.866 * vx + 0.5 * vy + L * wz);
    double vw2 = -(-0.866 * vx + 0.5 * vy + L * wz);
    double vw3 = -(-1.0 * vy + L * wz);

    double stuck_v = 0.0;
    double scale   = 1000.0;
    std::array<double, 3> v{vw1 * scale, vw2 * scale, vw3 * scale};

    for (int i = 0; i < 3; i++)
      if (std::abs(stuck_v) < std::abs(v[i])) stuck_v = v[i];

    if (std::abs(stuck_v) < 1e-4) {
      printf("%lf\n", stuck_v);
      stuck_v = 0.0;
    } else {
      for (int i = 0; i < 3; i++)
        v[i] /= stuck_v;

      stuck_v = std::clamp((float)stuck_v, -1000.0f, 1000.0f);
    }

    auto msg = robomas_interfaces::msg::RobomasPacket();

    robomas_interfaces::msg::MotorCommand cmd1;
    cmd1.motor_id = 1;
    cmd1.mode     = 1;
    cmd1.target   = stuck_v * v[0];
    msg.motors.push_back(cmd1);

    robomas_interfaces::msg::MotorCommand cmd2;
    cmd2.motor_id = 2;
    cmd2.mode     = 1;
    cmd2.target   = stuck_v * v[1];
    msg.motors.push_back(cmd2);

    robomas_interfaces::msg::MotorCommand cmd3;
    cmd3.motor_id = 3;
    cmd3.mode     = 1;
    cmd3.target   = stuck_v * v[2];
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
    while (a > M_PI)  a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  // ==========================
  // 射出
  // ==========================
  void shoot()
  {
    auto msg = robomas_interfaces::msg::RobomasPacket();
    robomas_interfaces::msg::MotorCommand cmd;
    self_driving::msg::TargetStatus status_msg;

    cmd.motor_id   = 4;
    status_msg.index = latest_target_.index;

    status_msg.status = shooter_.motor(
      feedback_msg,
      cmd,
      3500,          // ta_angle
      360 * 18 * 9,  // ta_v
      false,         // negfrag
      mode_init_     // init
    );

    mode_init_ = false;

    msg.motors.push_back(cmd);
    pub_cmd_->publish(msg);
    std::cout << "SHOOT END!!!!!!!!!!!!!!!!" << std::endl;
    pub_status_->publish(status_msg);
    // まずそう
    // 田巻
    has_target_ = false;
  }

  // ==========================
  // 把持・昇降ユーティリティ
  // ==========================
  bool haji_open()
  {
    if (closed) {
      auto msg = robomas_interfaces::msg::RobomasPacket();
      robomas_interfaces::msg::MotorCommand cmd6;
      cmd6.motor_id = 6;
      cmd6.mode     = 2;
      m6_th_tgt     = motor6_th_now - 50000.0;
      cmd6.target   = m6_th_tgt;
      msg.motors.push_back(cmd6);
      pub_cmd_->publish(msg);
      opening = true;
      closed  = false;
    }

    if (motor6_th_now < m6_th_tgt - 10.0 || motor6_th_now > m6_th_tgt + 10.0) {
      return false;
    } else if (opening) {
      opening = false;
      opened  = true;
      std::cout << "Hazi Opening end!!!!!!!!!!!!!!!!!!" << std::endl;
      return true;
    } else {
      std::cout << "Unreachable!!!!!!!!!!!!!!!!!!" << std::endl;
      throw 0;
      return true;
    }
  }

  bool haji_close()
  {
    if (opened) {
      auto msg = robomas_interfaces::msg::RobomasPacket();
      robomas_interfaces::msg::MotorCommand cmd6;
      cmd6.motor_id = 6;
      cmd6.mode     = 2;
      m6_th_tgt     = motor6_th_now + 50000.0;
      cmd6.target   = m6_th_tgt;
      msg.motors.push_back(cmd6);
      pub_cmd_->publish(msg);
      closing = true;
      opened  = false;
    }

    if (motor6_th_now < m6_th_tgt - 10.0 || motor6_th_now > m6_th_tgt + 10.0) {
      return false;
    } else if (closing) {
      closing = false;
      closed  = true;
      return true;
    } else {
      return true;
    }
  }

  bool up()
  {
    if (downed) {
      auto msg = robomas_interfaces::msg::RobomasPacket();
      robomas_interfaces::msg::MotorCommand cmd5;
      cmd5.motor_id = 5;
      cmd5.mode     = 2;
      m5_th_tgt     = motor5_th_now + 85000.0;
      cmd5.target   = m5_th_tgt;
      msg.motors.push_back(cmd5);
      pub_cmd_->publish(msg);
      uping  = true;
      downed = false;
    }

    if (motor5_th_now < m5_th_tgt - 10.0 || motor5_th_now > m5_th_tgt + 10.0) {
      return false;
    } else if (uping) {
      uping = false;
      uped  = true;
      return true;
    } else {
      return true;
    }
  }

  bool down()
  {
    std::cout << "in down()" << std::endl;
    if (uped) {
      auto msg = robomas_interfaces::msg::RobomasPacket();
      robomas_interfaces::msg::MotorCommand cmd5;
      cmd5.motor_id = 5;
      cmd5.mode     = 2;
      m5_th_tgt     = motor5_th_now - 85000.0;
      cmd5.target   = m5_th_tgt;
      msg.motors.push_back(cmd5);
      pub_cmd_->publish(msg);
      downing = true;
      uped    = false;
    }
    std::cout << "down: motor5_th_now: " << motor5_th_now << std::endl;
    if (motor5_th_now < m5_th_tgt - 200.0 || motor5_th_now > m5_th_tgt + 10.0) {
      return false;
    } else if (downing) {
      downing = false;
      downed  = true;
      return true;
    } else {
      return true;
    }
  }

  // ==========================
  // 把持シーケンス
  // ==========================
  bool grip()
  {
    // ここでは「開く→下げる→閉じる→上げる」など、
    // 実際のシーケンスに合わせて組む。
    // 例として「開く→下げる→閉じる→上げる」を書く。

    static int grip_phase = 0;

    bool done_open = false;
    bool done_down = false;
    bool done_close = false;
    bool done_up = false;

    switch (grip_phase) {
    case 0:
      done_open = haji_open();
      if (done_open) grip_phase = 1;
      break;
    case 1:
      done_down = down();
      if (done_down) grip_phase = 2;
      break;
    case 2:
      done_close = haji_close();
      if (done_close) grip_phase = 3;
      break;
    case 3:
      if (haji_open()) grip_phase = 4;
      break;
    case 4:
      done_up = up();
      if (done_up) grip_phase = 5;
      break;
    default:
      break;
    }

    if (grip_phase == 5) {
      self_driving::msg::TargetStatus status_msg;
      status_msg.index  = latest_target_.index;
      status_msg.status = true;
      std::cout << "Grip END!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
      pub_status_->publish(status_msg);
      grip_phase = 0;
      // 田巻
      has_target_ = false;
      return true;
    }

    return false;
  }

  // 田巻
  void grip_setup() {
    static int setup_step = 0;

    switch(setup_step) {
      case 0:
      if(haji_open()) setup_step = 1;
      break;

      case 1:
      if(up()) setup_step = 2;
      break;
    }

    if(setup_step == 2) {
      self_driving::msg::TargetStatus status_msg;
      status_msg.status = true;
      std::cout << "setup END!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
      pub_status_->publish(status_msg);
      setup_step = 0;
      has_target_ = false;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavNode>());
  rclcpp::shutdown();
  return 0;
}