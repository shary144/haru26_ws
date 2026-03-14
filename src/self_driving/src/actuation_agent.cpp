#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"

#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"
#include "my_tf2.hpp"
#include "frame_viz.hpp"
#include "topic.hpp"

#include <vector>
#include <array>
#include <deque>
#include <map>
#include <cmath>
#include <algorithm>

class NavNode : public rclcpp::Node, 
public FrameViz<NavNode>
{
public:
  NavNode()
  : Node("actuation_agent", 
    rclcpp::NodeOptions().allow_undeclared_parameters(true)),
    //Topic
    target_pose(this,"target_pose"), //Listener
    robot_pose_icp(this,"robot_pose_icp"), //get_latest
    motor_cmd_array(this,"/robomas/motor_cmd_array"), //publish
    pursuit_status(this,"/pursuit/status"), //pusblish
    feedback(this,"/robomas/feedback"), //get_latest
    //座標系
    base_link("base_link"),
    lidar("lidar_frame"),
    map("map"),
    motor1("motor1"),
    motor2("motor2"),
    motor3("motor3")
  {
    //target_poseにコールバックつける
    target_pose.listen([this](const auto& msg){
      this->target_callback(msg);
    });

    lidar.set_base(base_link(
      lidar_offset_x,
      lidar_offset_y,
      lidar_offset_yaw));

    //モーターの相対位置設定
    std::array<my_tf2::Frame,3> motors = {motor1, motor2, motor3};
    for (int i=0;i<3;i++){
      double th = std::numbers::pi*(1+i*2)/3;
      motors[i].set_base(base_link(
        L*std::cos(th),
        L*std::sin(th),
        th+std::numbers::pi/2)
      );
    }
    RCLCPP_INFO(get_logger(), "actuation_agent started");
  }

private:
  // クラス要素一覧 ======================
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

  //座標系の宣言
  my_tf2::Frame frame_link;
  my_tf2::Frame lidar;
  my_tf2::Frame map;
  my_tf2::Frame motor1;
  my_tf2::Frame motor2;
  my_tf2::Frame motor3;
  
  //topicラッパークラス
  Topic<std_msgs::msg::Float32MultiArray> robot_pose_icp;
  Topic<self_driving::msg::Target> target_pose;
  Topic<self_driving::msg::TargetStatus> pursuit_status;
  Topic<robomas_interfaces::msg::RobomasPacket> motor_cmd_array;
  Topic<robomas_interfaces::msg::RobomasFrame> feedback;

  std::deque<std::array<double, 3>> history_;

  void target_callback(const self_driving::msg::Target& target_msg)
  {
    uint8_t index = target_msg.index;
    uint8_t mode = target_msg.mode;
    double tx = target_msg.x;
    double ty = target_msg.y;
    double tyaw = target_msg.yaw;

    if (auto icp_msg = robot_pose_icp.use_latest()){
      double x = icp_msg->data[0];
      double y = icp_msg->data[1];
      double yaw = icp_msg->data[2]; 
      // LiDARの位置と向きを考慮してロボットの位置と向きを求める
      this->lidar.set_base(this->map(x,y,yaw));
    } else return;

    if (mode == 0) {
      //足回りpursuit
      my_tf2::Pose tpose{tx,ty,tyaw,this->map};
      move_to(tpose);

    } else if (mode == 1){
      //把持 モーター165回転
      //動作継続
      /*
      if (this->prev_mode == mode){
        if (this->start_time-this->now()) >= rclcpp::durable(5);
      } else {
        this->start_time = this->now();
      }
      
      motor_cmd_array.publish{
        [this](auto &out){
          robomas_interfaces::msg::MotorCommand cmd;
          cmd.id = 5;
          cmd.mode = 2; //位置(角度)制御
          if (auto fbmsg = this->feedback.use_latest())
          else return;
          fbmsg 
          cmd.target = 360*8; //仮
          out.push_back(cmd)
        }
      };*/
      
    } else if (mode == 2){
      //射出
      //control::shoot
    }
    
    pursuit_status.publish([this](auto& status)
    {
      status.index=index;
      status.isend=false;
    });
  }  


  void move_to(my_tf2::Pose& tpose/*map座標系の目標位置になる*/){
      motor_cmd_array.publish(
        [this](auto &out){
          auto tbpose = this->base_link(tpose);
          tbpose.x*=pgain_x;
          tbpose.y*=pgain_y;
          double yaw = tbpose.yaw = normalize(tbpose.yaw)*pgain_theta;

          std::vector<my_tf2::Frame*> motor= {*motor1,*motor2,*motor3};
          float maxv = 0
          float v = 0;
          for (int i=0;i<3;i++)
          {
            robomas_interfaces::msg::MotorCommand cmd;
            cmd.id = i;
            cmd.mode = 1;
            v = std::abs(cmd.target = (*motor[i])(tpose).x + yaw*this->L);
            if (maxv<v) maxv=v;
            my_tf2::Pose pos;
            pos.x = cmd.target;
            pos.frame = motor[i];
            viz_pos(this->map,pos);
            out.motors.push_back(cmd);
          }
        }
      );
    
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