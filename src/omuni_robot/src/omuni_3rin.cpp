#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "robomas_package_2/msg/motor_cmd_array.hpp"
#include "robomas_package_2/msg/motor_cmd.hpp"
#include "robomas_package_2/msg/ems.hpp"

// 以下標準ライブラリ.
#include <vector>
#include <array>
#include <cmath>
//#include <numbers> C++20以降の機能なので使えない
#include <map>

class Omuni3Rin : public rclcpp::Node
{
public:
    Omuni3Rin() : Node("omuni_3rin"){
        sub_joy_   = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Omuni3Rin::joy_callback, this, std::placeholders::_1));
        pub_motor_ = this->create_publisher<robomas_package_2::msg::MotorCmdArray>("motor_cmd_array", 10);
        pub_ems_   = this->create_publisher<robomas_package_2::msg::Ems>("ems_tx", 10);
    }

private:

/*
    const double PI = std::numbers::pi;
    const double PI = 3.14159265;
    // 機械的パラメータ
    std::array<double,3> motor_id = {1,3,4};
    std::array<double,3> wheel_angles = {PI/3,-PI,-PI/3};

    std::array<std::map<std::string,double>,3> wheel_velocities(double x,double y){
        double d = std::sqrt(x*x+y*y);
        double e_x = x/d;
        double e_y = y/d;
        int i=0;
        std::array<std::map<std::string, double>,3> arr;
        std::map<std::string, double> m;
        for (const double angle: wheel_angles){
            std::array<double,2> e_theta = {std::cos(angle+PI/2),std::sin(angle+PI/2)};
            m["value"] = e_theta[0]*e_x+e_theta[1]*e_y;
            m["id"] = motor_id[i];
            arr[i++] = m;
        }
        return arr;
    }
*/
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        if(msg->buttons[6]){
            // 非常停止モードへ
            robomas_package_2::msg::Ems EMS;
            EMS.ems_stop = true;
            pub_ems_->publish(EMS);

            RCLCPP_INFO(this->get_logger(), "ems true");
        }
        else if(msg->buttons[7]){
            // 駆動モードへ
            robomas_package_2::msg::Ems EMS;
            EMS.ems_stop = false;
            pub_ems_->publish(EMS);

            RCLCPP_INFO(this->get_logger(), "ems false");
        }
        
        robomas_package_2::msg::MotorCmdArray out;

        robomas_package_2::msg::MotorCmd cmd;

// [0] 左スティック X軸
//     左: -1 / 右: +1
// [1] 左スティック Y軸
//     上: -1 / 下: +1
// です。READMEにもメモっておきました。
/*        double x = msg->axes[0]*1000.0f; //左スティックX軸(右向きを正)
        double y = -(msg->axes[1]*1000.0f); //左スティックY軸(上向きを正としている)
        
        for (auto wheel: wheel_velocities(x,y)){
            cmd.id = wheel["id"];
            cmd.mode = 2;
            cmd.value = wheel["value"];
            out.cmds.push_back(cmd);
        }
        
        if(msg->buttons[0] && !msg->buttons[1]){
            cmd.id = 2;
            cmd.mode = 1;
            cmd.value = 1000.0f;
            out.cmds.push_back(cmd);
        }

        if (msg->buttons[3]) {
            cmd.id = 2;
            cmd.mode = 1;
            cmd.value = 0.0f;
            out.cmds.push_back(cmd);
        }

        

*/

        float cos = msg->axes[0]; //左スティックX
        float sin = -(msg->axes[1]); //左スティックY

        //とりますべて速度制御(mode = 1)

        //motor1
        cmd.id = 1;
        cmd.mode = 1;
        cmd.value = (-0.5f * cos + 0.866f * sin) * 1000.0f;
        out.cmds.push_back(cmd);

        //motor2
        cmd.id = 2;
        cmd.mode = 1;
        cmd.value = (-0.5f * cos - 0.866f * sin) * 1000.0f;
        out.cmds.push_back(cmd);

        //motor3
        cmd.id = 3;
        cmd.mode = 1;
        cmd.value = cos * 1000.0f;
        out.cmds.push_back(cmd);

      /*  if(msg->buttons[1] && !msg->buttons[0]){
            cmd.id = 4;
            cmd.mode = 2;
            cmd.value = 0.0f;
            out.cmds.push_back(cmd);
        }

        if(msg->buttons[0] && msg->buttons[1]){
            cmd.id = 4;
            cmd.mode = 2;
            cmd.value = 15.0f;
            out.cmds.push_back(cmd);
        } */
        //ここまで自由記述

        pub_motor_->publish(out);

        if(!out.cmds.empty()){
            RCLCPP_INFO(this->get_logger(), "m1=%.1f", out.cmds[0].value);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<robomas_package_2::msg::MotorCmdArray>::SharedPtr pub_motor_;
    rclcpp::Publisher<robomas_package_2::msg::Ems>::SharedPtr pub_ems_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Omuni3Rin>());
    rclcpp::shutdown();
    return 0;
}