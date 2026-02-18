#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <sensor_msgs/msg/joy.hpp>
// カスタムメッセージのインポート
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"

using namespace std::chrono_literals;

class MyRobotNode : public rclcpp::Node {
public:
    MyRobotNode() : Node("my_robot_node") {
        // Publisherの作成
        sub_joy_   = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&MyRobotNode::joy_callback, this, std::placeholders::_1));        
        pub_motor_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joyinfo){
        auto msg = robomas_interfaces::msg::RobomasPacket();

        float cos = joyinfo->axes[0]; //左スティックX
        float sin = -(joyinfo->axes[1]); //左スティックY

        //とりますべて速度制御(mode = 1)

        //motor1
        robomas_interfaces::msg::MotorCommand cmd1;
        cmd1.motor_id = 1;
        cmd1.mode = 1;
        cmd1.target = (-0.5f * cos + 0.866f * sin) * 1000.0f;
        msg.motors.push_back(cmd1);

        //motor2
        robomas_interfaces::msg::MotorCommand cmd2;
        cmd2.motor_id = 2;
        cmd2.mode = 1;
        cmd2.target = (-0.5f * cos - 0.866f * sin) * 1000.0f;

        //motor3
        robomas_interfaces::msg::MotorCommand cmd3;
        cmd3.motor_id = 3;
        cmd3.mode = 1;
        cmd3.target = cos * 1000.0f;
        msg.motors.push_back(cmd3);

        // Publish!
        pub_motor_->publish(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr pub_motor_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotNode>());
    rclcpp::shutdown();
    return 0;
}