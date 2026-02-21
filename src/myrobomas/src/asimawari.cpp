#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <sensor_msgs/msg/joy.hpp>
// カスタムメッセージのインポート
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"

using namespace std::chrono_literals;

class MyRobotNode : public rclcpp::Node {
public:
    MyRobotNode() : Node("my_robot_node") {
        // Publisherの作成
        sub_joy_   = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&MyRobotNode::joy_callback, this, std::placeholders::_1));        
        pub_motor_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        can_pub_   = this->create_publisher<robomas_interfaces::msg::CanFrame>("/robomas/can_tx", 10);
    }

private:
    void send_can_on() { // 電磁弁に電流を送る
        auto msg_on = robomas_interfaces::msg::CanFrame();
        msg_on.id = 0x111;         // 送信したいCAN ID
        msg_on.dlc = 8;            // データ長
        msg_on.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 電流を送りましょう
        
        can_pub_->publish(msg_on);
    }

    void send_can_off() { // 電磁弁に流れる電流を止める
        auto msg_off = robomas_interfaces::msg::CanFrame();
        msg_off.id = 0x111;         // 送信したいCAN ID
        msg_off.dlc = 8;            // データ長
        msg_off.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 電流を止めましょう
        
        can_pub_->publish(msg_off);
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joyinfo){
        auto msg = robomas_interfaces::msg::RobomasPacket();

        float cos = joyinfo->axes[0]; //左スティックX
        float sin = -(joyinfo->axes[1]); //左スティックY

        //とりますべて速度制御(mode = 1)

        //motor1(足回り)
        robomas_interfaces::msg::MotorCommand cmd1;
        cmd1.motor_id = 1;
        cmd1.mode = 1;
        cmd1.target = (-0.5f * cos + 0.866f * sin) * 1000.0f;
        msg.motors.push_back(cmd1);

        //motor2(足回り)
        robomas_interfaces::msg::MotorCommand cmd2;
        cmd2.motor_id = 2;
        cmd2.mode = 1;
        cmd2.target = (-0.5f * cos - 0.866f * sin) * 1000.0f;

        //motor3(足回り)
        robomas_interfaces::msg::MotorCommand cmd3;
        cmd3.motor_id = 3;
        cmd3.mode = 1;
        cmd3.target = cos * 1000.0f;
        msg.motors.push_back(cmd3);

/*
        //motor4(昇降)
        robomas_interfaces::msg::MotorCommand cmd4;
        cmd4.motor_id = 4;
        cmd4.mode = ;
        cmd4.target = ;
        msg.motors.push_back(cmd4);
*/

        // Publish!
        pub_motor_->publish(msg);

        
        if (joyinfo->axes[0]) {// Aボタンが押されたら
            send_can_on(); // 電磁弁に電流が送られる
        }
        else { // Aボタンを離したら
            send_can_off(); // 電流が止まる
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr pub_motor_;
    rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr can_pub_;    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotNode>());
    rclcpp::shutdown();
    return 0;
}