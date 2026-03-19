#include <rclcpp/rclcpp.hpp>
#include <vector>
// カスタムメッセージのインポート
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"
#include "robomas_interfaces/msg/robomas_frame.hpp"

using namespace std::chrono_literals;

class MyPoint10Node : public rclcpp::Node {
public:
    MyPoint10Node() : Node("my_point10_node") {
        // Publisherの作成    
        pub_motor_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        can_pub_   = this->create_publisher<robomas_interfaces::msg::CanFrame>("/robomas/can_tx", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&MyPoint10Node::timer_callback, this));
    }

private:
    void timer_callback() {
        auto msg_can = robomas_interfaces::msg::CanFrame();
        msg_can.id = 0x100;         // 送信したいCAN ID
        msg_can.dlc = 8;            // データ長
        msg_can.data = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; // データ本体 （uint型であることに注意）
        
        can_pub_->publish(msg_can);

        // 前進し続けます。
        float cos = 0;
        float sin = 1;

        auto msg = robomas_interfaces::msg::RobomasPacket();

        //motor1(足回り)
        robomas_interfaces::msg::MotorCommand cmd1;
        cmd1.motor_id = 1;
        cmd1.mode = 1;
        cmd1.target = (0.5f * cos - 0.866f * sin) * 1000.0f;
        msg.motors.push_back(cmd1);

        //motor2(足回り)
        robomas_interfaces::msg::MotorCommand cmd2;
        cmd2.motor_id = 2;
        cmd2.mode = 1;
        cmd2.target = (0.5f * cos + 0.866f * sin) * 1000.0f;
        msg.motors.push_back(cmd2);

        //motor3(足回り)
        robomas_interfaces::msg::MotorCommand cmd3;
        cmd3.motor_id = 3;
        cmd3.mode = 1;
        cmd3.target = (-cos) * 1000.0f;
        msg.motors.push_back(cmd3);

        // Publish!
        pub_motor_->publish(msg);   
    }     

    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr pub_motor_;
    rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr can_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPoint10Node>());
    rclcpp::shutdown();
    return 0;
}