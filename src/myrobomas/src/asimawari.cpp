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
    void jidou() { // 自動モードであることを示す
        auto msg_jidou = robomas_interfaces::msg::CanFrame();
        msg_jidou.id = 0x100;         // 送信したいCAN ID
        msg_jidou.dlc = 8;            // データ長
        msg_jidou.data = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
        
        can_pub_->publish(msg_jidou);
    }

    void syudou() { // 手動モードであることを示す
        auto msg_syudou = robomas_interfaces::msg::CanFrame();
        msg_syudou.id = 0x100;         // 送信したいCAN ID
        msg_syudou.dlc = 8;            // データ長
        msg_syudou.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        
        can_pub_->publish(msg_syudou);
    }

    void fanfare() { // ファンファーレを示す
        auto msg_fanfare = robomas_interfaces::msg::CanFrame();
        msg_fanfare.id = 0x001;         // 送信したいCAN ID
        msg_fanfare.dlc = 8;            // データ長
        msg_fanfare.data = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
        
        can_pub_->publish(msg_fanfare);
    }

    void fanfare_no() { // ファンファーレを示す
        auto msg_fanfare_no = robomas_interfaces::msg::CanFrame();
        msg_fanfare_no.id = 0x001;         // 送信したいCAN ID
        msg_fanfare_no.dlc = 8;            // データ長
        msg_fanfare_no.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        
        can_pub_->publish(msg_fanfare_no);
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
        cmd1.target = (-0.5f * cos + 0.866f * sin + joyinfo->buttons[4] - joyinfo->buttons[5]) * 1000.0f;
        msg.motors.push_back(cmd1);

        //motor2(足回り)
        robomas_interfaces::msg::MotorCommand cmd2;
        cmd2.motor_id = 2;
        cmd2.mode = 1;
        cmd2.target = (-0.5f * cos - 0.866f * sin + joyinfo->buttons[4] - joyinfo->buttons[5]) * 1000.0f;
        msg.motors.push_back(cmd2);

        //motor3(足回り)
        robomas_interfaces::msg::MotorCommand cmd3;
        cmd3.motor_id = 3;
        cmd3.mode = 1;
        cmd3.target = (cos + joyinfo->buttons[4] - joyinfo->buttons[5]) * 1000.0f;
        msg.motors.push_back(cmd3);

        //motor4(射出)
        robomas_interfaces::msg::MotorCommand cmd4;
        cmd4.motor_id = 4;
        cmd4.mode = 1;
        cmd4.target = -3500.0f * (/*joyinfo->buttons[3]*/ - joyinfo->buttons[1]); // Yボタンで上昇、Aボタンで何も起こらない
        //Aボタンじゃないと壊れる右buttons[1]=1のとき→cmd.target > 0
        msg.motors.push_back(cmd4);

        //motor5(昇降)
        robomas_interfaces::msg::MotorCommand cmd5;
        cmd5.motor_id = 5;
        cmd5.mode = 1;
        cmd5.target = -1000.0f * joyinfo->axes[3]; // 右スティックで上昇、下降
        msg.motors.push_back(cmd5);

        //motor6(把持)
        robomas_interfaces::msg::MotorCommand cmd6;
        cmd6.motor_id = 6;
        cmd6.mode = 1;
        cmd6.target = 1000.0f * (joyinfo->buttons[0] - joyinfo->buttons[2]);// Xボタンで、Aボタンで
        msg.motors.push_back(cmd6);

        // Publish!
        pub_motor_->publish(msg);
        
        // if (joyinfo->buttons[3]) {// Yボタンが押されたら
        //     jidou();// 自動制御
        // }
        //  else { // Yボタンを離したら
        //      syudou(); // 手動制御
        //  }

        //  if (joyinfo->buttons[2]){ // Bボタンが押されたら
        //      fanfare(); // ファンファーレ達成
        //  }

        //  else {
        //     fanfare_no();
        //  }

        RCLCPP_INFO(this->get_logger(), "joy_callback");
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