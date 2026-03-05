#include <rclcpp/rclcpp.hpp>
#include "ball_detector/msg/ball.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"
#include <chrono>
#include <string>
#include <functional>

class JidouNotes : public rclcpp::Node {
public:
    JidouNotes() : Node("jidou_notes") {
        // Publisher、subscriberの作成
        sub_ball_ = this->create_subscription<ball_detector::msg::Ball>("/ball_position", 10, std::bind(&JidouNotes::ball_callback, this, std::placeholders::_1));
        pub_motor_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&JidouNotes::timer_callback, this));
        can_pub_  = this->create_publisher<robomas_interfaces::msg::CanFrame>("/robomas/can_tx", 10);
        sub_robot_ = this->create_subscription<robomas_interfaces/msg/RobomasFrame>("/robomas/feedback", 10, std::bind(&JidouNotes::robomas_callback, this, std::placeholders::_1));
    }

private:
    std::string color; // ボールの色を保持する

// オムニ3輪用
    void asimawari(int cos, int sin, int mode, float target) { //足回りを動かす関数
        //motor1(足回り)
        auto msg =robomas_interfaces::msg::RobomasPacket();

        robomas_interfaces::msg::MotorCommand cmd1;
        cmd1.motor_id = 1;
        cmd1.mode = mode;
        cmd1.target = (-0.5f * cos + 0.866f * sin) * target;
        msg.motors.push_back(cmd1);

        //motor2(足回り)
        robomas_interfaces::msg::MotorCommand cmd2;
        cmd2.motor_id = 2;
        cmd2.mode = mode;
        cmd2.target = (-0.5f * cos - 0.866f * sin) * target;
        msg.motors.push_back(cmd2);

        //motor3(足回り)
        robomas_interfaces::msg::MotorCommand cmd3;
        cmd3.motor_id = 3;
        cmd3.mode = mode;
        cmd3.target = cos * target;
        msg.motors.push_back(cmd3);

        pub_motor_->publish(msg);
    }

//オムニ4輪用
/* 
    void asimawari(int cos, int sin, int mode, float target) { //足回りを動かす関数(オムニ4輪用)
        auto msg = robomas_interfaces::msg::RobomasPacket();

        robomas_interfaces::msg::MotorCommand cmd1;
        cmd.motor_id = 2;
        cmd3.mode = mode;
        cmd2.target = (-0.707 * cos + 0.707 * sin) * target;
        msg.motors.push_back(cmd3);

        //motor2
        robomas_interfaces::msg::MotorCommand cmd2;
        cmd2.motor_id = 2;
        cmd2.mode = mode;
        cmd2.target = (-0.707 * cos + 0.707 + sin) * target;
        msg.motors.push_back(cmd2);

        //motor3
        robomas_interfaces::msg::MotorCommand cmd3;
        cmd3.motor_id = 3;
        cmd3.mode = mode;
        cmd3.target = (0.707 * cos - 0.707 + sin) * target;
        msg.motors.push_back(cmd3);

        //motor4
        robomas_interfaces::msg::MotorCommand cmd3;
        cmd3.motor_id = 4;
        cmd3.mode = mode;
        cmd3.target = (0.707 * cos + 0.707 + sin) * target;
        msg.motors.push_back(cmd3);

        pub_motor_->publish(msg)
    }    
*/

    void shoukou(int i) { // 昇降(i = 1の時、上昇、i = -1の時、下降)
        auto msg =robomas_interfaces::msg::RobomasPacket();
        robomas_interfaces::msg::MotorCommand cmd5;
        cmd5.motor_id = 5;
        cmd5.mode = 2; // 位置制御
        cmd5.target = 720 * i;
        msg.motors.push_back(cmd5);
        pub_motor_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(3)); // とりあえず3秒待ってみる
    }

    void send_can_on() { // 電磁弁に電流を送る関数
        auto msg_on = robomas_interfaces::msg::CanFrame();
        msg_on.id = 0x100;         // 送信したいCAN ID (256にしています)
        msg_on.dlc = 8;            // データ長
        msg_on.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 電流を送りましょう
        
        can_pub_->publish(msg_on);
    }

    void send_can_off() { // 電磁弁に流れる電流を止める関数
        auto msg_off = robomas_interfaces::msg::CanFrame();
        msg_off.id = 0x100;         // 送信したいCAN ID (256にしています)
        msg_off.dlc = 8;            // データ長
        msg_off.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 電流を止めましょう
        
        can_pub_->publish(msg_off);
    }

    int step = 0; 
    // 0...機体から最も近いボールを見つける
    // 1...ボールのx座標をそろえる
    // 2...ボールと機体のy方向の距離を調整する
    // 3...昇降を上げる、エアシリを広げる、昇降を下げる、エアシリを戻す
    // 4...昇降を上げる、昇降を下げる(ボールを中に入れる動作)
    // 5...元の軌道に戻るよう、機体のy方向の位置を調整する        
    // 6...入れたボールの色と同じボールを探して、ボールのx座標をそろえる
    //  ...左に戻る（LiDARに任せようかな。左の壁から少し余裕あるくらいでお願いしたい。）

    void timer_callback(){ // ボールの情報を必要としないときに使う関数
        if (step == 0) {            
            // 初めに、右に動いてもらう。
            asimawari(1, 0, 1, 500.0f); //　速度制御、右に少しずづ進む
        }

        if (step == 3 || step == 9) { //昇降を上げる、エアシリを広げる、昇降を下げる、エアシリを戻す
            // 昇降を上げる(とりあえず720度回転させる)
            step *= -1; //timer_callback関数が呼び出されても何も起きないようにする。
            shoukou(1); // 昇降を上げる

            send_can_on(); // 把持を開く

            shoukou(-1); // 昇降を下げる

            send_can_off(); // 把持を閉じる
            
            step *= -1;
            step++; // ステップを進める
        }

        if (step == 4) { // 昇降を上げる、昇降を下げる
            step *= -1;
            shoukou(1); // 昇降を上げる

            shoukou(-1); // 昇降を下げる

            step *= -1;
            step++;         
        }     

        if (step == 5) { // 足回りを後ろにずらす
            step *= -1;
            asimawari(0, -1, 2, 360); // 位置制御、後ろに下がる
            while() // 目標値に近づくまでwhile
            step *= -1;
            step++;
        }

        if (step == 6) {
            asimawari(1, 0, 1, 500.0f); // 右に少しづつ動いてもらう
        }
/*
        if (step == 10) { // LiDARで移動
           asimawari(<目標値との差をLiDARで取得>, 0, 1, 500.0f);
           if (abs(目標値との差) < 0.005) {
               break; // このノードの役目はいったん終了
           }
        }
*/
    }

            
    void ball_callback(const ball_detector::msg::Ball::SharedPtr ball){

        double dx = ball->position.x;
        double dy = ball->position.y;

        if (step == 0) {
            step++; // ボールを見つけたら、ステップを進める
            color = ball->color; // 初めに見つけたボールの色を保存
        }

        if (step == 1 || (step == 7 && ball->color == color)) { // ボールのx座標をそろえる
            //motor3(足回り)
            asimawari(dx, 0, 1, 1000.0f); // 速度制御、偏差を速度として送る

            if (abs(dx) < 0.005) { // 5cm以内になったら、
                asimawari(0, 0, 0, 0); // 初期化
                step++; //次のステップに続く
            }
        }

        if (step == 2 || (step == 8 && ball->color == color)) { //ボールと機体のy方向の距離を調整する
            asimawari(0, dy, 1, 1000.0f);

            if (abs(dy) < 0.005) { // 5cm以内になったら
                asimawari(0, 0, 0, 0); // 初期化
                step++; // 次のステップへ
            }
        }

        if (step == 6 && ball->color == color) { // 探している色のボールが見つかったら、
            step++; // 次のステップへ
        }
            
    }

    rclcpp::Subscription<ball_detector::msg::Ball>::SharedPtr sub_ball_;
    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr pub_motor_;
    rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr can_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JidouNotes>());
    rclcpp::shutdown();
    return 0;
}
