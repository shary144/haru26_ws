#include <rclcpp/rclcpp.hpp>

class angleControl {
public:
    angleControl();

    int phase = 0;
    uint8_t rising_a = 10000;
    int safe_dist = 100;
    double init_angle;
    double v; //一応状態変数
    double dt = 0.025 //制御間隔時間
    std::optional<double> cur_angle;
    
    // feedback_sub_へのコールバック
    void motor( robomas_interfaces::msg::RobomasFrame feedback_msg, robomas_interfaces::msg::MotorCommand& cmd, 
        double ta_angle, double ta_v, bool neg_frag) 
    {
        robomas_interfaces::msg::MotorCommand cmd;
        if (!cur_angle) {
            init_angle = feedback_msg.angle;
            cur_angle = 0;
        }
        else cur_angle = feedback_msg.angle-init_angle;

        if (phase == 0){
            cmd.mode = 1
            if (v*v)/(2*rising_a) <= (ta_angle - cur_angle - safe_dist) {
                v -= dt*rising_a;
            } else {
                if (v < ta_v) v += dt*rising_a;
            }
            if ((v<0)||(cur_angle <= ta_angle-safe_dist)) {
                phase++;
                cmd.target = 0;
            }
            cmd.target = (negfrag ? -1:1) * v;
        } else if (phase==1) {
            //位置制御
            cmd.mode = 2;
            cmd.target = (negfrag ? -1:1) * ta_angle + init_angle;
        }
    }

}