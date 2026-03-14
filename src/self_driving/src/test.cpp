// route_before_shoot_(kind,back=True) -> (<aftershoot>? 1:0)
// kind_<status>
// || .route_<status>
//    .route_index_<status>
//ひとまず画像認識ノードの情報から次の姿勢を出すためのノード
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"
#include <cmath>
#include <cassert>
#include "topic.hpp"

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode()
    : Node("test_node"), target_pose(this, "target_pose")
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ManagerNode::publish_once, this)
        );
    }
private:
    void publish_once() {
        //目標値をpublishして挙動を確認(テスト用)
        target_pose.publish([&](auto& msg){
            msg.index = 0;
            msg.mode = 0;
            msg.x = 1.0;
            msg.y = 2.0;
            msg.yaw = 0.5;
            printf("%d,%lf,%lf,%lf",msg.index,msg.x,msg.y,msg.yaw);
            //RCLCPP_INFO(get_logger(), "Published target pose: index=%d, x=%.2f, y=%.2f, yaw=%.2f", 
            //        msg.index, msg.x, msg.y, msg.yaw);
        });
    }
    rclcpp::TimerBase::SharedPtr timer_;   // ← これが必要！
    Topic<self_driving::msg::Target> target_pose;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManagerNode>());
  rclcpp::shutdown();
  return 0;
}
