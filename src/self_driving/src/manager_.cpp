#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <array>
#include <map>
#include <functional>
#include <pair>
#include <chrono>
#include <cmath>

#include "self_driving/msg/target_status.hpp"
#include "self_driving/msg/target.hpp"

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode()
    : Node("manager_node")
    {
        route_index_ = 0;
        waypoint_index_ = 0;
        promise_index_ = 0;

        sub_status_ = create_subscription<self_driving::msg::TargetStatus>(
            "pursuit/status", 10,
            std::bind(&ManagerNode::update_status, this, std::placeholders::_1)
        );

        pub_pose_ = this->create_publisher<self_driving::msg::Target>(
            "target_pose", 10
        );

        sub_pose_icp_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "robot_pose_icp", 10,
            std::bind(&ManagerNode::pose_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ManagerNode::run_promise_chain, this)
        );

        // 例: 最初に init ルートを展開しておく
        unfold_route("init", false);
    }

private:
    // ==========================
    // ROS I/F
    // ==========================
    rclcpp::Subscription<self_driving::msg::TargetStatus>::SharedPtr sub_status_;
    rclcpp::Publisher<self_driving::msg::Target>::SharedPtr           pub_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_icp_;
    rclcpp::TimerBase::SharedPtr                                      timer_;


    std::vector<std::pair<std::function<int, bool()>> cond_link;
    uint8 inner_order = 0;
    std::vector
    //オートマトンノード一覧 (条件分岐の起点となりうる)

    //0:スタート
    //1:ノーツ入口
    //2:赤把持
    //3:青把持
    //4:黄把持
    //5:赤射出
    //6:青射出
    //7:黄射出

    //リンク
    //1の持つcond_linkがtrueになった方に遷移
    //for (link : links){
    //  link.cond_f(field)
    //}
    //全部デバッグプリントする。

    double note_front_zone[3][2] = 
    {
        {1.362, 3.5},//blue
        {1.8,   1.85},//yellow
        {0.924, 5.112}//red
    }, 
    
    void automaton_parser(){
        switch(phase) {
        case 0:
            printf("start")
            if (pursuit({0.35,  5.888, M_PI/2}))
                phase = 1;
        case 1:
            printf("catch_ball")
            switch (this.pickcolor)
            if (inner_order==0 && pursuit({chosen_ball[0].x, 5.888, M_PI/2})) {inner_order++};
            if (inner_order==1 && pursuit({chosen_ball[1].x, 5.888, M_PI/2})) {inner_order=0; phase = 2;}
        case 2:
            printf("return_and_shoot")
            if (inner_order==0 && pursuit({0.35,
                note_front_zone[choosen_ball.color_id][1],
                M_PI})) inner_order++;
            if (inner_order==1 && pursuit({note_front_zone[choosen_ball.color_id][0], 
                note_front_zone[choosen_ball.color_id][1],
                M_PI})) {inner_order++};
            if (inner_order==2){
                if (color_cache.size()>=3) {puts("fanfare");}
                else {inner_order=0;phase=0;}
            }
    }
}
}