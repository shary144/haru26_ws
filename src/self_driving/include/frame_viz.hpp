#include "my_tf2.hpp"
#include "BallChache.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include <numbers>

//CRTPとして実装
template<class Derived>
class FrameViz: public Derived{
public:
    FrameViz(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Derived(options)
    {
        // FrameViz 独自の初期化
        unique_id = 0;
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frame_viz", 10);
    }
    void viz_pos(my_tf2::Frame base_frame,my_tf2::Pose pos){
        my_tf2::Pose base_pos=base_frame(pos.frame(0,0,0));
        tf2::Quaternion q;
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        visualization_msgs::msg::Marker arrowx;
        arrowx.ns = pos.frame->name+"_x"; //
        arrowx.pose.position.x = base_pos.x;
        arrowx.pose.position.y = base_pos.y;
        q.setRPY(0, 0, base_pos.th);
        arrowx.scale.x = pos.x;
        arrowx.color.r = 1.0;
        arrowx.color.g = 0.0;
        arrowx.color.b = 0.0;
        arrowx.color.a = 0.5;
        inject_template2arrow(arrowx,q);
        marker_array.push_back(arrowx);

        visualization_msgs::msg::Marker arrowy;
        arrowy.ns = pos.frame->name+"_y"; //
        arrowy.pose.position.x = base_pos.x;
        arrowy.pose.position.y = base_pos.y;
        q.setRPY(0, 0, base_pos.th+std::numbers::pi/2); //
        arrowy.scale.x = pos.y; //
        arrowy.color.r = 0.0; //
        arrowy.color.g = 0.0; //
        arrowy.color.b = 1.0; //
        arrowy.color.a = 0.5; //
        inject_template2arrow(arrowy,q);
        marker_array.push_back(arrowy)

    }
    void inject_template2arrow(visualization_msgs::msg::Marker& arrow, const tf2::Quaternion& q){
        arrow.header.stamp = this->now();
        arrow.id = this.unique_id++;
        arrow.header.frame_id = "map";
        arrow.pose.orientation.x = q.x();
        arrow.pose.orientation.y = q.y();
        arrow.pose.orientation.z = q.z();
        arrow.pose.orientation.w = q.w();
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.y = arrow.scale.z = 0.01;
        arrow.pose.position.z = 0.1;
        arrow.lifetime = rclcpp::Duration(0.025);
    }
    void viz_balls(std::vector<ballcache::Ball> ball_array){
        auto marker_array = visualization_msgs::msg::MarkerArray()
        for (auto& ball: ball_array){
            visualization_msgs::msg::Marker arrowx
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "ball";
            marker.id = this->unique_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = ball.x;
            marker.pose.position.y = ball.y;
            marker.pose.position.z = 0.3;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
            //0: 赤 1:黄 2:青
            if (ball.color_id == 0){
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else if (ball.color_id==1){
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else{
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            if (ball.onstage){
                makrer.color.a = 1.0
            }else{
                marker.color.a = 0.3;
            }
            marker.lifetime = rclcpp::Duration(0.034);
            marker_array.push_back(marker);
        }
    }
private:
    visualization_msgs::msg::MarkerArray marker_pub_;
    int unique_id;
};