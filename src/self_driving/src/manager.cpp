// route_before_shoot_(kind,back=True) -> (<aftershoot>? 1:0)
// kind_<status>
// || .route_<status>
//    .route_index_<status>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

#define INCLUDE_MEMBER_PARAM(type, name) \
  type name; \
  this->declare_parameter<type>(#name); \
  this->get_parameter(#name, name);


class NavigaterNode : public rclcpp::Node
{
public:
    NavigateNode()
    : Node("navigater_node")
    {
        sub_res_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "pursuit/status", 10,
            std::bind(&NavigaterNode::callback, this, std::placeholders::_1)
        );
        pub_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "pursuit/target", 10
        );
    }
private:
    //ここはパラメータの宣言と取得。yamlの数値ををここで同名のメンバ変数として定義している。
    INCLUDE_MEMBER_PARAM(std::vector<double>, blue_nav_point)
    INCLUDE_MEMBER_PARAM(std::vector<double>, yellow_nav_point)
    INCLUDE_MEMBER_PARAM(std::vector<double>, red_nav_point)
    INCLUDE_MEMBER_PARAM(std::vector<double>, notezone_entrance)
    INCLUDE_MEMBER_PARAM(double, wallDistance)

    color_queue_ = std::queue<std::string>();
    void set_route(const std::string &kind);
    void publish_target(double x, double y, double yaw)
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.push_back(x);
        msg.data.push_back(y);
        msg.data.push_back(yaw);
        pub_pose_->publish(msg);
    }

    // ==========================
    // pursuit/status コールバック
    // ==========================
    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
        msg.index
        if (msg.status) {
            
        } else
    }

        // ==========================
    // ルート設定
    // ==========================
    void set_route(const std::string &kind)
    {
        route_ = routes_[kind];
        route_index_ = 0;
        RCLCPP_INFO(get_logger(), "Route set: %s", kind.c_str());
    }
}