/*
ここは自己位置推定が成功したときに使う
やること：
1. /robot_pose_icpの位置と角度のデータからできるだけ目標点に向かうように操縦
(ここで目標点の取り方は壁に沿う
+交差点で直角に曲がってノーツの手前で止まる
→射出後逆にたどってノーツゾーンに戻る)
2. 到達する予定の点を中心にeの半径の円の中に入ったら止まる
1.2.を繰り返す

起こりうる問題
1. センサーと指令の応答遅れによる振動→まあよほどのことがない限り気にしなくてもよさげ
2. 
*/
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <vector>
#include <cmath>
//ひとまずマップの左隅を基準に座標をとる(単位は[m])
//ボールのとこに入る境界がy = 3.5+0.35+1.612+0.5 = 約7.0 {0.35,7.0,M_PI/2}

double e = 0.3

//構造
//(kind{"red","green","blue"})
//v
//boxfront[kind] : 目標点の内ノーツ手前のもの
//v
//ノーツゾーンに向かう
//route.fetching_from(kind) = <box_fornt[kind]>
//箱手前に向かう
//route.pushing(kind) = <box_front[kind]>
//この時点でボールを拾ってきてからのボールをノーツに入れて次を取りに行くまで

//順序例
//(route_fetching_fromとして単にメンバ関数としての実装でいいかも)
//route.fetching_from("init")
//カメラに委託
//route.pushing("red")
//射出
//route.fetching_from("red")
//カメラに委託
//route.pushing("blue")
//射出
//route.fetching_from("blue")
//カメラに委託
//route.pushing("yellow")
//射出
//あとはメンバ関数navigate(target_pose:route)->status:intを

class NavNode : public rclcpp::Node
{
public:
  NavNode()
  : Node("nav_node") {
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/robot_pose_icp", 10,
      std::bind(&NavNode::callback,
                this, std::placeholders::_1));
  }
  void callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg
    ){
        if (msg->data.size() < 3) return;
        double x   = msg->data[0];
        double y   = msg->data[1];
        double yaw = msg->data[2];

        target_note = "red";
        int i=0;
        navigate(x,y,yaw,route_before_shoot_("red"));
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray sub_>::Shared_Ptr sub_;
  std::string target_note;
  

  std::vector<std::array<double,2>> route_before_shoot_(std::string kind){
    std::vector<std::array<double,2>> route_;
    switch (kind) {
        case "blue": 
        route_ = {{0.35,5.888},
                  {0.35,3.5},
                  {1.362,3.5}}; //射出の手前の時点までに姿勢を保っておかねばならない
        case "yellow":
        route_ = {{0.35,5.888},
                  {0.35,1.85},
                  {1.8 ,1.85}};
        case "red":
        route_ = {{0.35,5.888},
                  {0.35 ,5.112},
                  {0.924,5.112}};
        default:
        route_ = {{0.35,5.888}};
    }
    return route;
  }

  int navigate(std::string kind,std::array<double,3> target_pose){
    //target_poseは(x,y,theta)の配列
    //現在地とtarget_poseを比較して、目標に向かって動くための指令を出す
    //目標に近づいたら0を返す
    //まだ遠いなら1を返す
    dyaw = target_pose[1] - yaw;
    target_pose
  }
};
