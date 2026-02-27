//"class ImuProcessorNode : public rclcpp::Node"くらいから読んでほしい。

#include <cmath>
#include <memory>
#include <mutex>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

using std::placeholders::_1;

// ============================
// 3x3 回転行列 -> クオータニオン (w,x,y,z)
// （Python版と同じロジック）
// ============================
static Eigen::Vector4d rotmat_to_quat(const Eigen::Matrix3d& Rin)
{
  Eigen::Matrix3d R = Rin;

  // SVDで直交化: R = U * V^T
  {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
  }

  double qw, qx, qy, qz;
  const double tr = R.trace();

  if (tr > 0.0) {
    const double S = std::sqrt(tr + 1.0) * 2.0;
    qw = 0.25 * S;
    qx = (R(2,1) - R(1,2)) / S;
    qy = (R(0,2) - R(2,0)) / S;
    qz = (R(1,0) - R(0,1)) / S;
  } else {
    if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
      const double S = std::sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2.0;
      qw = (R(2,1) - R(1,2)) / S;
      qx = 0.25 * S;
      qy = (R(0,1) + R(1,0)) / S;
      qz = (R(0,2) + R(2,0)) / S;
    } else if (R(1,1) > R(2,2)) {
      const double S = std::sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2.0;
      qw = (R(0,2) - R(2,0)) / S;
      qx = (R(0,1) + R(1,0)) / S;
      qy = 0.25 * S;
      qz = (R(1,2) + R(2,1)) / S;
    } else {
      const double S = std::sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2.0;
      qw = (R(1,0) - R(0,1)) / S;
      qx = (R(0,2) + R(2,0)) / S;
      qy = (R(1,2) + R(2,1)) / S;
      qz = 0.25 * S;
    }
  }

  Eigen::Vector4d q(qw, qx, qy, qz);
  const double n = q.norm();
  if (n <= 0.0) return Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  return q / n;
}

//正規直交
static Eigen::Matrix3d orthonormalize(const Eigen::Matrix3d& mat)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      mat, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  Eigen::Matrix3d R = U * V.transpose();

  // det=+1 を保証（reflection除去）
  if (R.determinant() < 0.0) {
    U.col(2) *= -1.0;          // 3列目を反転（最小修正）
    R = U * V.transpose();
  }

  return R;
}


// ============================
// Gram-Schmidt（Python版と同じ：列2を基準、列0/列1を直交化）
// ※ここは Python と同じ “列ベクトル基準” 実装なのでそのまま
// ============================
static Eigen::Matrix3d reorthonormalize_R_gs(const Eigen::Matrix3d& Rin)
{
  constexpr double eps = 1e-12;

  Eigen::Vector3d z = Rin.col(2);
  Eigen::Vector3d e3;
  const double z2 = z.squaredNorm();
  e3 = (z2 < eps) ? Eigen::Vector3d(0.0, 0.0, 1.0) : (z / std::sqrt(z2));

  Eigen::Vector3d x = Rin.col(0);
  x = x - (e3.dot(x)) * e3;
  double xn = x.norm();
  if (xn < eps) {
    Eigen::Vector3d tmp = (std::abs(e3.x()) < 0.9) ? Eigen::Vector3d(1,0,0) : Eigen::Vector3d(0,1,0);
    x = tmp - (e3.dot(tmp)) * e3;
    x.normalize();
  } else {
    x /= xn;
  }
  Eigen::Vector3d e1 = x;

  Eigen::Vector3d y = Rin.col(1);
  y = y - (e3.dot(y)) * e3 - (e1.dot(y)) * e1;
  double yn = y.norm();
  Eigen::Vector3d e2;
  if (yn < eps) {
    e2 = e3.cross(e1);
    e2.normalize();
  } else {
    e2 = y / yn;
  }

  Eigen::Matrix3d R;
  R.col(0) = e1;
  R.col(1) = e2;
  R.col(2) = e3;

  if (R.determinant() < 0.0) {
    R.col(1) = -R.col(1);
  }
  return R;
}

//ここから読んでください。
class ImuProcessorNode : public rclcpp::Node
{
public:
  ImuProcessorNode()
  : rclcpp::Node("imu_processor_node")
  {

    //とりあえず向き(クオータニオン)だけpublishするやつ。多分使わない
    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("quat", 10);

    //位置と向きをまとめてpublish.
    pub_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("robot_pose", 10);

    //imuからデータの受信
    // imu_array: [ax, ay, az, wx, wy, wz]
    // accel: m/s^2, gyro: rad/s で揃えている前提
    sub_imu_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "imu_array", 10, std::bind(&ImuProcessorNode::cb_imu, this, _1)
    );

    //lidarからデータの受信。7次元の配列
    //"lidar_to_imu"=[lidarの更新時間(周期(s))、位置(x,y,z(単位:m))、角度の変化]
    //"lidar_to_imu":[dt_lidar_,x_lidar_, y_lidar_, z_lidar_ ,dthx_lidar_, dthy_lidar_, dthz_lidar_]
    //ここで角度の変化について。lidarの更新前から更新後への角度の変化をロボット座標系の微小角としてほしい。
    //それが"dthx_lidar_, dthy_lidar_, dthz_lidar_"である。よく話し合いが必要。
    sub_lidar_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "lidar_to_imu", 10, std::bind(&ImuProcessorNode::cb_lidar, this, _1)
    
    );

    //初期設定 この値の説明は一番最後にある(登場人物で検索)。
    g0_       = 9.80665; // m/s^2
    R_.setIdentity();
    last_time_valid_ = false;

    x_ = 1;
    y_ = 3;
    z_ = 0.05;
    vx_= 0.0;
    vy_= 0.0;
    vz_= 0.0;
    qw_= 1.0;
    qx_= 0.0;
    qy_= 0.0;
    qz_= 0.0;
    dt_=0.005;

    //imuのてきとーな初期設定
    ax_ = 0.0;
    ay_ = 0.0;
    az_ = 9.8;
    wx_ = 0.0;
    wy_ = 0.0;
    wz_ = 0.0;

    // ZUPT用
    a_total_ = std::sqrt(ax_*ax_+ay_*ay_+az_*az_)-g0_;
    w_total_ = std::sqrt(wx_*wx_+wy_*wy_+wz_*wz_);
    a_threshold_ = 0.1;//こいつらてきとー
    w_threshold_ = 0.05;

    filtered_a_total_ = a_total_;
    filtered_w_total_ = w_total_;

   //計算の初期設定
    pre_ax_=0.0;
    pre_ay_=0.0;
    pre_az_=0.0;
    pre_vx_=0.0;
    pre_vy_=0.0;
    pre_vz_=0.0;

    // ---- チューニング（SI）----
    gain_     = 0.1;
    sigma2_   = 0.001;    //  (m/s^2)^2 てきとー
    

    rot_lidar_para_ = 0.1;
    lidar_pose_gain_ = 0.9;//lidarの自己位置の信頼度.1でいい気がする
    lidar_v_gain_ =0.1;
   
    // LiDAR pending
    lidar_pending_ = false;

    RCLCPP_INFO(this->get_logger(), "IMU Processor node started.");
  }

private:
  //imuのデータを取得したら"cb_imu"で自己位置を計算
  void cb_imu(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    //imuからのデータが不足したとき
    if (msg->data.size() < 6) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "imu_array length is too short: %zu (need >= 6)", msg->data.size());
      return;
    }

    //  dt_ : このノードの更新時間を測定。
    const rclcpp::Time now = this->get_clock()->now();
    
    if (!last_time_valid_) {//1回目の処理
      last_time_ = now;
      last_time_valid_ = true;
      return;
    }

    // const double dt = (now - last_time_).seconds();
    dt_ = (now - last_time_).seconds();
    last_time_ = now;
    if (dt_ <= 0.0) return;
    
    // ---- accel ----
    ax_ = static_cast<double>(msg->data[0]);
    ay_ = static_cast<double>(msg->data[1]);
    az_ = static_cast<double>(msg->data[2]);
    // ---- gyro [rad/s] ----
    wx_ = static_cast<double>(msg->data[3]);
    wy_ = static_cast<double>(msg->data[4]);
    wz_ = static_cast<double>(msg->data[5]);

    
    {

      std::lock_guard<std::mutex> lock(mtx_);//分かってない。
      //まずは回転行列"R_"を計算(IMUのみ)
      RotMatIMU();
      
      //位置を計算(IMUのみ,なんか怪しい。)
      PoseIMU();

      //ここでlidarの更新を入れる。
      // ---- apply LiDAR pending once, if available ----
      if (lidar_pending_) {
  
        //lidarで回転行列"R_"を修正。
        RotMatLIDAR();
        //位置を計算
        PoseLIDAR();

        lidar_pending_ = false;//lidarからsubscribeしましたのスイッチオフ
      }

      // publishする
      publish_locked_();
    }
  }

//lidarから受け取ったとき
void cb_lidar(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 7) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "lidar_to_imu length is too short: %zu (need >= 7)", msg->data.size());
      return;
    }

    std::lock_guard<std::mutex> lock(mtx_);//わからん

    //lidarからsubscribe
    dt_lidar_   = static_cast<double>(msg->data[0]);
    x_lidar_   = static_cast<double>(msg->data[1]);
    y_lidar_   = static_cast<double>(msg->data[2]);
    z_lidar_   = static_cast<double>(msg->data[3]);
    dthx_lidar_ = static_cast<double>(msg->data[4]);
    dthy_lidar_ = static_cast<double>(msg->data[5]);
    dthz_lidar_ = static_cast<double>(msg->data[6]);
    //subscribeしましたのスイッチオン
    lidar_pending_ = true;
  }
void publish_locked_()
  {
    const Eigen::Vector4d q = rotmat_to_quat(R_);//回転行列をクオータニオンに
    qw_ = q[0];
    qx_ = q[1];
    qy_ = q[2];
    qz_ = q[3];

    std_msgs::msg::Float32MultiArray out;//これはとりあえず向き(クオータニオン)だけpublishするやつ。多分使わない
    out.data.resize(4);
    out.data[0] = static_cast<float>(qw_);
    out.data[1] = static_cast<float>(qx_);
    out.data[2] = static_cast<float>(qy_);
    out.data[3] = static_cast<float>(qz_);
    pub_->publish(out);

    //位置とむき(クオータニオン)を同時に出力
    std_msgs::msg::Float32MultiArray out_pose;
    out_pose.data.resize(7);
    out_pose.data[0] = static_cast<float>(x_);
    out_pose.data[1] = static_cast<float>(y_);
    out_pose.data[2] = static_cast<float>(z_);
    out_pose.data[3] = static_cast<float>(qw_);
    out_pose.data[4] = static_cast<float>(qx_);
    out_pose.data[5] = static_cast<float>(qy_);
    out_pose.data[6] = static_cast<float>(qz_);

    pub_pose_->publish(out_pose);
  }

void RotMatIMU()
  {
    //imuによる微小回転行列K
    Eigen::Matrix3d K;
    K <<  0.0, -wz_,  wy_,
          wz_,  0.0, -wx_,
         -wy_,  wx_,  0.0;

    // dR ≈ I + dt_*K
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity() + dt_ * K;

    // 右掛け
    R_ = R_*dR;
    
    double a_total = std::sqrt(ax_*ax_ + ay_*ay_ + az_*az_);

    //ベクトル、すぐ下で計算
    Eigen::Vector3d g_dir(0.0, 0.0, 1.0);
    Eigen::Vector3d w_dir(0.0, 0.0, 1.0);

    // このifはわかってない。なんか多分違う
    if (a_total > 0.0 || R_.row(2).norm() > 0.0) {
      //g_dirとは加速度の値から求まるz軸の向きである。
      if (a_total > 0.0) {
        g_dir = Eigen::Vector3d(ax_, ay_, az_) / a_total;
      }
      const double rn = R_.row(2).norm();
      if (rn > 0.0) {
        //R_の3行目を規格化しそれをベクトル"w_dir"とする。これが"g_dir(imuの加速度の値)"と一致するはずである。
        const Eigen::RowVector3d r2 = R_.row(2);
        w_dir = Eigen::Vector3d(r2(0), r2(1), r2(2)) / rn;
      }
    }

    //この二つがどれだけずれているのかは外積で求まる。
    // cr = w_dir × g_dir（外積）。crの分のずれを修正する必要がある。
    const Eigen::Vector3d cr = w_dir.cross(g_dir);

    Eigen::Matrix3d K2;
    K2 <<   0.0,   -cr.z(),  cr.y(),
            cr.z(), 0.0,    -cr.x(),
           -cr.y(), cr.x(),  0.0;

    // SI（m/s^2）前提：g0=9.80665
    //工夫した相補フィルター。パラメータ(sigma2_)はてきとー。
    const double diff = (a_total - g0_);
    const double weight = std::exp(-(diff * diff) / (2.0 * sigma2_));
    K2 *= weight;

    Eigen::Matrix3d dR2 = Eigen::Matrix3d::Identity() + K2 * gain_;
    
    R_ = R_ * dR2.transpose();

    // 3段目を基準とする直交化
    R_ = reorthonormalize_R_gs(R_);
    //RotMatIMU()終わり。
  }

/////////////////
void RotMatLIDAR()
  {
    Eigen::Matrix3d K;
    K <<  0.0, -dthz_lidar_,  dthy_lidar_,
          dthz_lidar_,  0.0, -dthx_lidar_,
         -dthy_lidar_,  dthx_lidar_,  0.0;

    // dR ≈ I + rot_lidar_para_*K
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity() + rot_lidar_para_ * K;

    // 右掛け
    R_ = R_ * dR;
    R_ = orthonormalize(R_);
  }
void PoseIMU()
{ 
  //ここでIMUのみで位置を計算するパート

  Eigen::Vector3d a_vec(ax_, ay_, az_);
  Eigen::Vector3d g0_vec(0.0, 0.0, g0_);
    
  // a_vecとはworld座標における加速度である。
  a_vec = (R_*a_vec) - g0_vec;

  vx_ += 0.5*(pre_ax_+a_vec(0))*dt_;
  vy_ += 0.5*(pre_ay_+a_vec(1))*dt_;
  vz_ += 0.5*(pre_az_+a_vec(2))*dt_;
  
  // 静動判定をする(はじめ)
  a_total_ = a_vec.norm();
  w_total_ = std::sqrt(wx_*wx_+wy_*wy_+wz_*wz_);

  //ローパスフィルター
  
  double lpf_tau_ = 0.05;
  double alpha = dt_ / (lpf_tau_ + dt_);

  filtered_a_total_ += alpha * (a_total_ - filtered_a_total_);
  filtered_w_total_ += alpha * (w_total_ - filtered_w_total_);

  // これはメモ。//a_threshold_ = 0.05 ;w_threshold_ = 0.05
  bool a_stop = (filtered_a_total_ < a_threshold_);
  bool w_stop = (filtered_w_total_ < w_threshold_);

  if((a_stop ) && (w_stop)){
    // 止まってたら速度を95％にしてみる。
    vx_ *=0.95;
    vy_ *=0.95;
    vz_ *=0.95;

  }
 ////////////(静動判定終わり)
  pre_ax_ = a_vec(0);
  pre_ay_ = a_vec(1);
  pre_az_ = a_vec(2);

  x_ += 0.5*(pre_vx_+vx_)*dt_;
  y_ += 0.5*(pre_vy_+vy_)*dt_;
  z_ += 0.5*(pre_vz_+vz_)*dt_;

  pre_vx_ = vx_;
  pre_vy_ = vy_;
  pre_vz_ = vz_;

}
void PoseLIDAR()
{ 
  if(dt_lidar_ < 1e-12 ){return;} else {

    if (dt_lidar_ > 1.0)  dt_lidar_= 1.0; 

  //lidarによる速度の計算。これでいいのだろうか。
  
  vx_ = (1-lidar_v_gain_)*vx_ +lidar_v_gain_*(x_lidar_-x_lidar_old_)/dt_lidar_;
  vy_ = (1-lidar_v_gain_)*vy_ +lidar_v_gain_*(y_lidar_-y_lidar_old_)/dt_lidar_;
  vz_ = (1-lidar_v_gain_)*vz_ +lidar_v_gain_*(z_lidar_-z_lidar_old_)/dt_lidar_;
  

  pre_vx_ = vx_;
  pre_vy_ = vy_;
  pre_vz_ = vz_;
  //位置の計算


  double en = std::sqrt((x_lidar_-x_)*(x_lidar_-x_) + (y_lidar_-y_)*(y_lidar_-y_) + (z_lidar_-z_)*(z_lidar_-z_));

  if (en > 0.6){
    x_lidar_=x_;
    y_lidar_=y_;
    z_lidar_=z_;
  }; // 1m以上飛んだら捨てる、など（要調整）

  x_  += lidar_pose_gain_*(x_lidar_-x_);
  y_  += lidar_pose_gain_*(y_lidar_-y_);
  z_  += lidar_pose_gain_*(z_lidar_-z_);

  
  x_lidar_old_ = x_lidar_;
  y_lidar_old_ = y_lidar_;
  z_lidar_old_ = z_lidar_;
  
  }
}

  private:
    // ROS I/O
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_imu_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_lidar_;


    //登場人物
    // State
    Eigen::Matrix3d R_;
    rclcpp::Time last_time_;
    bool last_time_valid_;

    // Params (keep as-is)
    double gain_;
    double g0_;
    double sigma2_;
    double lidar_pose_gain_,lidar_v_gain_;
    double rot_lidar_para_;
    //imuのデータ
    double ax_,ay_,az_,wx_,wy_,wz_;

    //ZUPT
    double a_total_,w_total_;
    double a_threshold_, w_threshold_;
    double filtered_a_total_,filtered_w_total_;

    //計算で使う(world座標である)
    double pre_ax_,pre_ay_,pre_az_,pre_vx_,pre_vy_,pre_vz_;

    // LiDARのデータ
    bool lidar_pending_;
    double dt_lidar_;
    double x_lidar_, y_lidar_, z_lidar_;
    double x_lidar_old_,y_lidar_old_,z_lidar_old_;
    double dthx_lidar_, dthy_lidar_, dthz_lidar_;


    //位置と姿勢と速度
    double dt_;
    double x_, y_, z_;
    double qw_,qx_,qy_,qz_;
    double vx_, vy_, vz_;


    // Concurrency guard
    std::mutex mtx_;
  };

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuProcessorNode>());
  rclcpp::shutdown();
  return 0;
}