// icp_localization_node.cpp
//
// 変更方針（前回までをなるべく維持）
// - robot_pose = [x,y,z,qw,qx,qy,qz] を購読（200Hz）して、最新の (x,y,th) を保持
// - LiDAR(40Hz) が来たとき、ICP の初期値として最新の (x,y,th) を使う
// - ICP の出力は「増分 [dx,dy,dth]（world）」として publish_to_IMU() で lidar_to_imu に流す
//と思ったが結局[x,y,z]を返す

// - これまでの x_,y_,th_ は「ICP後の推定姿勢」として維持（robot_pose が来ていればそれを初期値に置き換える）

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "my_lidar.hpp"  // readCSV, Lines2, modpi など

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::JacobiSVD;

// =====================================================
// グローバル変数: icp 関数が参照するマップ線分群 LINES2
//   形式: 2*J x 3 の行列 ( [x1 y1 1; x2 y2 1; ...] )
// =====================================================
MatrixXd LINES2;

// =====================================================
// yaw（theta）計算: quaternion(w,x,y,z) -> yaw
// robot_pose = [x,y,z,qw,qx,qy,qz] の前提
// =====================================================
static double yaw_from_quat(double qw, double qx, double qy, double qz)
{
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

// =====================================================
// icp 関数本体
//   入力: 現在の姿勢 (x, y, th), LiDAR 点群 (ロボット座標系, 2xN)
//   出力: 推定された新しい姿勢までの変化（VectorXd(3)）
//         [dx,dy,dth] を出力（world座標系）
// =====================================================
VectorXd icp(double x, double y, double th, MatrixXd LiDAR)
{

  // 裏の線分を除外
  std::vector<int> keep_line_indices;
  
  // --- 1. 残す線分の判定 ---
  for (int i = 0; i < LINES2.rows() / 2; ++i) {
    double x_start = LINES2(2*i, 0);
    double y_start = LINES2(2*i, 1);
    double x_fin = LINES2(2*i+1, 0);
    double y_fin = LINES2(2*i+1, 1);
    
    double cross =(x - x_start) * (y_fin - y_start)
                 -(y - y_start) * (x_fin - x_start);
    
    if (cross >= 0.0) {
    keep_line_indices.push_back(i);
    }
  }
  
  // --- 2. フィルタ後の行列を作成 ---
  MatrixXd LINES2_filtered(keep_line_indices.size() * 2, LINES2.cols());
  
  for (std::size_t k = 0; k < keep_line_indices.size(); ++k) {
    int i = keep_line_indices[k];
    LINES2_filtered.row(2*k) = LINES2.row(2*i);
    LINES2_filtered.row(2*k + 1) = LINES2.row(2*i + 1);
  }



  // 地図をロボット座標系に写像する変換行列
  Matrix3d transmatrix_field;
  transmatrix_field <<
      std::cos(th),  std::sin(th),  -x * std::cos(th) - y * std::sin(th),
     -std::sin(th),  std::cos(th),   x * std::sin(th) - y * std::cos(th),
      0,             0,              1;

  // LINES2_filtered を変換
  MatrixXd lines2t(LINES2_filtered.rows(), 3);
  lines2t = LINES2_filtered * (transmatrix_field.transpose());

  // LiDAR 点群を行ベクトル列 A (N×2) に
  MatrixXd A = LiDAR.transpose();
  int Arows = A.rows();
  int Acols = A.cols(); // ふつう 2

  // -------------------------------
  // 1. 初回フィルタリング：
  //    最も近い線分までの「二乗距離」が 大きい点を除外
  // -------------------------------
  //この値が重要
  ///////////////////////////

  const double max_dist  = 0.03;
  const double max_dist2 = max_dist * max_dist;
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////
// /////////////////////////////////////////

  std::vector<int> keep_indices;
  keep_indices.reserve(Arows);

  for (int j = 0; j < Arows; ++j) {
    double x0 = A(j, 0);
    double y0 = A(j, 1);

    double Short = std::numeric_limits<double>::infinity();  // min 二乗距離

    for (int i = 0; i < LINES2_filtered.rows() / 2; ++i) {
      double x1 = lines2t(2 * i,     0);
      double y1 = lines2t(2 * i,     1);
      double x2 = lines2t(2 * i + 1, 0);
      double y2 = lines2t(2 * i + 1, 1);

      double xi, yi;

      double vx1 = x2 - x1;
      double vy1 = y2 - y1;

      double vx0 = x0 - x1;
      double vy0 = y0 - y1;

      // 最近点を線分上のどこに取るか
      if ((vx0 * vx1 + vy0 * vy1) < 0.0) {
        xi = x1;
        yi = y1;
      } else if ((x0 - x2) * vx1 + (y0 - y2) * vy1 > 0.0) {
        xi = x2;
        yi = y2;
      } else {
        double denom = vx1 * vx1 + vy1 * vy1;
        if (denom <= 1e-12) {
          xi = x1;
          yi = y1;
        } else {
          double t = (vx0 * vx1 + vy0 * vy1) / denom;
          xi = x1 + t * vx1;
          yi = y1 + t * vy1;
        }
      }

      double dx = x0 - xi;
      double dy = y0 - yi;
      double di2 = dx * dx + dy * dy;  // 二乗距離

      if (di2 < Short) {
        Short = di2;
      }
    }

    if (Short <= max_dist2) {
      keep_indices.push_back(j);
    }
  }

  // 1点も残らなければ、増分ゼロ
  if (keep_indices.empty()) {
    VectorXd calc(3);
    calc << 0.0, 0.0, 0.0;
    return calc;
  }

  // フィルタ済み A を作成
  MatrixXd A_filtered(static_cast<int>(keep_indices.size()), Acols);
  for (std::size_t k = 0; k < keep_indices.size(); ++k) {
    A_filtered.row(static_cast<int>(k)) = A.row(keep_indices[k]);
  }
  A = A_filtered;
  Arows = A.rows();
  Acols = A.cols();

  // -------------------------------
  // 2. ICP ループ（フィルタ済み点群 A で実行）
  // -------------------------------
  MatrixXd totalTrans = MatrixXd::Identity(3, 3);
  MatrixXd totalTrans0;

  for (int k = 0; k < 20; ++k) {
    MatrixXd B(Arows, Acols);

    // B: A 各点の最近傍点
    for (int j = 0; j < Arows; ++j) {
      double x0 = A(j, 0);
      double y0 = A(j, 1);

      double Xnear = 0.0;
      double Ynear = 0.0;
      double Short = std::numeric_limits<double>::infinity();

      for (int i = 0; i < LINES2_filtered.rows() / 2; ++i) {
        double x1 = lines2t(2 * i,     0);
        double y1 = lines2t(2 * i,     1);
        double x2 = lines2t(2 * i + 1, 0);
        double y2 = lines2t(2 * i + 1, 1);

        double xi, yi;

        double vx1 = x2 - x1;
        double vy1 = y2 - y1;

        double vx0 = x0 - x1;
        double vy0 = y0 - y1;

        if ((vx0 * vx1 + vy0 * vy1) < 0.0) {
          xi = x1;
          yi = y1;
        } else if ((x0 - x2) * vx1 + (y0 - y2) * vy1 > 0.0) {
          xi = x2;
          yi = y2;
        } else {
          double denom = vx1 * vx1 + vy1 * vy1;
          if (denom <= 1e-12) {
            xi = x1;
            yi = y1;
          } else {
            double t = (vx0 * vx1 + vy0 * vy1) / denom;
            xi = x1 + t * vx1;
            yi = y1 + t * vy1;
          }
        }

        double dx = x0 - xi;
        double dy = y0 - yi;
        double di2 = dx * dx + dy * dy;

        if (di2 < Short) {
          Short = di2;
          Xnear = xi;
          Ynear = yi;
        }
      }

      B(j, 0) = Xnear;
      B(j, 1) = Ynear;
    }

    VectorXd Amean = A.colwise().mean();
    VectorXd Bmean = B.colwise().mean();
    MatrixXd Aresult = A.rowwise() - Amean.transpose();
    MatrixXd Bresult = B.rowwise() - Bmean.transpose();

    MatrixXd H = (Aresult.transpose()) * Bresult;

    JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();

    MatrixXd Rm = (U * V.transpose()).transpose();  // = V * U^T

    // （従来の判定を維持）
    if (Rm(0, 1) * Rm(1, 0) > 0) {
      Rm = MatrixXd::Identity(2, 2);
    }

    VectorXd xy = Bmean - Rm * Amean;

    MatrixXd Transmatrix(3, 3);
    Transmatrix <<
      Rm(0, 0), Rm(0, 1), xy(0),
      Rm(1, 0), Rm(1, 1), xy(1),
      0,        0,        1;

    totalTrans0 = Transmatrix * totalTrans;
    totalTrans  = totalTrans0;

    // A を更新（同次座標にしてから変換）
    A.conservativeResize(Arows, Acols + 1);
    A.col(Acols) = VectorXd::Ones(Arows);

    MatrixXd newA = A * (Transmatrix.transpose());
    A = newA;
    A.conservativeResize(Arows, Acols);
  }

  double dx  = totalTrans(0, 2);
  double dy  = totalTrans(1, 2);
  double dth = std::atan2(totalTrans(1, 0), totalTrans(0, 0));

  // world 増分に戻す（従来式維持）
  VectorXd calc(3);
  calc <<  dx * std::cos(th) - dy * std::sin(th),
           dx * std::sin(th) + dy * std::cos(th),
          dth;

  return calc;
}

// =====================================================
// ICP による自己位置推定ノード
//   入力 : "robot_pose"  (Float32MultiArray, [x,y,z,qw,qx,qy,qz]) 200Hz
//          "lidar_points" (Float32MultiArray, rows=N, cols=2)       40Hz
//   出力 : "robot_pose_icp" (Float32MultiArray, [x,y,th])
//          "lidar_to_imu"   (Float32MultiArray, [dt,dx,dy,0,0,0,dth])
// =====================================================
class IcpLocalizationNode : public rclcpp::Node
{
public:
  IcpLocalizationNode()
  : rclcpp::Node("icp_localization_node"),
    x_(0.2),
    y_(0.2),
    th_(0.0),
    dx_(0.0),
    dy_(0.0),
    dth_(0.0),
    dt_(0.01)
  {
    // --- マップ（線分群）を CSV から読み込み ---
    std::string pkg_share = ament_index_cpp::get_package_share_directory("lidar_imu");

    std::string csv_path  = pkg_share + "/lidar_1211.csv";

    RCLCPP_INFO(this->get_logger(), "Map CSV path: %s", csv_path.c_str());

    lines_ = my::readCSV(csv_path);
    if (lines_.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read csv (lines_ is empty)");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Loaded lines: %ld x %ld",
        static_cast<long>(lines_.rows()),
        static_cast<long>(lines_.cols()));
    }

    // グローバル LINES2 を構築（icp() が参照する）
    LINES2 = my::Lines2(lines_);
    RCLCPP_INFO(
      this->get_logger(),
      "LINES2 size: %ld x %ld",
      static_cast<long>(LINES2.rows()),
      static_cast<long>(LINES2.cols()));

    // robot_pose: [x,y,z,qw,qx,qy,qz] を購読（200Hz）
    sub_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "robot_pose",
      10,
      std::bind(&IcpLocalizationNode::PoseCallback, this, std::placeholders::_1));

    // LiDAR 点群（ロボット座標系, rows=N, cols=2）を購読（40Hz）
    sub_lidar_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "lidar_points",
      10,
      std::bind(&IcpLocalizationNode::lidarCallback, this, std::placeholders::_1));

    // ICP 推定した自己位置を publish
    pub_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "robot_pose_icp",
      10);

    // IMU側へ補正を publish
    pub_to_imu_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "lidar_to_imu",
      10);

    // 初期姿勢を一度 publish
    publishPose();
  }

private:
  // robot_pose から最新の (x,y,th) を保持
  void PoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 7) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "robot_pose length is too short: %zu (need >= 7)", msg->data.size());
      return;
    }

    const double x  = static_cast<double>(msg->data[0]);
    const double y  = static_cast<double>(msg->data[1]);
    const double qw = static_cast<double>(msg->data[3]);
    const double qx = static_cast<double>(msg->data[4]);
    const double qy = static_cast<double>(msg->data[5]);
    const double qz = static_cast<double>(msg->data[6]);

    x_pose_ = x;
    y_pose_ = y;
    th_pose_ = yaw_from_quat(qw, qx, qy, qz);
    pose_valid_ = true;
  }

  void lidarCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const std::size_t n_raw = msg->data.size();
    if (n_raw < 2) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "lidar_points is too small (size=%zu)", n_raw);
      return;
    }

    // 行列サイズ取得 (rows=N, cols=2) を優先
    int rows = 0;
    int cols = 0;
    if (msg->layout.dim.size() >= 2) {
      rows = static_cast<int>(msg->layout.dim[0].size);
      cols = static_cast<int>(msg->layout.dim[1].size);
    } else {
      cols = 2;
      rows = static_cast<int>(n_raw / 2);
    }

    if (cols < 2) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "lidar_points cols (%d) < 2", cols);
      return;
    }
    if (rows * cols > static_cast<int>(n_raw)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "layout rows*cols (%d) > data size (%zu)", rows * cols, n_raw);
      return;
    }

    const int N = rows;
    if (N <= 0) return;

    // LiDAR を 2 x N の行列に詰める（ロボット座標系）
    MatrixXd LiDAR(2, N);
    for (int i = 0; i < N; ++i) {
      const int base = i * cols;
      LiDAR(0, i) = static_cast<double>(msg->data[base + 0]); // x
      LiDAR(1, i) = static_cast<double>(msg->data[base + 1]); // y
    }

    // LiDARの到着間隔で dt を計測
    const rclcpp::Time now = this->get_clock()->now();
    if (!last_time_valid_) {
      last_time_ = now;
      last_time_valid_ = true;
      return;
    }
    dt_ = (now - last_time_).seconds();
    last_time_ = now;
    if (dt_ <= 0.0) return;

    // ICP 初期値は robot_pose が来ていればそれを採用（来ていなければ従来の x_,y_,th_）
    const double x0  = pose_valid_ ? x_pose_  : x_;
    const double y0  = pose_valid_ ? y_pose_  : y_;
    const double th0 = pose_valid_ ? th_pose_ : th_;

    // ICPを1回まわす（戻り値は増分）
    VectorXd est = icp(x0, y0, th0, LiDAR);
    if (est.size() < 3) {
      RCLCPP_WARN(this->get_logger(), "icp() result size < 3, skip update");
      return;
    }

    dx_  = est(0);
    dy_  = est(1);
    dth_ = my::modpi(est(2));

    // 「ICP後の推定姿勢」を内部状態として保持（前回までの構造を維持）
    x_  = x0 + dx_;
    y_  = y0 + dy_;
    th_ = my::modpi(th0 + dth_);

    publishPose();
    publish_to_IMU();
  }

  void publishPose()
  {
    std_msgs::msg::Float32MultiArray out;
    out.data.resize(3);
    out.data[0] = static_cast<float>(x_);
    out.data[1] = static_cast<float>(y_);
    out.data[2] = static_cast<float>(th_);
    pub_pose_->publish(out);
  }

  void publish_to_IMU()
  {
    std_msgs::msg::Float32MultiArray to_imu;
    to_imu.data.resize(7);
    to_imu.data[0] = static_cast<float>(dt_);
    to_imu.data[1] = static_cast<float>(x_);
    to_imu.data[2] = static_cast<float>(y_);
    to_imu.data[3] = static_cast<float>(0.0);
    to_imu.data[4] = static_cast<float>(0.0);
    to_imu.data[5] = static_cast<float>(0.0);
    to_imu.data[6] = static_cast<float>(dth_);
    pub_to_imu_->publish(to_imu);
  }

private:
  // マップの元データ（線分群）: CSV から読み込んだもの
  my::MatrixXd lines_;

  // Sub/Pub
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pose_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_lidar_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    pub_pose_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    pub_to_imu_;

  // ICP推定（公開する側の状態）
  double dt_;
  double x_;
  double y_;

  double th_;
  double dx_;
  double dy_;
  double dth_;

  // robot_pose（IMU統合）から来る最新値（ICP初期値に使用）
  double x_pose_ = x_;
  double y_pose_ = y_;
  double th_pose_ = th_;
  bool   pose_valid_ = false;

  // LiDAR dt 計測
  rclcpp::Time last_time_;
  bool last_time_valid_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IcpLocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
