#pragma once

#include <Eigen/Dense>
#include <string>

namespace my {

extern const double pi;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// CSV を読み込んで MatrixXd を返す
MatrixXd readCSV(const std::string& filename);

// [-pi, pi] への正規化
double modpi(double x);

// 等間隔ベクトル生成
VectorXd linspace(double start, double stop, int num);

// LiDAR レイキャスト（ロボット座標系で 2×N の点群を返す）
// lines0: 各行 [x1, y1, x2, y2]
MatrixXd lidarlist(double x, double y, double th, const MatrixXd& lines0);

// 線分 [x1,y1,x2,y2] → 端点リスト [x,y,1]×2 に展開
MatrixXd Lines2(const MatrixXd& LINES);

} // namespace my
