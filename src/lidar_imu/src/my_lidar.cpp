
#include "my_lidar.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

namespace my {

const double pi = M_PI;

MatrixXd readCSV(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    std::vector<std::vector<double>> data;

    if (file.is_open()) {
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::vector<double> row;
            while (std::getline(ss, value, ',')) {
                if (!value.empty()) {
                    row.push_back(std::stod(value));
                }
            }
            if (!row.empty()) {
                data.push_back(row);
            }
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return MatrixXd();
    }

    if (data.empty()) {
        return MatrixXd();
    }

    MatrixXd matrix(data.size(), data[0].size());
    for (std::size_t i = 0; i < data.size(); ++i) {
        for (std::size_t j = 0; j < data[i].size(); ++j) {
            matrix(static_cast<int>(i), static_cast<int>(j)) = data[i][j];
        }
    }

    return matrix;
}

double modpi(double x) {
    double result = std::fmod(x, 2.0 * pi);
    if (result < -pi) {
        result += 2.0 * pi;
    } else if (result > pi) {
        result -= 2.0 * pi;
    }
    return result;
}

VectorXd linspace(double start, double stop, int num) {
    VectorXd result(num);
    double step = (stop - start) / (num - 1);
    for (int i = 0; i < num; ++i) {
        result[i] = start + i * step;
    }
    return result;
}

MatrixXd lidarlist(double x, double y, double th, const MatrixXd& lines0) {
    int J = static_cast<int>(lines0.rows());
    int beemn = 1000;

    VectorXd thetas = linspace(-1.6, 1.6, beemn);
    Eigen::RowVectorXd lidarxlist = Eigen::RowVectorXd::Zero(beemn);
    Eigen::RowVectorXd lidarylist = Eigen::RowVectorXd::Zero(beemn);

    double R =5.0;
    Eigen::VectorXd closestlist = Eigen::VectorXd::Zero(beemn);

    for (int i = 0; i < beemn; ++i) {
        double xrobo = x;
        double yrobo = y;

        double xbeem = x + R * std::cos(th + thetas(i));
        double ybeem = y + R * std::sin(th + thetas(i));

        double closest_distance0 = R ;
            

        for (int j = 0; j < J; ++j) {
            double x1 = lines0(j, 0);
            double y1 = lines0(j, 1);
            double x2 = lines0(j, 2);
            double y2 = lines0(j, 3);

            double denominator =
                (x1 - x2) * (yrobo - ybeem) -
                (y1 - y2) * (xrobo - xbeem);

            if (denominator == 0.0) {
                continue;
            }

            double t =
                ((x1 - xrobo) * (yrobo - ybeem) -
                 (y1 - yrobo) * (xrobo - xbeem)) / denominator;

            double u =
                -((x1 - x2) * (y1 - yrobo) -
                  (y1 - y2) * (x1 - xrobo)) / denominator;

            if ((0.0 <= t) && (t <= 1.0) && (0.0 <= u) && (u <= 1.0)) {
                double intersection_x = x1 + t * (x2 - x1);
                double intersection_y = y1 + t * (y2 - y1);

                double robo_wall_distance =
                    std::sqrt(std::pow(intersection_x - xrobo, 2.0) +
                              std::pow(intersection_y - yrobo, 2.0));

                if (robo_wall_distance <= closest_distance0) {
                    closest_distance0 = robo_wall_distance;
                }
            }
        }

        double Lnoise = 1.0;
        closestlist(i) = Lnoise * closest_distance0;
        lidarxlist(i) = closestlist(i) * std::cos(thetas(i));
        lidarylist(i) = closestlist(i) * std::sin(thetas(i));
    }

    MatrixXd LiDAR(2, beemn);
    LiDAR << lidarxlist, lidarylist;
    return LiDAR;
}

MatrixXd Lines2(const MatrixXd& LINES) {
    int Linerow = static_cast<int>(LINES.rows());
    MatrixXd LINES_2(2 * Linerow, 3);

    for (int i = 0; i < Linerow; ++i) {
        LINES_2(2 * i, 0)     = LINES(i, 0);
        LINES_2(2 * i, 1)     = LINES(i, 1);
        LINES_2(2 * i, 2)     = 1.0;
        LINES_2(2 * i + 1, 0) = LINES(i, 2);
        LINES_2(2 * i + 1, 1) = LINES(i, 3);
        LINES_2(2 * i + 1, 2) = 1.0;
    }
    return LINES_2;
}

} // namespace my
