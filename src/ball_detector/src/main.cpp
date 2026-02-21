#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "ball_detector/msg/ball.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include <cstdlib> // getenv
#include <filesystem>
#include <string>

using namespace std;

void pray_for_wsl_gui_startup();

cv::Mat und_m, mask_r_m, mask_b_m, mask_y_m;
std::mutex mtx;

// ホームディレクトリを取得する関数
std::string getHomeDirectory() {
        // Linux/macOSは "HOME", Windowsは "USERPROFILE" を確認
        const char* home = std::getenv("HOME");
        if (home == nullptr) {
            home = std::getenv("USERPROFILE");
        }
        return (home != nullptr) ? std::string(home) : "";
    }

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ball_publisher");
    auto logger_ = rclcpp::get_logger("ball_detector");
    RCLCPP_INFO(logger_, "Ball detector node started.");
    auto pub = node->create_publisher<ball_detector::msg::Ball>("/ball_position", 10);
    RCLCPP_INFO(logger_, "Publisher created.");


    bool debug_mode = node->declare_parameter<bool>("debug", false);
    // for (int i = 1; i < argc; i++) {
    //     if (std::string(argv[i]) == "--nondebug") {
    //         debug_mode = false;
    //     }
    // }
    RCLCPP_INFO(logger_, "Debug mode: %s", debug_mode ? "ON" : "OFF");

    // === カメラパラメータ読み込み ===

    std::string homeDir = getHomeDirectory();
    if (homeDir.empty()) {
        std::cerr << "ホームディレクトリが見つかりません" << std::endl;
        return -1;
    }

    filesystem::path basePath(homeDir);
    filesystem::path fullPath = basePath / "haru26_ws" / "src" / "ball_detector" / "camera_calib.yml";
    cv::FileStorage fs(fullPath.string(), cv::FileStorage::READ);//ymlファイルの絶対パスを指定してください
    if (!fs.isOpened()) {
        RCLCPP_ERROR(logger_, "Failed to open camera calibration file!");
        return 0;
    }
    RCLCPP_INFO(logger_, "yml file loaded.");

    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
    RCLCPP_INFO(logger_, "Camera parameters loaded.");

    cameraMatrix.convertTo(cameraMatrix, CV_64F);
    distCoeffs.convertTo(distCoeffs, CV_64F);

    RCLCPP_INFO(logger_, "cameraMatrix size: %dx%d", cameraMatrix.rows, cameraMatrix.cols);
    

    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    RCLCPP_INFO(logger_, "Camera fx=%.2f, fy=%.2f", fx, fy);

    // === カメラ ===

    cv::VideoCapture cap("http://172.29.128.1:8080/video");
    if (!cap.isOpened()) {
        RCLCPP_ERROR(logger_, "Failed to open camera stream!");
        return 0;
    }
    RCLCPP_INFO(logger_, "Camera stream opened.");

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    // === 歪み補正マップ作成 ===
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(
        cameraMatrix, distCoeffs, cv::Mat(),
        cameraMatrix, cv::Size(640, 480), CV_16SC2, map1, map2
    );

    // hsvトラックバー用変数
    static bool trackbar_initialized = false;
    static int h_low_r = 0, h_high_r = 17, s_low_r = 80, s_high_r = 255, v_low_r = 80, v_high_r = 255;
    static int h_low_r2 = 160, h_high_r2 = 179;
    static int h_low_b = 85, h_high_b = 145, s_low_b = 80, s_high_b = 255, v_low_b = 80, v_high_b = 255;
    static int h_low_y = 17, h_high_y = 35, s_low_y = 80, s_high_y = 255, v_low_y = 80, v_high_y = 255;

    // === GUI スレッド === debug_modeがtrueのときだけ起動
    if(debug_mode){
    std::thread viewer([&]() {
            if(!trackbar_initialized) {
                cv::destroyAllWindows();
                cv::waitKey(1000);
                //起動するようにお祈りをする(wsl限定かも？)
                pray_for_wsl_gui_startup();
                cv::namedWindow("result", cv::WINDOW_NORMAL);
                cv::waitKey(1000);
                
                cv::resizeWindow("result", 1920, 1080);

                // 赤
                cv::createTrackbar("R1_H_low", "result", &h_low_r, 179);
                cv::createTrackbar("R1_H_high", "result", &h_high_r, 179);
                cv::createTrackbar("R_S_low", "result", &s_low_r, 255);
                cv::createTrackbar("R_S_high", "result", &s_high_r, 255);
                cv::createTrackbar("R_V_low", "result", &v_low_r, 255);
                cv::createTrackbar("R_V_high", "result", &v_high_r, 255);

                cv::createTrackbar("R2_H_low", "result", &h_low_r2, 179);
                cv::createTrackbar("R2_H_high", "result", &h_high_r2, 179);
                // 青
                cv::createTrackbar("B_H_low", "result", &h_low_b, 179);
                cv::createTrackbar("B_H_high", "result", &h_high_b, 179);
                cv::createTrackbar("B_S_low", "result", &s_low_b, 255);
                cv::createTrackbar("B_S_high", "result", &s_high_b, 255);
                cv::createTrackbar("B_V_low", "result", &v_low_b, 255);
                cv::createTrackbar("B_V_high", "result", &v_high_b, 255);
                // 黄
                cv::createTrackbar("Y_H_low", "result", &h_low_y, 179);
                cv::createTrackbar("Y_H_high", "result", &h_high_y, 179);
                cv::createTrackbar("Y_S_low", "result", &s_low_y, 255);
                cv::createTrackbar("Y_S_high", "result", &s_high_y, 255);
                cv::createTrackbar("Y_V_low", "result", &v_low_y, 255);
                cv::createTrackbar("Y_V_high", "result", &v_high_y, 255);
                trackbar_initialized = true;
            }

        while (rclcpp::ok()) {
            cv::Mat r, b, y, u;

            {
                std::lock_guard<std::mutex> lock(mtx);
                r = mask_r_m.clone();
                b = mask_b_m.clone();
                y = mask_y_m.clone();
                u = und_m.clone();
            }

          
                if (r.channels() == 1) cv::cvtColor(r, r, cv::COLOR_GRAY2BGR);
                if (b.channels() == 1) cv::cvtColor(b, b, cv::COLOR_GRAY2BGR);
                if (y.channels() == 1) cv::cvtColor(y, y, cv::COLOR_GRAY2BGR);
                if (u.channels() == 1) cv::cvtColor(u, u, cv::COLOR_GRAY2BGR);

                cv::Size size(320, 240);
                cv::resize(r, r, size);
                cv::resize(b, b, size);
                cv::resize(y, y, size);
                cv::resize(u, u, size);

                int w = size.width;
                int h = size.height;

                // 区切り線の太さ
                int lineThickness = 3;

                // 最終画像のサイズ（線の分も足す）
                cv::Mat canvas(h * 2 + lineThickness,  // 2段 + 横線
                            w * 2 + lineThickness,  // 2列 + 縦線
                            CV_8UC3,
                            cv::Scalar(0, 0, 0));   // 背景色（黒）

                // --- 画像を貼り付ける ---
                u.copyTo(canvas(cv::Rect(0, 0, w, h)));
                r.copyTo(canvas(cv::Rect(w + lineThickness, 0, w, h)));
                b.copyTo(canvas(cv::Rect(0, h + lineThickness, w, h)));
                y.copyTo(canvas(cv::Rect(w + lineThickness, h + lineThickness, w, h)));

                // --- 区切り線（白） ---
                cv::line(canvas, {w, 0}, {w, canvas.rows}, cv::Scalar(255,0,0), lineThickness);  // 縦線
                cv::line(canvas, {0, h}, {canvas.cols, h}, cv::Scalar(255,0,0), lineThickness);  // 横線

                // --- ラベル（名前）を付ける ---
                cv::putText(canvas, "und", {10, 30},
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

                cv::putText(canvas, "mask_r", {w + lineThickness + 10, 30},
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

                cv::putText(canvas, "mask_b", {10, h + lineThickness + 30},
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

                cv::putText(canvas, "mask_y", {w + lineThickness + 10, h + lineThickness + 30},
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

                cv::imshow("result", canvas);
               

                if(cv::waitKey(1) == 'q') {
                    rclcpp::shutdown();
                    break;
                }
        }
    });
    
    viewer.detach();
    }

    // === メインループ ===
    cv::Mat frame, und, hsv;
    constexpr double ball_radius_m = 0.095; // ボールの実際の半径[m] 後で変えてね

    while (rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) break;

        cv::remap(frame, und, map1, map2, cv::INTER_LINEAR);
        cv::cvtColor(und, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask_r1, mask_r2, mask_r, mask_b, mask_y;

        cv::inRange(hsv, cv::Scalar(h_low_r, s_low_r, v_low_r), cv::Scalar(h_high_r, s_high_r, v_high_r), mask_r1);
        cv::inRange(hsv, cv::Scalar(h_low_r2, s_low_r, v_low_r), cv::Scalar(h_high_r2, s_high_r, v_high_r), mask_r2);
        cv::bitwise_or(mask_r1, mask_r2, mask_r);

        cv::inRange(hsv, cv::Scalar(h_low_b, s_low_b, v_low_b), cv::Scalar(h_high_b, s_high_b, v_high_b), mask_b);
        cv::inRange(hsv, cv::Scalar(h_low_y, s_low_y, v_low_y), cv::Scalar(h_high_y, s_high_y, v_high_y), mask_y);
        // ノイズ除去
        cv::morphologyEx(mask_r, mask_r, cv::MORPH_OPEN, {}, {-1,-1}, 2);
        cv::morphologyEx(mask_r, mask_r, cv::MORPH_CLOSE, {}, {-1,-1}, 2);

        cv::morphologyEx(mask_b, mask_b, cv::MORPH_OPEN, {}, {-1,-1}, 2);
        cv::morphologyEx(mask_b, mask_b, cv::MORPH_CLOSE, {}, {-1,-1}, 2);

        cv::morphologyEx(mask_y, mask_y, cv::MORPH_OPEN, {}, {-1,-1}, 2);
        cv::morphologyEx(mask_y, mask_y, cv::MORPH_CLOSE, {}, {-1,-1}, 2);

        

        // --- ボール検出---
        std::vector<std::vector<cv::Point>> contours_r, contours_b, contours_y;
        cv::findContours(mask_r, contours_r, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(mask_b, contours_b, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(mask_y, contours_y, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        //キメラ合体コード
        std::vector<std::tuple<cv::Mat,std::string,std::vector<std::vector<cv::Point>>>> masks = {
            {mask_r, "red", contours_r},
            {mask_b, "blue", contours_b},
            {mask_y, "yellow", contours_y}
        };

        for (const auto& m : masks) {//各色のマスクごとに処理
            for (const auto& contour : std::get<2>(m)) {//各輪郭ごとに処理
                double area = cv::contourArea(contour);
                if (area < 300 || 640 * 480 / 2 < area) continue;

                double perimeter = cv::arcLength(contour, true);
                double circularity = 4 * CV_PI * area / (perimeter * perimeter);
                if (circularity < 0.7) continue;

                cv::Moments mu = cv::moments(contour);
                cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
                float radius_px;
                cv::minEnclosingCircle(contour, center, radius_px);

                std::string color = std::get<1>(m);

                // --- PnP（簡易） ---
                double Z_cam = (fy * ball_radius_m) / radius_px;
                double X_cam = -(center.x - cx) * Z_cam / fx;
                double Y_cam = -(center.y - cy) * Z_cam / fy;

                // --- カメラ→ロボット座標変換 ---
                // カメラ位置・姿勢（仮定：後で変更可能）
                double theta_x = 30.0 * M_PI / 180.0; // 下向き30度
                double theta_y = 0.0;
                double theta_z = 0.0;
                double x_0 = 0; //横オフセット[m]
                double y_0 = 0.48; //上下オフセット[m]
                double z_0 = 0.0; //前後オフセット[m]


                double X_robot_ = X_cam + x_0;
                double Y_robot_ = Y_cam*cos(theta_x) - Z_cam*sin(theta_x) + y_0;
                double Z_robot_ = Y_cam*sin(theta_x) + Z_cam*cos(theta_x) + z_0;
                double X_robot = -X_robot_;
                double Y_robot = Z_robot_;
                double Z_robot =  Y_robot_;

                RCLCPP_INFO(rclcpp::get_logger("ball_detector"), "%s ball: Camera(XYZ)=(%.3f, %.3f, %.3f) Robot(XYZ)=(%.3f, %.3f, %.3f)",
                    color.c_str(), X_cam, Y_cam, Z_cam, X_robot, Y_robot, Z_robot
                );

                if(debug_mode){
                    cv::circle(und, center, (int)radius_px, cv::Scalar(0, 255, 0), 2);
                    cv::putText(und, color, center + cv::Point2f(5, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                }

                // publish
                ball_detector::msg::Ball msg;
                msg.header.stamp = node->now();
                msg.header.frame_id = "robot_base";
                msg.position.x = X_robot;
                msg.position.y = Y_robot;
                msg.position.z = Z_robot;
                msg.color = color;
                pub->publish(msg);
            }
        }

        // --- 描画用に画像共有 ---
        {
            std::lock_guard<std::mutex> lock(mtx);
            und.copyTo(und_m);
            mask_r.copyTo(mask_r_m);
            mask_b.copyTo(mask_b_m);
            mask_y.copyTo(mask_y_m);
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}

void pray_for_wsl_gui_startup() {
    // WSL環境でGUIアプリケーションが正しく起動するように祈る関数
    "love and peace for everyone";
    "thank you, universe!";
    "hope for the best!";
    "sending positive vibes to WSL GUI subsystem";
    "increasing my luck now!";
    "imagining successful GUI startup!";
}