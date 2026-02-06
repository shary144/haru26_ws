#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "robomas_package_2/msg/motor_feedback.hpp"
#include "robomas_package_2/msg/can_frame.hpp"
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

//////// メッセージ送信周波数（ユーザー定義）begin

// 1 ~ 500Hz まで設定可能
const int Hz = 10;

//////// メッセージ送信周波数（ユーザー定義）end

int trigger = 500 / Hz;
const int32_t ENC_COUNTS_PER_REV = 8192;

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("can_receiver_can1");
	auto pub1 = node->create_publisher<robomas_package_2::msg::MotorFeedback>("motor_rx_1_can1", 10);
    auto pub2 = node->create_publisher<robomas_package_2::msg::MotorFeedback>("motor_rx_2_can1", 10);
    auto pub3 = node->create_publisher<robomas_package_2::msg::MotorFeedback>("motor_rx_3_can1", 10);
    auto pub4 = node->create_publisher<robomas_package_2::msg::MotorFeedback>("motor_rx_4_can1", 10);
    auto pub5 = node->create_publisher<robomas_package_2::msg::MotorFeedback>("motor_rx_5_can1", 10);
    auto pub6 = node->create_publisher<robomas_package_2::msg::MotorFeedback>("motor_rx_6_can1", 10);
    auto pub7 = node->create_publisher<robomas_package_2::msg::MotorFeedback>("motor_rx_7_can1", 10);
    auto pub8 = node->create_publisher<robomas_package_2::msg::MotorFeedback>("motor_rx_8_can1", 10);

    auto pub_can_ = node->create_publisher<robomas_package_2::msg::CanFrame>("can_frame_rx_can1", 10);

    const char * ifname = (argc > 1) ? argv[1] : "can1";

    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create CAN socket");
        return -1;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(node->get_logger(), "ioctl SIOCGIFINDEX failed for interface %s", ifname);
        close(sock);
        return -1;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(node->get_logger(), "bind failed for interface %s", ifname);
        close(sock);
        return -1;
    }

	struct can_frame frame;

	long long unsigned int count[8] = {0,0,0,0,0,0,0,0};

    uint16_t prev_raw[8] = {0};
    bool first_recv[8] = {true, true, true, true, true, true, true, true};
    int64_t cum_ticks[8] = {0}; // 累積差分（ticks）

	while (rclcpp::ok()) {
		if (read(sock, &frame, sizeof(frame)) < 0) continue;

		if(frame.can_id >= 0x201 && frame.can_id <= 0x208){
            robomas_package_2::msg::MotorFeedback msg;

			msg.id = (uint8_t)(frame.can_id - (uint8_t)0x200);

			// 生の角度（16bit）の取得
            uint16_t raw_angle = static_cast<uint16_t>((frame.data[0] << 8) | frame.data[1]);

            // 差分計算（ラップ補正）
            if (first_recv[msg.id - 1]) {
                prev_raw[msg.id - 1] = raw_angle;
                first_recv[msg.id - 1] = false;
                // 初回は差分0として扱う
            } else {
                int32_t delta = static_cast<int32_t>(raw_angle) - static_cast<int32_t>(prev_raw[msg.id - 1]);

                int32_t half = ENC_COUNTS_PER_REV / 2;
                if (delta > half) {
                    delta -= ENC_COUNTS_PER_REV;
                } else if (delta < -half) {
                    delta += ENC_COUNTS_PER_REV;
                }

                cum_ticks[msg.id - 1] += delta;
                prev_raw[msg.id - 1] = raw_angle;
            }

            // 回転数（回）を計算してメッセージに入れる
            float revolutions = static_cast<float>(cum_ticks[msg.id - 1]) / static_cast<float>(ENC_COUNTS_PER_REV);
            msg.revolutions = revolutions; // <-- MotorFeedback にこのフィールドを追加しておく

            // 既存の field も残したいなら保持する（raw angle など）
            msg.angle = raw_angle;

			uint16_t rec_rotational_speed = (static_cast<uint16_t>((frame.data[2]) << 8) | frame.data[3]);
			std::memcpy(&msg.rotational_speed, &rec_rotational_speed, sizeof(msg.rotational_speed));

			uint16_t rec_current = (static_cast<uint16_t>((frame.data[4]) << 8) | frame.data[5]);
			std::memcpy(&msg.current, &rec_current, sizeof(msg.current));

			msg.motor_temperature = frame.data[6];

            count[msg.id - 1]++;
            if(count[msg.id - 1] % trigger == 0){
                if(msg.id == 1) pub1->publish(msg);
                else if(msg.id == 2) pub2->publish(msg);
                else if(msg.id == 3) pub3->publish(msg);
                else if(msg.id == 4) pub4->publish(msg);
                else if(msg.id == 5) pub5->publish(msg);
                else if(msg.id == 6) pub6->publish(msg);
                else if(msg.id == 7) pub7->publish(msg);
                else if(msg.id == 8) pub8->publish(msg);
            }
		}else if(frame.can_id != 0x0FF && frame.can_id != 0x100 && frame.can_id != 0x101 &&
                 frame.can_id != 0x102 && frame.can_id != 0x103 && frame.can_id != 0x104 &&
                 frame.can_id != 0x105 && frame.can_id != 0x106 && frame.can_id != 0x107 &&
                 frame.can_id != 0x108 && frame.can_id != 0x1FF && frame.can_id != 0x200){
            robomas_package_2::msg::CanFrame can_msg;
            can_msg.id = frame.can_id;
            can_msg.dlc = frame.can_dlc;
            for(int i = 0; i < 8; i++){
                can_msg.data[i] = frame.data[i];
            }
            pub_can_->publish(can_msg);
        }
	}

	close(sock);
	rclcpp::shutdown();
	return 0;
}
