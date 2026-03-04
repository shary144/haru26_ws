#pragma once

#include <chrono>
#include <future>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include "cobs.hpp"
#include "test.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

namespace usb_command
{
    enum Command : uint8_t
    {
        Normal = 0,
        Negotiation = 1,
        Imu = 4

    };
} // namespace usb_command

namespace odometry_bridge
{

    class OdometryBridge : public rclcpp::Node
    {
    private:
        /////////////Slacan Series Status///////////////////
        // ctrl+c or ros2 shutdown is called.
        bool is_shutdown_ = false;
        // serial port connected but "HelloSlcan" has not been returned yet.
        bool is_connected_ = false;
        // sconection with usbcan series is active. the serial port is new usbcan series.
        bool is_active_ = false;
        /////////////Slacan Series Status///////////////////

        std::string port_name_ = "/dev/odometry";

        std::shared_ptr<boost::asio::io_context> io_context_;
        std::shared_ptr<boost::asio::serial_port> serial_port_;
        // it will prohabit the io_context to stop.
        std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_;

        boost::asio::streambuf read_streambuf_;

        // thread fot running io_context
        std::thread io_context_thread_;

        std::unique_ptr<std::thread> reading_thread_;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

        const int initialize_timeout_ = 1000; // ms
        // port open and setting.
        void initializeSerialPort(const std::string port_name);

        const int handshake_timeout_ = 1000; // ms
        // check the serial deveice is usbcan series.
        bool handshake();

        // convert message from usbcan series, and process it.
        void readingProcess(const std::vector<uint8_t> data);

        void asyncWrite(const usb_command::Command command, const std::vector<uint8_t> data);

        // Directly use is deprecated.
        void asyncWrite(const std::vector<uint8_t> data);

        // you should call this function at once after the connection is established.
        void asyncRead();

        void asyncReadOnce();

        // these function will be called when the data is read from the serial port.
        void readOnceHandler(const boost::system::error_code &error, std::size_t bytes_transferred);
        void readHandler(const boost::system::error_code &error, std::size_t bytes_transferred);

        // these function will be called when the data is written to the serial port. for error handling.
        void writeHandler(const boost::system::error_code &error, std::size_t bytes_transferred);

    public:
        OdometryBridge(const rclcpp::NodeOptions &options);

        // shutfdown process
        void onShutdown()
        {
            is_shutdown_ = true;
            is_active_ = false;
            io_context_->stop();
            serial_port_->close();
            RCLCPP_INFO(this->get_logger(), "END");
        }
    };
}