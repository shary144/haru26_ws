#include <odometry_plugins/odometry_bridge.hpp>

namespace odometry_bridge
{

    OdometryBridge::OdometryBridge(const rclcpp::NodeOptions &options) : Node("odometry_bridge", options)
    {
        RCLCPP_INFO(get_logger(), "OdometryBridge init is started.");
        rclcpp::on_shutdown([this]()
                            { this->onShutdown(); });
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // initalize asio members
        io_context_ = std::make_shared<boost::asio::io_context>();
        serial_port_ = std::make_shared<boost::asio::serial_port>(io_context_->get_executor());
        work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_context_->get_executor());
        // start io_context thread
        io_context_thread_ = std::thread([this]()
                                         {
                                             io_context_->run();
                                             RCLCPP_INFO(this->get_logger(), "io_context_->run() is finished."); });

        RCLCPP_INFO(get_logger(), "OdometryBridge is initialized.");

        initializeSerialPort(port_name_);
        asyncRead();
        handshake();
    }

    // port open and setting.
    void OdometryBridge::initializeSerialPort(const std::string port_name)
    {
        rclcpp::WallRate rate(10ms);
        while (!is_shutdown_)
        {
            try
            {
                serial_port_->open(port_name);
                serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
                serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                serial_port_->set_option(boost::asio::serial_port_base::baud_rate(115200));
            }
            catch (boost::system::system_error &e)
            {
                switch (e.code().value())
                {
                case 2:
                    RCLCPP_ERROR(get_logger(), "Cannot connect. No such file or directory");
                    break;
                case 13:
                    RCLCPP_ERROR(get_logger(), "Cannot connect. Permission denied");
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Cannot connect. %s", e.what());
                    serial_port_->close();
                    break;
                }
            }
            if (serial_port_->is_open())
            {
                RCLCPP_INFO(get_logger(), "connected");
                is_connected_ = true;
                break;
            }
            rate.sleep();
        }
        return;
    }

    void OdometryBridge::asyncWrite(const std::vector<uint8_t> data)
    {
        io_context_->post([this, data]()
                          { boost::asio::async_write(*serial_port_, boost::asio::buffer(data),
                                                     boost::bind(&OdometryBridge::writeHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)); });
        return;
    }

    void OdometryBridge::asyncWrite(const usb_command::Command command, const std::vector<uint8_t> data)
    {
        if (command == usb_command::Normal)
            RCLCPP_ERROR(get_logger(), "asyncWrite(Command) can not use normal. you need to use asyncWrite(Frame)");

        // data structure
        /*
        uint8_t command & frame_type: (command: if it is normal can frame, it is 0x00.)<<4 | is_rtr << 2 | is_extended << 1 | is_error
        uint8_t id[] : data
        */
        std::vector<uint8_t> raw_data(1 + data.size());
        raw_data[0] = (command << 4);
        for (std::size_t i = 0; i < data.size(); i++)
        {
            raw_data[1 + i] = data[i];
        }
        std::vector<uint8_t> output = cobs::encode(raw_data);

        asyncWrite(output);
    }

    void OdometryBridge::readingProcess(const std::vector<uint8_t> data)
    {
        try {
            std::vector<uint8_t> cobs_output_buffer_ = cobs::decode(data);

            RCLCPP_INFO(get_logger(), "readingProcess %s", test::hex_to_string(cobs_output_buffer_).c_str());

            // check it is handshake. OdometryBoard will send "HelloSlcan" when the connection is established.
            constexpr uint8_t HelloSlcan[] = {usb_command::Negotiation << 4, 'H', 'e', 'l', 'l', 'o', 'S', 'L', 'C', 'A', 'N'};
            if (cobs_output_buffer_.size() == 10 + 1)
            {
                bool is_handshake = true;
                for (int i = 0; i < 10; i++)
                {
                    if (cobs_output_buffer_[i] != HelloSlcan[i])
                    {
                        is_handshake = false;
                        break;
                    }
                }
                if (is_handshake)
                {
                    RCLCPP_INFO(get_logger(), "negotiation success");
                    is_active_ = true;
                    return;
                }
            }

            // publish the data to the topic.
            if (data.size() < 12)
            {
                RCLCPP_ERROR(get_logger(), "data size is too small");
                return;
            }

            // data structure
            struct IMUData{
                uint32_t timestamp=0;
                float accelX=0;
                float accelY=0;
                float accelZ=0;
                float gyroX=0;
                float gyroY=0;
                float gyroZ=0;
                uint8_t checksum=0;
            };
            IMUData imu_data;
            std::memcpy(&imu_data, &cobs_output_buffer_[1], sizeof(IMUData));
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp = this->now();
            msg->header.frame_id = std::to_string(imu_data.timestamp);
            msg->orientation.x = 0.0;
            msg->orientation.y = 0.0;
            msg->orientation.z = 0.0;
            msg->orientation.w = 0.0;
            msg->orientation_covariance[0] = 0.0;
            msg->angular_velocity.x = imu_data.gyroX;
            msg->angular_velocity.y = imu_data.gyroY;
            msg->angular_velocity.z = imu_data.gyroZ;
            msg->angular_velocity_covariance[0] = 0.0;
            msg->linear_acceleration.x = imu_data.accelX;
            msg->linear_acceleration.y = imu_data.accelY;
            msg->linear_acceleration.z = imu_data.accelZ;
            msg->linear_acceleration_covariance[0] = 0.0;
            imu_pub_->publish(std::move(msg));
        }
        catch(const cobs::CobsError & e) {
            RCLCPP_WARN(this->get_logger(), "Error Ocurred: %s", e.what());
        }
    }

    bool OdometryBridge::handshake()
    {
        rclcpp::WallRate rate(10ms);
        while (!is_active_ && !is_shutdown_)
        {
            const std::vector<uint8_t> HelloUSBCAN = {'H', 'e', 'l', 'l', 'o', 'U', 'S', 'B', 'C', 'A', 'N'};
            asyncWrite(usb_command::Negotiation, HelloUSBCAN);
            RCLCPP_INFO(get_logger(), "Waiting for negotiation...");
            rate.sleep();
        }
        return true;
    }

    void OdometryBridge::readOnceHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
    {
        if (error)
        {
            RCLCPP_ERROR(get_logger(), "readOnceHandler error %s", error.message().c_str());
            return;
        }

        std::vector<uint8_t> data(bytes_transferred);

        // it can use iostream but
        const uint8_t *data_ptr = (const uint8_t *)boost::asio::buffer_cast<const char *>(read_streambuf_.data());
        for (std::size_t i = 0; i < bytes_transferred; i++)
        {
            data[i] = data_ptr[i];
        }

        OdometryBridge::readingProcess(data);

        // RCLCPP_INFO(get_logger(),"readOnceHandler %s",test::hex_to_string(data,bytes_transferred).c_str());
        read_streambuf_.consume(bytes_transferred);
        return;
    }

    void OdometryBridge::readHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
    {
        // end of file; it means usb disconnect.
        if (error == boost::asio::error::eof)
        {
            is_active_ = false;
            is_connected_ = false;
            // reconnect
            serial_port_->close();
            io_context_->reset();
            serial_port_.reset();
            work_guard_.reset();
            io_context_thread_.detach();
            // initalize asio members
            serial_port_ = std::make_shared<boost::asio::serial_port>(io_context_->get_executor());
            work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_context_->get_executor());
            // start io_context thread
            io_context_thread_ = std::thread([this]()
                                             {
                                                 io_context_->run();
                                                 RCLCPP_INFO(this->get_logger(), "io_context_->run() is finished."); });

            initializeSerialPort(port_name_);
            asyncRead();
            handshake();
            return;
        }
        readOnceHandler(error, bytes_transferred);
        asyncRead();

        return;
    }

    // write data to the serial port. it calls asyncReadOnce() after reading.
    void OdometryBridge::asyncReadOnce()
    {
        // read and write functions can worl in the same time.
        // so, it is not necessary to use io_context_->post()      (this is a strand.)
        boost::asio::async_read_until(*serial_port_, read_streambuf_, '\0',
                                      boost::bind(&OdometryBridge::readOnceHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        return;
    }

    void OdometryBridge::writeHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
    {
        if (error)
        {
            RCLCPP_ERROR(get_logger(), "writeHandler error: tried to write %ld byte", bytes_transferred);

            // the followings are generated by copilot.
            // TODO:CHECK IT!
            switch (error.value())
            {
            case boost::system::errc::no_such_device_or_address:
                RCLCPP_ERROR(get_logger(), "no_such_device_or_address");
                break;
            case boost::system::errc::no_such_file_or_directory:
                RCLCPP_ERROR(get_logger(), "no_such_file_or_directory");
                break;
            case boost::system::errc::permission_denied:
                RCLCPP_ERROR(get_logger(), "permission_denied");
                break;
            case boost::system::errc::bad_file_descriptor:
                RCLCPP_ERROR(get_logger(), "bad_file_descriptor");
                break;
            case boost::system::errc::resource_unavailable_try_again:
                RCLCPP_ERROR(get_logger(), "resource_unavailable_try_again");
                break;
            default:
                RCLCPP_ERROR(get_logger(), "unknown error");
                break;
            }
        }
    }

    // write data to the serial port. it calls asyncRead() after reading.
    void OdometryBridge::asyncRead()
    {
        // read and write functions can worl in the same time.
        // so, it is not necessary to use io_context_->post()      (this is a strand.)
        boost::asio::async_read_until(*serial_port_, read_streambuf_, '\0',
                                      boost::bind(&OdometryBridge::readHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        return;
    }

} // namespace odometry_bridge
