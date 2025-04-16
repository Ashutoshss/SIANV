// Author: Ashutosh Singh
// April 6, 2025

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <cmath>

class InertialMeasurementUnit : public rclcpp::Node
{
public:
    InertialMeasurementUnit() : Node("imu_driver_node")
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();

        try
        {
            serial_conn_.setPort(serial_port_);
            serial_conn_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_conn_.setTimeout(timeout);
            serial_conn_.open();

            if (serial_conn_.isOpen())
            {
                RCLCPP_INFO(this->get_logger(), "Serial port opened: %s", serial_port_.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
                rclcpp::shutdown();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial Exception: %s", e.what());
            rclcpp::shutdown();
        }

        imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&InertialMeasurementUnit::Process, this));
    }

private:
    void Process()
    {
        if (serial_conn_.available())
        {
            std::string line = serial_conn_.readline(1024, "\n");
            std::stringstream ss(line);
            std::string token;
            std::vector<float> data;

            while (std::getline(ss, token, ','))
            {
                try
                {
                    data.push_back(std::stof(token));
                }
                catch (...)
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid float in serial data: %s", token.c_str());
                    return;
                }
            }

            if (data.size() != 10)
            {
                RCLCPP_WARN(this->get_logger(), "Expected 10 float values, got %zu", data.size());
                return;
            }

            sensor_msgs::msg::Imu imu_msg;

            // Orientation (quaternion)
            imu_msg.orientation.x = data[0];
            imu_msg.orientation.y = data[1];
            imu_msg.orientation.z = data[2];
            imu_msg.orientation.w = data[3];

            // Linear acceleration (m/s^2)
            imu_msg.linear_acceleration.x = data[4];
            imu_msg.linear_acceleration.y = data[5];
            imu_msg.linear_acceleration.z = data[6];

            // Angular velocity (rad/s)
            imu_msg.angular_velocity.x = data[7];
            imu_msg.angular_velocity.y = data[8];
            imu_msg.angular_velocity.z = data[9];

            imu_msg.header.stamp = this->get_clock()->now();
            imu_msg.header.frame_id = "base_link";  

            imu_publisher->publish(imu_msg);
        }
    }


    serial::Serial serial_conn_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string serial_port_;
    int baud_rate_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                   // Initialize ROS 2 communication

    rclcpp::spin(std::make_shared<InertialMeasurementUnit>());  // Create the node and spin it

    rclcpp::shutdown();                               // Shutdown ROS 2 communication

    return 0;
}