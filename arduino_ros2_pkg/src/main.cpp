#include <rclcpp/rclcpp.hpp>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>

class ProcessNode : public rclcpp::Node
{
public:
    ProcessNode() : Node("arduino_control_driver")
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        try
        {
            RCLCPP_INFO(this->get_logger(), "Attempting to open serial port: %s", serial_port.c_str());
            serial_connection.setPort(serial_port);
            serial_connection.setBaudrate(baud_rate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_connection.setTimeout(timeout);
            serial_connection.open();

            if (serial_connection.isOpen())
            {
                RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully.", serial_port.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s.", serial_port.c_str());
                rclcpp::shutdown();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while opening serial port: %s", e.what());
            rclcpp::shutdown();
        }

        vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&ProcessNode::twist_callback, this, std::placeholders::_1));
    }
private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        float linear = msg->linear.x;
        float angular = msg->angular.z;

        std::ostringstream data_stream;
        data_stream << linear << "," << angular << "\n";

        std::string data = data_stream.str();

        try {

            serial_connection.write(data);
            // RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", data.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", e.what());
        }
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
    serial::Serial serial_connection;
    std::string serial_port;
    int baud_rate;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                   // Initialize ROS 2 communication

    rclcpp::spin(std::make_shared<ProcessNode>());  // Create the node and spin it

    rclcpp::shutdown();                               // Shutdown ROS 2 communication

    return 0;
}
