#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <vector>
#include <array>
#include <functional>
#include <memory>
#include <thread>
#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using PatternMatchPose = matcher_action_interfaces::action::Matcher;
using GoalHandlePatternMatchPose = rclcpp_action::ClientGoalHandle<PatternMatchPose>;

class DiffDriveNode : public rclcpp::Node
{
public:
    DiffDriveNode() : Node("diff_drive_node")
    {

        // RCLCPP_INFO(this->get_logger(), "Loaded pattern PCD file with %zu points", pattern_cloud_->points.size());

         // Subscriber to /cmd_vel topic
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelocityPublisherNode::cmdVelCallback, this, std::placeholders::_1));

        // Timer to publish the last velocity at 20 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&VelocityPublisherNode::publishVelocity, this));

        // Initialize last velocity to zero
        last_velocity_ = geometry_msgs::msg::Twist();
        
        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        connect();

    }

private:

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_velocity_ = *msg;
    }

    void publishVelocity()
    {
        double v_left, v_right;
        
        double v = last_velocity_.linear.x;  // Linear velocity (m/s)
        double omega = last_velocity_.angular.z;  // Angular velocity (rad/s)

        // Calculate wheel velocities
        double v_left = (2 * v - omega * wheel_base) / (2 * wheel_radius);
        double v_right = (2 * v + omega * wheel_base) / (2 * wheel_radius);

        std::string data = "{\"left_vel\": " + std::to_string(v_left) +
                            ",\"right_vel\": " + std::to_string(v_right) + "}";

        std::string response = send_msg(data + "\n");

        double joint_states_left = (getValueFromJson(response, "left_pos"));
        double joint_states_right = (getValueFromJson(response, "right_pos"));

        // Publish joint states (example)
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.name = {"wheel_left_joint", "wheel_right_joint"};
        joint_state_msg.position = {joint_states_left, joint_states_right};
        joint_states_publisher_->publish(joint_state_msg);

    }

    LibSerial::BaudRate convert_baud_rate()
        {
            return LibSerial::BaudRate::BAUD_115200;
        }

    void connect()
    {  
        timeout_ms_ = timeout_ms;
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate());
    }

    void disconnect()
    {
        serial_conn_.Close();
    }

    bool connected() const
    {
        return serial_conn_.IsOpen();
    }


    std::string send_msg(const std::string &msg_to_send, bool print_output = true)
    {
        serial_conn_.FlushIOBuffers(); // Just in case
        serial_conn_.Write(msg_to_send);

        std::string response = "";
        try
        {
        // Responses end with \r\n so we will read up to (and including) the \n.
        serial_conn_.ReadLine(response, '\n', timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout&)
        {
            std::cerr << "The ReadByte() call has timed out." << std::endl ;
        }

        if (print_output)
        {
        std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
        }

        return response;
    }

    double getValueFromJson(const std::string& data, const std::string& key) 
    {
        std::size_t key_pos = data.find(key);
        if (key_pos == std::string::npos) {
            throw std::runtime_error("Key not found in JSON string");
        }
        std::size_t colon_pos = data.find(":", key_pos);
        if (colon_pos == std::string::npos) {
            throw std::runtime_error("Invalid JSON format");
        }
        std::size_t comma_pos = data.find(",", colon_pos);
        std::size_t end_pos = (comma_pos == std::string::npos) ? data.find("}", colon_pos) : comma_pos;
        std::string value_str = data.substr(colon_pos + 1, end_pos - colon_pos - 1);
        return std::stod(value_str);
    }

    LibSerial::SerialPort serial_conn_;
    std::string data;
    std::string device = "/dev/ttyUSB0";
    int baud_rate = 115200;
    int timeout_ms = 1000;
    double wheel_base = 0.5;
    double wheel_radius = 0.1;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist last_velocity_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;

};

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveNode>());

    rclcpp::shutdown();
    return 0;
}