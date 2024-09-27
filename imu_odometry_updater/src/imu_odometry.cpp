#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <libserial/SerialPort.h>
#include <string>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class ImuOdometry : public rclcpp::Node
{
public:
    ImuOdometry()
        : Node("imu_odometry"), yaw_(0.0)
    {
        // Subscriber for the original odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diffbot_base_controller/odom", 10,
            std::bind(&ImuOdometry::odomCallback, this, std::placeholders::_1));
        
        // Subscriber for the transform
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/diffbot_base_controller/tf", 10,
            std::bind(&ImuOdometry::tfCallback, this, std::placeholders::_1));

        // Publisher for the modified odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Start the serial communication
        serial_port_.Open(SERIAL_PORT); 
        serial_port_.SetBaudRate(BAUDRATE);

        // Create a timer to read data periodically
        timer_ = this->create_wall_timer(10ms, std::bind(&ImuOdometry::updateImuData, this));
    }

private:
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg)
    {
        // Update the rotation of the transform using IMU yaw
        for (const auto& transform : tf_msg->transforms)
        {
            // Only modify the transform for the "odom" frame
            if (transform.child_frame_id == "base_link")
            {
                // Create a new transform with updated orientation
                geometry_msgs::msg::TransformStamped updated_transform = transform;

                // Set the new rotation based on the yaw from IMU
                double yaw_rad = yaw_ * M_PI / 180.0; // Convert to radians
                tf2::Quaternion quaternion;
                quaternion.setRPY(0.0, 0.0, yaw_rad); // Set roll, pitch, and yaw

                updated_transform.transform.rotation.x = quaternion.x();
                updated_transform.transform.rotation.y = quaternion.y();
                updated_transform.transform.rotation.z = quaternion.z();
                updated_transform.transform.rotation.w = quaternion.w();

                // // Publish the updated transform
                // tf2_msgs::msg::TFMessage tf_message;
                // tf_message.transforms.push_back(updated_transform);
                // tf_pub_->publish(tf_message);

                // Broadcast the updated transform
                tf_broadcaster_->sendTransform(updated_transform);
            }
            else
            {
                tf_broadcaster_->sendTransform(transform);
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        // Use odometry message and modify it if necessary
        auto updated_odom = *odom_msg;  // Make a copy of the odometry message
        
        // Update orientation using the latest yaw from IMU
        double yaw_rad = yaw_ * M_PI / 180.0;  // Convert to radians
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, yaw_rad);  // Set roll, pitch, and yaw

        updated_odom.pose.pose.orientation.x = quaternion.x();
        updated_odom.pose.pose.orientation.y = quaternion.y();
        updated_odom.pose.pose.orientation.z = quaternion.z();
        updated_odom.pose.pose.orientation.w = quaternion.w();

        // Publish the updated odometry
        odom_pub_->publish(updated_odom);

    }

    void updateImuData()
    {
        if (serial_port_.IsOpen())
        {   
            serial_port_.FlushIOBuffers();
            std::string line;
            try
            {
            // Responses end with \r\n so we will read up to (and including) the \n.
            serial_port_.ReadLine(line, '\n', 1000);
            parseImuData(line);
            }
            catch (const LibSerial::ReadTimeout&)
            {
                // std::cerr << "The ReadByte() call has timed out." << std::endl ;
                RCLCPP_WARN(this->get_logger(), "The ReadByte() call has timed out.");
            }
        }
    }

    void parseImuData(const std::string &line)
    {
        // Example: "Yaw: 45.0 Pitch: 0.0 Roll: 0.0"
        std::istringstream ss(line);
        std::string prefix;
        ss >> prefix; // Yaw:
        ss >> yaw_;   // Yaw value
        RCLCPP_INFO(this->get_logger(), "%f", yaw_);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double yaw_;
    std::string SERIAL_PORT = "/dev/ttyUSB0";
    LibSerial::BaudRate BAUDRATE = LibSerial::BaudRate::BAUD_115200;
    LibSerial::SerialPort serial_port_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImuOdometry>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
