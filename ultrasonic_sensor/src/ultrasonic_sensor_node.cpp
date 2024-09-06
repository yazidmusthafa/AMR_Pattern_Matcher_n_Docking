#include <libserial/SerialPort.h>
#include <map>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <string>
#include <iostream>
#include <mutex>
#include "custom_msgs/msg/ultrasonic_sensor_data.hpp"
#include <nlohmann/json.hpp>

class SensorNode : public rclcpp::Node
{
public:
    SensorNode() : Node("ultrasonic_sensor_node")
    {
        // Create a callback group that allows concurrent callbacks
        auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        sensor_data_pub_ = this->create_publisher<custom_msgs::msg::UltrasonicSensorData>("/ultrasonic_sensor_data", 10);

        for (int i = 0; i < 6; ++i) {
            sensor_data["sensor" + std::to_string(i)] = nullptr;  // Using -1 to signify no data
        }

        int i = 0;
        for (std::string& serial_port : SERIAL_PORTS){

            connect(serial_port, BAUDRATE, TIMEOUT, i);

            if (!serial_conn_[i]->IsOpen()) {
                std::cerr << "Error opening serial port " << serial_port << std::endl;
            }

            i++;
        }

        // Create timers for each sensor
        for (int i = 0; i < 6; ++i) {
            threads_.emplace_back(std::thread(&SensorNode::read_sensor_data, this, std::ref(serial_conn_[i]), i));
        }

        threads_.emplace_back(std::thread(&SensorNode::publish_sensor_data,this));
        
        for (auto& t : threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
    }

private:
    void read_sensor_data(std::unique_ptr<LibSerial::SerialPort>& serial_device, int i)
    {
        unsigned char request_command = 0xFF;

        while (true){
            serial_device->FlushIOBuffers();
            // RCLCPP_INFO(this->get_logger(), "getting in %i", i);

            std::string request_command_str(1, request_command);
            serial_device->Write(request_command_str);

            // Read data from the sensor
            std::vector<unsigned char> buffer(4);
            serial_device->Read(buffer, buffer.size());

            if (buffer.size() >= 4) {
                // Verify checksum
                unsigned char calculated_checksum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;
                if (calculated_checksum == buffer[3]) {
                    // Process distance

                    double distance = (buffer[1] << 8) + buffer[2];
                    
                    std::lock_guard<std::mutex> lock(mutex_);

                    sensor_data["sensor" + std::to_string(i)] = distance;
                    // RCLCPP_INFO(this->get_logger(), "Time: %ld ms, Distance: %f mm", millis, distance);
                    
                } else {
                    std::cerr << "Checksum mismatch" << std::endl;
                }
            }
        }
        
        // serial_device->Close();

    }

    void publish_sensor_data(){

        auto message = custom_msgs::msg::UltrasonicSensorData();
        while (true){
            sensor_data_ready = true;
            for (const auto& item : sensor_data.items()) {
                // Check if any value is null
                if (item.value().is_null()) {
                    sensor_data_ready = false;  // If any value is null, return false
                    break;
                }
            }
            if (sensor_data_ready){
                message.data  = sensor_data.dump();
                auto now = std::chrono::system_clock::now();
                auto duration = now.time_since_epoch();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                RCLCPP_INFO(this->get_logger(), "Time: %ld ms, Sensor Data: %s ", millis, message.data.c_str());
                sensor_data_pub_->publish(message);
                for (int i = 0; i < 6; ++i) {
                    sensor_data["sensor" + std::to_string(i)] = nullptr;  // Using -1 to signify no data
                }
            }
        }
    }

    void connect(const std::string &serial_device, LibSerial::BaudRate baud_rate, int32_t timeout_ms, int i)
    {  
        std::cout << "port : " << serial_device << std::endl;
        timeout_ms_ = timeout_ms;
        serial_conn_.emplace_back(std::make_unique<LibSerial::SerialPort>());
        serial_conn_[i]->Open(serial_device);
        serial_conn_[i]->SetBaudRate(baud_rate);
    }

    rclcpp::Publisher<custom_msgs::msg::UltrasonicSensorData>::SharedPtr sensor_data_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::thread> threads_;
    std::string SERIAL_PORTS[6] = {"/dev/ttyCH9344USB0", "/dev/ttyCH9344USB1", "/dev/ttyCH9344USB2", "/dev/ttyCH9344USB3", "/dev/ttyCH9344USB4", "/dev/ttyCH9344USB5"};
    std::mutex mutex_;  // Mutex to ensure thread safety
    LibSerial::BaudRate BAUDRATE = LibSerial::BaudRate::BAUD_115200;
    const int TIMEOUT = 2000;  // Timeout for serial reading in seconds
    int timeout_ms_;
    std::vector<std::unique_ptr<LibSerial::SerialPort>> serial_conn_;
    nlohmann::json sensor_data;  // Map to store sensor data
    std::condition_variable data_ready;  // Condition variable for synchronization
    bool sensor_data_ready = false;  // Flag to indicate if all sensors have reported data

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SensorNode>();

    // Create a MultiThreadedExecutor with 7 threads
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 7);
    executor.add_node(node);

    // Spin with multiple threads
    executor.spin();

    rclcpp::shutdown();
    return 0;
}