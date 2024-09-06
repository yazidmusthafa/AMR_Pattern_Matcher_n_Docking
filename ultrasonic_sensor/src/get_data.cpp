#include <libserial/SerialPort.h>
#include <thread>
#include <map>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <iostream>
#include <mutex>

using namespace LibSerial;

std::string SERIAL_PORTS[6] = {"/dev/ttyCH9344USB0", "/dev/ttyCH9344USB1", "/dev/ttyCH9344USB2", "/dev/ttyCH9344USB3", "/dev/ttyCH9344USB4", "/dev/ttyCH9344USB5"};

LibSerial::BaudRate BAUDRATE = LibSerial::BaudRate::BAUD_115200;
const int TIMEOUT = 2000;  // Timeout for serial reading in seconds
int timeout_ms_;
std::vector<std::unique_ptr<LibSerial::SerialPort>> serial_conn_;
std::map<std::string, int> sensor_data;  // Map to store sensor data
std::mutex mtx;  // Mutex for thread safety
std::condition_variable data_ready;  // Condition variable for synchronization
bool all_sensors_ready = false;  // Flag to indicate if all sensors have reported data


void connect(const std::string &serial_device, LibSerial::BaudRate baud_rate, int32_t timeout_ms, int i)
  {  
    std::cout << "port : " << serial_device << std::endl;
    timeout_ms_ = timeout_ms;
    serial_conn_.emplace_back(std::make_unique<LibSerial::SerialPort>());
    serial_conn_[i]->Open(serial_device);
    serial_conn_[i]->SetBaudRate(baud_rate);
  }

void initialize_sensor_data() {
    std::lock_guard<std::mutex> lock(mtx);
    for (int i = 0; i < 8; ++i) {
        sensor_data["sensor" + std::to_string(i)] = -1;  // Using -1 to signify no data
    }
}

void read_range_sensor(std::unique_ptr<LibSerial::SerialPort>& serial_device){
    unsigned char request_command = 0xFF;

    while (true) {
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

                int distance = (buffer[1] << 8) + buffer[2];
                auto now = std::chrono::system_clock::now();
                auto duration = now.time_since_epoch();
                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                
                std::cout << "Time: " << millis << " ms, Distance: " << distance << " mm" << std::endl;
            } else {
                std::cerr << "Checksum mismatch" << std::endl;
            }
        }

        // Wait for a while before requesting data again
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    serial_device->Close();
}

int main() {

    int i = 0;
    for (std::string& serial_port : SERIAL_PORTS){

        connect(serial_port, BAUDRATE, TIMEOUT, i);

        if (!serial_conn_[i]->IsOpen()) {
            std::cerr << "Error opening serial port " << serial_port << std::endl;
            return EXIT_FAILURE;
        }

        i++;
    }

    read_range_sensor(serial_conn_[0]);

    return 0;
}
