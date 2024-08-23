#ifndef ROBOTIC_ARM_ARDUINO_COMMS_HPP
#define ROBOTIC_ARM_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
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

  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_servo_values(std::vector<double> &joint_position_, std::vector<double> joint_position_command_)
  {
    std::string temp_data = "{\"servo1\":" + std::to_string(std::round(joint_position_command_[0]*180/3.14159265359)+90) +
                            ",\"servo2\":" + std::to_string(std::round(joint_position_command_[1]*180/3.14159265359)+90) +
                            ",\"servo3\":" + std::to_string(std::round(joint_position_command_[2]*180/3.14159265359)+90) + "}";

    if(temp_data != data){

      data = "{\"servo1\":" + std::to_string(std::round(joint_position_command_[0]*180/3.14159265359)+90) +
            ",\"servo2\":" + std::to_string(std::round(joint_position_command_[1]*180/3.14159265359)+90) +
            ",\"servo3\":" + std::to_string(std::round(joint_position_command_[2]*180/3.14159265359)+90) + "}";


      std::string response = send_msg(data + "\n");

      // RCLCPP_INFO(rclcpp::get_logger("RobotSystem_2w"), "%s", response.c_str());

      joint_position_[0] = (getValueFromJson(response, "servo1")-88)* 3.14159265359/180;
      joint_position_[1] = (getValueFromJson(response, "servo2")-89)* 3.14159265359/180;
      joint_position_[2] = (getValueFromJson(response, "servo3")-89)* 3.14159265359/180;
    }
    // else{
    //   joint_position_[0] = joint_position_command_[0];
    //   joint_position_[1] = joint_position_command_[1];
    //   joint_position_[2] = joint_position_command_[2];
    // }

  }

  void set_motor_values(std::vector<double> joint_position_)
  {
    std::string data = "{\"servo1\":" + std::to_string(joint_position_[0]*180/3.14159265359) +
                        ",\"servo2\":" + std::to_string(joint_position_[1]*180/3.14159265359) +
                        ",\"servo3\":" + std::to_string(joint_position_[2]*180/3.14159265359) + "}";

    // Write JSON string to serial
    // serial_.write(data + "\n");
    send_msg(data + "\n");
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

private:
    LibSerial::SerialPort serial_conn_;
    std::string data;
    int timeout_ms_;
};

#endif // ROBOTIC_ARM_ARDUINO_COMMS_HPP