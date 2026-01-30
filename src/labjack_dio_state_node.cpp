/**
 * @file labjack_dio_state_node.cpp
 * @author Robert Raye (robert.raye@nlr.gov)
 * @brief ros node for reading digital input values from a Labjack T8 device
 * @version 0.1
 * @date 2024-10-02
 * 
 * @copyright Copyright (c) 2024
 * 
*/
/**
  Purpose:
    1. Read the state of the DIO on the T8
    2. Publish those states

*/

// #include <functional>
// #include <memory>
#include <chrono>
#include <stdio.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "modaq_messages/msg/t8dio.hpp"

#include <LabJackM.h>
// #include "LJM_StreamUtilities.h"
#include "LJM_Utilities.h"

// using std::placeholders::_1;
using namespace std::chrono_literals;

class LabJack_DIO_Reader : public rclcpp::Node
{
public:
  LabJack_DIO_Reader() : Node("LabJack_DIO_Reader_Node")
  {
    this->declare_parameter<std::string>("IPAddress", "10.10.0.5");
    std::string IPAddress = this->get_parameter("IPAddress").as_string();
    this->declare_parameter<int>("sampleRateMS", 1000);
    int sampleRateMS = this->get_parameter("sampleRateMS").as_int();
    this->declare_parameter<std::string>("dinTopic", "/din");
    std::string dinTopic = this->get_parameter("dinTopic").as_string();

    din_pub = this->create_publisher<modaq_messages::msg::T8dio>(dinTopic, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(sampleRateMS), std::bind(&LabJack_DIO_Reader::timer_callback, this));

    handle = OpenOrDie(LJM_dtT8, LJM_ctETHERNET_UDP, IPAddress.c_str()); // opens T8 with specific IP Address
    PrintDeviceInfoFromHandle(handle);
    int IOStateMask = 0b00001111; // 1 = Output, 0 = Input 
    int err_Direction = LJM_eWriteName(handle,"FIO_DIRECTION",IOStateMask);

    int IOPullUp = 0b11111111;
    int err_Pullup = LJM_eWriteName(handle,"DIO_PULLUP_DISABLE",IOPullUp);

        int IOPullDown = 0b11110000;
    int err_PullDown = LJM_eWriteName(handle,"DIO_PULLDOWN_ENABLE",IOPullDown);

    if (err_Direction){
      std::cout << "Direction Error: " << err_Direction << std::endl;
    }
  }

  ~LabJack_DIO_Reader()
  {
    CloseOrDie(handle);
  }

private:
  void timer_callback()
  {
    time = std::chrono::system_clock::now();
    timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
    // err = LJM_eReadNames(handle, 1, aNames, aValues, &errorAddress);
    err = LJM_eReadName(handle, aNames[0], &value);
    if (err != LJME_NOERROR)
    {
      // Deal with error
      std::cout << "DIO State node Read Error" << std::endl;
    }

    int out = int(value);
    msg.fio0 = out % 2;
    msg.fio1 = (out >>= 1) % 2;
    msg.fio2 = (out >>= 1) % 2;
    msg.fio3 = (out >>= 1) % 2;
    msg.fio4 = (out >>= 1) % 2;

    //std::cout << "State:" << int(value) << " " << msg.fio0 << msg.fio1 << msg.fio2 << msg.fio3 <<  msg.fio4 << std::endl;
    // rclcpp::Time ts = this->now();
    msg.header.set__stamp(now());
    din_pub->publish(msg);
  }

  int err, errorAddress, handle;
  // const char *aNames[4] = {"FIO0", "FIO1", "FIO2", "FIO3"}; // channel names to write per LJM
  const char *aNames[1] = {"FIO_STATE"}; // channel names to write per LJM
  // double aValues[1];
  double value = 0;

  uint64_t timestamp_ns;
  modaq_messages::msg::T8dio msg;

  rclcpp::Publisher<modaq_messages::msg::T8dio>::SharedPtr din_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::time_point<std::chrono::system_clock> time;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LabJack_DIO_Reader>());
  rclcpp::shutdown();
  return 0;
}
