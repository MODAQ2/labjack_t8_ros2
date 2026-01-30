/**
 * @file labjack_reader_streamer.cpp
 * @author Casey Nichols (casey.nichols@nlr.gov)
 * @brief ros node for reading single analog datapoints from a Labjack T8 device
 * Purpose:
 *   1. Read the state of the DIO on the T8
 *   2. Publish those states
 * @version 0.1
 * @date 2024-10-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */


// #include <functional>
// #include <memory>
#include <chrono>
#include <stdio.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "modaq_messages/msg/t8ain.hpp"

#include <LabJackM.h>
// #include "LJM_StreamUtilities.h"
#include "LJM_Utilities.h"

// using std::placeholders::_1;
using namespace std::chrono_literals;

class LabJack_AIN_Reader : public rclcpp::Node
{
public:
  LabJack_AIN_Reader() : Node("LabJack_AIN_Reader_Node")
  {
    this->declare_parameter<std::string>("IPAddress", "10.10.0.7");
    std::string IPAddress = this->get_parameter("IPAddress").as_string();
    this->declare_parameter<int>("sampleRateMS", 1000);
    int sampleRateMS = this->get_parameter("sampleRateMS").as_int();
    this->declare_parameter<std::string>("ainTopic", "/labjack_ain_blank");
    std::string ainTopic = this->get_parameter("ainTopic").as_string();

    ain_pub_ = this->create_publisher<modaq_messages::msg::T8ain>(ainTopic, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(sampleRateMS), std::bind(&LabJack_AIN_Reader::timer_callback, this));

    handle = OpenOrDie(LJM_dtT8, LJM_ctETHERNET_UDP, IPAddress.c_str()); // opens T8 with specific IP Address
    PrintDeviceInfoFromHandle(handle);
  }

  ~LabJack_AIN_Reader()
  {
    CloseOrDie(handle);
  }

private:
  void timer_callback()
  {
    time = std::chrono::system_clock::now();
    timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
    // err = LJM_eReadNames(handle, 1, aNames, aValues, &errorAddress);
    err = LJM_eReadNames(handle, 9, channelNames, aData, &errorAddress);
    if (err != LJME_NOERROR)
    {
      char errorName[LJM_MAX_NAME_SIZE]; // LJM_MAX_NAME_SIZE is 256
      LJM_ErrorToString(err, errorName);
      printf("%s \n", errorName);
      std::cout << "ERROR Line 54:" << err << std::endl;
    }

    int out = int(value);
    msg.ain0 = aData[0];
    msg.ain1 = aData[1];
    msg.ain2 = aData[2];
    msg.ain3 = aData[3];
    msg.ain4 = aData[4];
    msg.ain5 = aData[5];
    msg.ain6 = aData[6];
    msg.ain7 = aData[7];
    msg.core_timer = aData[8];

    // std::cout << "State:" << int(value) << " " << msg.fio0 << msg.fio1 << msg.fio2 << msg.fio3 <<  msg.fio4 << std::endl;
    //  rclcpp::Time ts = this->now();
    msg.header.set__stamp(now());
    ain_pub_->publish(msg);
  }

  int err, errorAddress, handle;
  const char *channelNames[9] = {"AIN0", "AIN1", "AIN2", "AIN3", "AIN4", "AIN5", "AIN6", "AIN7", "CORE_TIMER"}; // need an extra 16 bit register for the core timer!
  double aData[9] = {0};                                                                                                                  // Initialize array with 10 elements
  double value = 0;

  uint64_t timestamp_ns;
  modaq_messages::msg::T8ain msg;

  rclcpp::Publisher<modaq_messages::msg::T8ain>::SharedPtr ain_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::time_point<std::chrono::system_clock> time;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LabJack_AIN_Reader>());
  rclcpp::shutdown();
  return 0;
}
