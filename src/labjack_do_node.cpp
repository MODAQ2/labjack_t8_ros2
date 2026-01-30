/**
 * @file labjack_do_node.cpp
 * @author Robert Raye (robert.raye@nlr.gov)
 * @brief ros node for writing digital output values from a Labjack T8 device
 * @version 0.1
 * @date 2024-10-02
 * 
 * @copyright Copyright (c) 2024
*/
/**
  Purpose:
    1. Subscribe to LabJack Control Publisher
    2. Write DO to device using a UDP connection

    Testing:
    Publish once from command line

    ros2 topic pub /LJ_Ctl_Pub modaq_messages/msg/LjDo "{fio0: 1}" -1
    ros2 topic pub /LJ_Ctl_Pub modaq_messages/msg/LjDo "{fio0: 0, fio1: 1}" -1
    ros2 topic pub /LJ_Ctl_Pub modaq_messages/msg/LjDo "{fio0: 0, fio1: 1, fio2: 1, fio3: 1}" -1

*/

//#include <functional>
//#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "modaq_messages/msg/t8dio.hpp"

#include <LabJackM.h>
#include "LJM_Utilities.h"

using std::placeholders::_1;

class LabJack_DO_Writer : public rclcpp::Node
{
public:
  LabJack_DO_Writer() : Node("LabJack_DO_Writer_Node")
  {
    this->declare_parameter<std::string>("IPAddress", "0.0.0.0");
    std::string IPAddress = this->get_parameter("IPAddress").as_string();
    this->declare_parameter<std::string>("doTopic", "/do");
    std::string subTopic = this->get_parameter("doTopic").as_string();

    hmi_sub_ = this->create_subscription<modaq_messages::msg::T8dio>(subTopic, 10, std::bind(&LabJack_DO_Writer::action_callback, this, _1));

    handle = OpenOrDie(LJM_dtT8, LJM_ctETHERNET_UDP, IPAddress.c_str()); // opens T8 with specific IP Address
    PrintDeviceInfoFromHandle(handle);

    //Set all Outputs to Low to start
    err = LJM_eWriteNames(handle, 4, aNames, aValues, &errorAddress);

    if (err != LJME_NOERROR)
    {
      // Deal with error
      std::cout << "DO Node LJM error 1" << std::endl;
      std::cout << err << std::endl;
    }
  }

  ~LabJack_DO_Writer()
  {
    CloseOrDie(handle);
  }

private:
  void action_callback(const modaq_messages::msg::T8dio &msg)
  {

    // LJM seems to use C style arrays and 
    aValues[0] = msg.fio0;
    aValues[1] = msg.fio1;
    aValues[2] = msg.fio2;
    aValues[3] = msg.fio3;

    RCLCPP_INFO(this->get_logger(), "I heard: %d %d %d %d", msg.fio0, msg.fio1, msg.fio2, msg.fio3);
    err = LJM_eWriteNames(handle, 4, aNames, aValues, &errorAddress);

    if (err != LJME_NOERROR)
    {
      // Deal with error
      std::cout << "DO Node LJM error 2" << std::endl;
      std::cout << err << std::endl;
    }
  }

  int err, errorAddress, handle;
  const char *aNames[4] = {"FIO0", "FIO1", "FIO2", "FIO3"}; // channel names to write per LJM
  double aValues[4] = {0,0,0,0};
  rclcpp::Subscription<modaq_messages::msg::T8dio>::SharedPtr hmi_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LabJack_DO_Writer>());
  rclcpp::shutdown();
  return 0;
}
