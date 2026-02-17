/**
 * @file labjack_dac_writer.cpp
 * @author Casey Nichols (casey.nichols@nlr.gov)
 * @brief ros node for writing an analog output value to a Labjack T8 device
 * @version 0.1
 * @date 2024-10-02
 * 
 * @copyright Copyright (c) 2024
*/
/**
  Purpose:
    1. Subscribe to LabJack t8dac Publisher
    2. Write DAC to device using a UDP connection

    Testing:
    Publish once from command line

    ros2 topic pub /T8dac modaq_messages/msg/T8dac "{dacvoltage: 1.0}" -1


*/


#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "modaq_messages/msg/t8dac.hpp"

#include <LabJackM.h>
#include "LJM_Utilities.h"


using std::placeholders::_1;

class LabJack_DAC_Writer : public rclcpp::Node
{
public:
  LabJack_DAC_Writer() : Node("LabJack_DAC_Writer_Node")
  {
    this->declare_parameter<std::string>("IPAddress", "10.10.0.7");
    std::string IPAddress = this->get_parameter("IPAddress").as_string();
    this->declare_parameter<std::string>("dacTopic", "/T8dac");
    std::string subTopic = this->get_parameter("dacTopic").as_string();
    this->declare_parameter<std::string>("analogOutTag", "TDAC0");
    aNames = this->get_parameter("analogOutTag").as_string();  

    dac_sub_ = this->create_subscription<modaq_messages::msg::T8dac>(subTopic, 10, std::bind(&LabJack_DAC_Writer::action_callback, this, _1));

    handle = OpenOrDie(LJM_dtT8, LJM_ctETHERNET_UDP, IPAddress.c_str()); // opens T8 with specific IP Address
    PrintDeviceInfoFromHandle(handle);

    //Set all Outputs to Low to start
    err = LJM_eWriteName(handle, aNames.c_str(), 0.0);
  }

  ~LabJack_DAC_Writer()
  {
    err = LJM_eWriteName(handle, aNames.c_str(), 0.0);
    CloseOrDie(handle);
  }

private:
  void action_callback(const modaq_messages::msg::T8dac &msg)
  {

    // LJM seems to use C style arrays and 
    aValues  = msg.dacvoltage;



    err = LJM_eWriteName(handle, aNames.c_str(), aValues);

    char errstring [1024];

    if (err != LJME_NOERROR)
    {
      LJM_ErrorToString(err, errstring);
      std::cout << "ERROR: " <<  errstring << std::endl;
    }
  }

  int err, errorAddress, handle;
  std::string aNames; // channel name to write per LJM
  float aValues = 0;
  rclcpp::Subscription<modaq_messages::msg::T8dac>::SharedPtr dac_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LabJack_DAC_Writer>());
  rclcpp::shutdown();
  return 0;
}
