/**
 * @file labjack_hmi_control_node.cpp
 * @author Robert Raye (robert.raye@nlr.gov)
 * @brief ros node for controling digital output with the HMI to a Labjack T8 device
 * @version 0.1
 * @date 2024-10-02
 * 
 * @copyright Copyright (c) 2024
*/
/**
Purpose:
  1. Subscribe to control publishers, including from HMI
  2. Execute logic prioritizing and/or processing control directives
  3. Publish actions for write to output device
This middleware and only processes data heard on the subscriber(s) and publishes results
for action by other nodes- in particular the labjack_do_node.

  Testing:
    Publish once from command line

    ros2 topic pub /hmi_ctl modaq_messages/msg/Hmi "{do0: 1}" -1
    ros2 topic pub /hmi_ctl modaq_messages/msg/Hmi "{do0: 0, do1: 1}" -1
    ros2 topic pub /hmi_ctl modaq_messages/msg/Hmi "{do0: 0, do1: 1, do2: 1, do3: 0}" -1
*/

#include <functional>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "modaq_messages/msg/hmi.hpp"
#include "modaq_messages/msg/lj_do.hpp"

using std::placeholders::_1;

class LabJack_Control : public rclcpp::Node
{
public:
  LabJack_Control() : Node("LabJack_Control_Node")
  {
    // subscribe to the HMI publisher topic
    hmi_sub_ = this->create_subscription<modaq_messages::msg::Hmi>("hmi_ctl", 10, std::bind(&LabJack_Control::action_callback, this, _1));

    // Create a publisher for digital output state topic
    LJ_Ctl_Pub_ = this->create_publisher<modaq_messages::msg::LjDo>("LJ_Ctl_Pub", 10);
  }

private:
  void action_callback(const modaq_messages::msg::Hmi &msg)
  {
    time = std::chrono::system_clock::now();
    timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
    /*
      Do the logic here

      Right now, this is just copying the message received to the publish object
    */

    if (msg.override)
    {
      ctl_msg.set__fio0(msg.hmi_cmd_0);
      ctl_msg.set__fio1(msg.hmi_cmd_1);
      ctl_msg.set__fio2(msg.hmi_cmd_2);
      ctl_msg.set__fio3(msg.hmi_cmd_3);
      ctl_msg.header.set__stamp(now());

      RCLCPP_INFO(this->get_logger(), "I heard: %d %d %d %d", msg.hmi_cmd_0, msg.hmi_cmd_1, msg.hmi_cmd_2, msg.hmi_cmd_3);
      RCLCPP_INFO(this->get_logger(), "Publishing: %d %d %d %d", ctl_msg.fio0, ctl_msg.fio1, ctl_msg.fio2, ctl_msg.fio3);

      LJ_Ctl_Pub_->publish(ctl_msg);
    } else {
      RCLCPP_INFO(this->get_logger(), "HMI Override Not Sent, Ignoring Solenoid Change Request");
    }
  }

  std::chrono::time_point<std::chrono::system_clock> time;
  uint64_t timestamp_ns;

  // std::unique_ptr<modaq_messages::msg::LjDo> ctl_msg;
  modaq_messages::msg::LjDo ctl_msg; // SHOULD THIS BE A SHARED POINTER???
  rclcpp::Subscription<modaq_messages::msg::Hmi>::SharedPtr hmi_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<modaq_messages::msg::LjDo>::SharedPtr LJ_Ctl_Pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LabJack_Control>());
  rclcpp::shutdown();
  return 0;
}
