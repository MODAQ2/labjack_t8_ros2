/*
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
#include "modaq_messages/msg/ljtimesync.hpp"

#include <LabJackM.h>
#include "LJM_Utilities.h"
#include "LabJackMModbusMap.h"

// using std::placeholders::_1;
using namespace std::chrono_literals;

class LabJack_Timesync : public rclcpp::Node
{
public:
    LabJack_Timesync() : Node("LabJack_Timesync_Node")
    {
        this->declare_parameter<std::string>("IPAddress", "10.10.0.6");
        std::string IPAddress = this->get_parameter("IPAddress").as_string();
        this->declare_parameter<int>("sampleRateMS", 1000);
        int sampleRateMS = this->get_parameter("sampleRateMS").as_int();
        this->declare_parameter<std::string>("tsTopic", "/ljtimesync");
        std::string topicPubName = this->get_parameter("tsTopic").as_string();

        ljtimesync_pub_ = this->create_publisher<modaq_messages::msg::Ljtimesync>(topicPubName, 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(sampleRateMS), std::bind(&LabJack_Timesync::timer_callback, this));

        handle = OpenOrDie(LJM_dtT8, LJM_ctETHERNET_UDP, IPAddress.c_str()); // opens T8 with specific IP Address
        PrintDeviceInfoFromHandle(handle);

        
        err = LJM_eReadAddress(handle, LJM_CORE_TIMER_ADDRESS,LJM_CORE_TIMER_TYPE, &value);
        auto core_timer_start_value = uint64_t(value);
        // get system clock value at this moment
        auto sysClock = std::chrono::high_resolution_clock::now().time_since_epoch();
        acq_start_time = std::chrono::duration_cast<std::chrono::nanoseconds>(sysClock).count(); // Get system time in ns
        printf("acq_start_time: %lld ns\n", acq_start_time);
        core_timer_2_utc_offset = acq_start_time - (core_timer_start_value * 10);

    }

    ~LabJack_Timesync()
    {
        CloseOrDie(handle);
    }

private:
    void timer_callback()
    {
        time_before = std::chrono::system_clock::now();
        // err = LJM_eReadNames(handle, 1, aNames, aValues, &errorAddress);
        err = LJM_eReadAddress(handle, LJM_CORE_TIMER_ADDRESS,LJM_CORE_TIMER_TYPE, &value);
        time_after = std::chrono::system_clock::now();
        if (err != LJME_NOERROR)
        {
            // Deal with error
            RCLCPP_INFO(this->get_logger(), "LJ Time Sync: eReadName error");
        }
        core_timer_read = static_cast<uint64_t> (value);
        if (last_core_timer != 0)
        {
            int64_t wrap_delta = (int64_t)core_timer_read - (int64_t)last_core_timer;
            if (wrap_delta < -10) // add 2^32 each time core_timer rolls over
            {
                core_timer_rollover += UINT32_MAX;
            }

            extended_core_timer = core_timer_read + core_timer_rollover;
        }
        msg.system_time = core_timer_2_utc_offset + (extended_core_timer * 10);
        last_core_timer = core_timer_read;
        msg.core_timer = core_timer_read;
        msg.wrapped_core_timer = extended_core_timer;
        msg.unix_ns_before = std::chrono::duration_cast<std::chrono::nanoseconds>(time_before.time_since_epoch()).count();
        msg.unix_ns_after = std::chrono::duration_cast<std::chrono::nanoseconds>(time_after.time_since_epoch()).count();
        uint64_t mean_time = (msg.unix_ns_before + msg.unix_ns_after)/2;

        msg.header.stamp = rclcpp::Time(mean_time);

        ljtimesync_pub_->publish(msg);
    }
    int err, errorAddress, handle;
    const char *aNames[1] = {"CORE_TIMER"}; // channel names to write per LJM
    double value = 0;

    uint64_t timestamp_ns, core_timer_rollover = 0, core_timer_read, extended_core_timer, acq_start_time, core_timer_2_utc_offset, last_core_timer = 0;
    modaq_messages::msg::Ljtimesync msg;
    rclcpp::Time rosTime;

    rclcpp::Publisher<modaq_messages::msg::Ljtimesync>::SharedPtr ljtimesync_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::time_point<std::chrono::system_clock> time_before, time_after;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LabJack_Timesync>());
    rclcpp::shutdown();
    return 0;
}
