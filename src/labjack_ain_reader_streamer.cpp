/**
 * @file labjack_ain_reader_streamer.cpp
 * @author Casey Nichols (casey.nichols@nlr.gov)
 * @brief ros node for streaming analog data from a Labjack T8 device
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <LabJackM.h>
#include "LJM_StreamUtilities.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <pthread.h>
#include <chrono>

#include "modaq_messages/msg/t8ainstream.hpp"
#include "modaq_messages/msg/systemmsg.hpp"

using namespace std::chrono_literals;

class LabJackAINStream : public rclcpp::Node
{
public:
  LabJackAINStream()
      : Node("labjack_ain_reader")
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("IPAddress", "10.10.0.6");
    IPAddress = this->get_parameter("IPAddress").as_string();
    this->declare_parameter<double>("SampleRate", 10000);
    INIT_SCAN_RATE = this->get_parameter("SampleRate").as_double();
    this->declare_parameter<int>("ScansPerRead", 100);
    SCANS_PER_READ = this->get_parameter("ScansPerRead").as_int();
    this->declare_parameter<std::string>("ainTopic", "/labjack_ain_blank");
    std::string ainTopic = this->get_parameter("ainTopic").as_string();

    this->declare_parameter<std::vector<double>>("channelScales", {1, 1, 1, 1, 1, 1, 1, 1});
    channelScales = this->get_parameter("channelScales").as_double_array();
    this->declare_parameter<std::vector<double>>("channelOffsets", {0, 0, 0, 0, 0, 0, 0, 0});
    channelOffsets = this->get_parameter("channelOffsets").as_double_array();

    // Initialize publisher and timer
    rclcpp::QoS qos_profile(10000); // Keep last 1000 messages

    // Set reliability and durability policies
    qos_profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);
    publisher_ = this->create_publisher<modaq_messages::msg::T8ainstream>(ainTopic, qos_profile);
    publisher_sysmsg = this->create_publisher<modaq_messages::msg::Systemmsg>("/system_messenger", 10);

    // Allocate memory for data
    aDataSize = NUM_CHANNELS * SCANS_PER_READ;
    aData = new double[aDataSize];
    waveformPeriod = (1 / INIT_SCAN_RATE) * SCANS_PER_READ;

    // Open LabJack device
    handle = OpenOrDie(LJM_dtT8, LJM_ctETHERNET_TCP, IPAddress.c_str());
    LJM_eStreamStop(handle);
    PrintDeviceInfoFromHandle(handle);
    HardcodedConfigureStream(handle);
    Stream(handle);

    // Log and display configuration details
    std::cout << "IPAddress: " << IPAddress << "\n"
              << "Sample Rate: " << INIT_SCAN_RATE << "\n"
              << "Scans Per Read: " << SCANS_PER_READ << "\n"
              << "Data Size: " << aDataSize << "\n";

    // Send system start message
    modaq_messages::msg::Systemmsg initmsg;
    initmsg.email_option = 0;
    initmsg.header.stamp = now();
    initmsg.message_tag = "Labjack Reader Started";
    initmsg.log_enable = true;
    publisher_sysmsg->publish(initmsg);

    set_thread_priority();
  }

  ~LabJackAINStream()
  {

    // Free allocated memory
    delete[] aData;
    free(aScanList);
    std::cout << "Deconstructed!!" << std::endl;
  }
  void stream_reader()
  {
    while (rclcpp::ok())
    {
      rclcpp::spin_some(this->get_node_base_interface());

      // Read data from LabJack
      err = LJM_eStreamRead(handle, aData, &deviceScanBacklog, &LJMScanBacklog);
      auto now = std::chrono::system_clock::now();
      uint64_t streamtN = std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();
      uint64_t streamt0 = streamtN - (waveformPeriod * 1000000000LL);
      uint64_t sec = streamt0 / 1'000'000'000ULL;
      uint64_t nsec = streamt0 % 1'000'000'000ULL;
      rclcpp::Time ros_time(sec, nsec, RCL_SYSTEM_TIME);

      // Set t0 timestamp in the message
      msg.header.set__stamp(ros_time);
      ErrorCheck(err, "LJM_eStreamRead");

      int ii = 0;

      // Process data from each channel and store in vectors
      for (uint channel = 0; channel <= aDataSize - NUM_CHANNELS; channel += NUM_CHANNELS)
      {
        // Scales and offsets applied
        data_vector_ain0.push_back(aData[channel] * channelScales[0] + channelOffsets[0]);
        data_vector_ain1.push_back(aData[channel + 1] * channelScales[1] + channelOffsets[1]);
        data_vector_ain2.push_back(aData[channel + 2] * channelScales[2] + channelOffsets[2]);
        data_vector_ain3.push_back(aData[channel + 3] * channelScales[3] + channelOffsets[3]);
        data_vector_ain4.push_back(aData[channel + 4] * channelScales[4] + channelOffsets[4]);
        data_vector_ain5.push_back(aData[channel + 5] * channelScales[5] + channelOffsets[5]);
        data_vector_ain6.push_back(aData[channel + 6] * channelScales[6] + channelOffsets[6]);
        data_vector_ain7.push_back(aData[channel + 7] * channelScales[7] + channelOffsets[7]);

        core_timer_read = aData[channel + 8] + 65536 * aData[channel + 9];
        data_vector_core_timer.push_back(core_timer_read);

        if (last_core_timer != 0)
        {
          int64_t wrap_delta = (int64_t)core_timer_read - (int64_t)last_core_timer;
          if (wrap_delta < -1000000) // add 2^32 each time core_timer rolls over
          {
            core_timer_rollover += UINT32_MAX;
          }

          extended_core_timer = core_timer_read + core_timer_rollover;
        }
        last_core_timer = core_timer_read;
        data_vector_system_time.push_back((extended_core_timer * 10) + core_timer_2_utc_offset);

        data_vector_recv_time_calc.push_back(streamt0 + (ii / INIT_SCAN_RATE) * 1e9);
        ii++;
        // std::cout << "core_timer_read: " << core_timer_read << " core_timer_2_utc_offset: " << core_timer_2_utc_offset << " extended_core_timer: " << extended_core_timer << " system_time: " << (extended_core_timer * 10) + core_timer_2_utc_offset << std::endl;
      }

      // Populate message fields
      msg.set__ain0(data_vector_ain0);
      msg.set__ain1(data_vector_ain1);
      msg.set__ain2(data_vector_ain2);
      msg.set__ain3(data_vector_ain3);
      msg.set__ain4(data_vector_ain4);
      msg.set__ain5(data_vector_ain5);
      msg.set__ain6(data_vector_ain6);
      msg.set__ain7(data_vector_ain7);
      msg.set__core_timer(data_vector_core_timer);
      msg.set__system_time(data_vector_system_time);
      msg.set__recv_time_calc(data_vector_recv_time_calc);
      msg.set__ns_recv_time(streamtN);
      

      // Clear vectors for next read
      data_vector_ain0.clear();
      data_vector_ain1.clear();
      data_vector_ain2.clear();
      data_vector_ain3.clear();
      data_vector_ain4.clear();
      data_vector_ain5.clear();
      data_vector_ain6.clear();
      data_vector_ain7.clear();
      data_vector_core_timer.clear();
      data_vector_system_time.clear();
      data_vector_recv_time_calc.clear();

      // Count and report skipped scans
      numSkippedScans = CountAndOutputNumSkippedScans(NUM_CHANNELS, SCANS_PER_READ, aData);
      if (numSkippedScans)
      {
        {
          totalSkippedScans += numSkippedScans;
          RCLCPP_INFO(this->get_logger(), "LJMBacklog: %d; deviceBacklog: %d; Total skipped: %d, error: %d", LJMScanBacklog, deviceScanBacklog, totalSkippedScans, err);
          for (int j = 0; j < NUM_CHANNELS * SCANS_PER_READ; j++)
          {
            // std::cout << "aData" << j << ": " << aData[j] << std::endl;
          }
        }
      }

      // Publish the message
      publisher_->publish(msg);
    }
    // Stop the stream and close the device
    printf("Stopping stream\n");
    err = LJM_eStreamStop(handle);
    ErrorCheck(err, "Stopping stream");
    CloseOrDie(handle);
  }

private:
  void HardcodedConfigureStream(int handle)
  {
    const int STREAM_TRIGGER_INDEX = 0;
    const int STREAM_CLOCK_SOURCE = 0;
    const int STREAM_RESOLUTION_INDEX = 0;
    const double STREAM_SETTLING_US = 0;
    const double AIN_ALL_RANGE = 0;
    const int AIN_ALL_NEGATIVE_CH = LJM_GND;

    printf("Writing configurations:\n");

    // Configure stream trigger index
    printf("    Setting STREAM_TRIGGER_INDEX to %d\n", STREAM_TRIGGER_INDEX);
    WriteNameOrDie(handle, "STREAM_TRIGGER_INDEX", STREAM_TRIGGER_INDEX);

    // Configure stream clock source
    printf("    Setting STREAM_CLOCK_SOURCE to %d\n", STREAM_CLOCK_SOURCE);
    WriteNameOrDie(handle, "STREAM_CLOCK_SOURCE", STREAM_CLOCK_SOURCE);

    // Configure analog input settings
    printf("    Setting STREAM_RESOLUTION_INDEX to %d\n", STREAM_RESOLUTION_INDEX);
    WriteNameOrDie(handle, "STREAM_RESOLUTION_INDEX", STREAM_RESOLUTION_INDEX);

    printf("    Setting STREAM_SETTLING_US to %f\n", STREAM_SETTLING_US);
    WriteNameOrDie(handle, "STREAM_SETTLING_US", STREAM_SETTLING_US);

    printf("    Setting AIN_ALL_RANGE to %f\n", AIN_ALL_RANGE);
    WriteNameOrDie(handle, "AIN_ALL_RANGE", AIN_ALL_RANGE);

    printf("    Setting AIN_ALL_NEGATIVE_CH to %s\n", AIN_ALL_NEGATIVE_CH == LJM_GND ? "LJM_GND" : std::to_string(AIN_ALL_NEGATIVE_CH).c_str());
    WriteNameOrDie(handle, "AIN_ALL_NEGATIVE_CH", AIN_ALL_NEGATIVE_CH);

    std::cout << "Finished configuration" << std::endl;
  }

  void Stream(int handle)
  {
    std::cout << "Starting Stream()" << std::endl;

    // Get handle info
    err = LJM_GetHandleInfo(handle, NULL, &connectionType, NULL, NULL, NULL, NULL);
    ErrorCheck(err, "LJM_GetHandleInfo");

    // Get channel addresses
    err = LJM_NamesToAddresses(NUM_CHANNELS, channelNames, aScanList, NULL);
    ErrorCheck(err, "Getting positive channel addresses");

    err = LJM_eReadAddress(handle, 61520, LJM_UINT32, &value);
    auto core_timer_start_value = uint64_t(value);
    // get system clock value at this moment
    auto sysClock = std::chrono::high_resolution_clock::now().time_since_epoch();
    acq_start_time = std::chrono::duration_cast<std::chrono::nanoseconds>(sysClock).count(); // Get system time in ns
    printf("acq_start_time: %lld ns\n", acq_start_time);
    auto rostime = this->now().nanoseconds();

    std::cout << "ros: " << rostime << std::endl;
    std::cout << "labjack core_timer at start: " << core_timer_start_value << std::endl;

    core_timer_2_utc_offset = acq_start_time - (core_timer_start_value * 10);
    std::cout << "core_timer 2 UTC offset: " << core_timer_2_utc_offset << std::endl;

    printf("Stream started. Actual scan rate: %.02f Hz (%.02f sample rate)\n", INIT_SCAN_RATE, INIT_SCAN_RATE * NUM_CHANNELS);

    modaq_messages::msg::Systemmsg streammsg;
    streammsg.email_option = 0;
    streammsg.header.stamp = now();
    streammsg.header.frame_id = "labjack_ain_reader_streamer";
    streammsg.message_tag = "Labjack stream started";

    // Include acq_start_time and core_timer_start_value in the message body
    std::stringstream message_body;
    message_body << "Acquisition start time: " << acq_start_time << " ns, ";
    message_body << "Core timer start value: " << core_timer_start_value;
    streammsg.message_body = message_body.str();

    streammsg.log_enable = true;
    publisher_sysmsg->publish(streammsg);

    // Start stream
    printf("\nStarting stream...\n");
    err = LJM_eStreamStart(handle, SCANS_PER_READ, NUM_CHANNELS, aScanList, &INIT_SCAN_RATE);
    ErrorCheck(err, "LJM_eStreamStart");
    printf("Stream started. Actual scan rate: %.02f Hz (%.02f sample rate)\n", INIT_SCAN_RATE, INIT_SCAN_RATE * NUM_CHANNELS);
    printf("\n");
  }

  void set_thread_priority()
  {
    struct sched_param param;
    param.sched_priority = 49; // High priority
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param))
    {
      perror("pthread_setschedparam");
    }
    RCLCPP_INFO(this->get_logger(), "setting thread priority to: %d", param.sched_priority);
  }

  rclcpp::Publisher<modaq_messages::msg::T8ainstream>::SharedPtr publisher_;
  rclcpp::Publisher<modaq_messages::msg::Systemmsg>::SharedPtr publisher_sysmsg;
  rclcpp::TimerBase::SharedPtr timer_;
  modaq_messages::msg::T8ainstream msg;

  int handle;
  std::string IPAddress;
  double INIT_SCAN_RATE = 100;
  int SCANS_PER_READ = 10;
  enum
  {
    NUM_CHANNELS = 10
  };
  int err, channel;
  int last_time = 0;
  int numSkippedScans = 0;
  int totalSkippedScans = 0;
  int deviceScanBacklog = 0;
  int LJMScanBacklog = 0;
  unsigned int receiveBufferBytesSize = 0;
  unsigned int receiveBufferBytesBacklog = 0;
  int connectionType;
  const char *channelNames[NUM_CHANNELS] = {"AIN0", "AIN1", "AIN2", "AIN3", "AIN4", "AIN5", "AIN6", "AIN7", "CORE_TIMER", "STREAM_DATA_CAPTURE_16"}; // need an extra 16 bit register for the core timer!
  int *aScanList = (int *)malloc(sizeof(int) * NUM_CHANNELS);
  unsigned int aDataSize;
  double *aData, value;
  std::vector<double> data_vector_ain0;
  std::vector<double> data_vector_ain1;
  std::vector<double> data_vector_ain2;
  std::vector<double> data_vector_ain3;
  std::vector<double> data_vector_ain4;
  std::vector<double> data_vector_ain5;
  std::vector<double> data_vector_ain6;
  std::vector<double> data_vector_ain7;
  std::vector<uint64_t> data_vector_core_timer;
  std::vector<uint64_t> data_vector_system_time;
  std::vector<uint64_t> data_vector_recv_time_calc;

  std::vector<double> channelScales;
  std::vector<double> channelOffsets;
  rclcpp::Time streamtN;
  double waveformPeriod;
  rclcpp::Time streamt0;
  unsigned long core_timer_read, last_core_timer = 0;
  unsigned long long int acq_start_time = 0, core_timer_2_utc_offset = 0, extended_core_timer = 0, core_timer_rollover = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LabJackAINStream>();
  node->stream_reader();
  rclcpp::shutdown();
  return 0;
}