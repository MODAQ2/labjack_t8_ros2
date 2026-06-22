cmake_minimum_required(VERSION 3.8)
project(labjack_t8_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(modaq_messages REQUIRED)



add_executable(labjack_ain_streamer src/labjack_ain_reader_streamer.cpp)
add_executable(labjack_ain_reader src/labjack_ain_reader.cpp)
add_executable(labjack_reader_test src/labjack_reader_test.cpp)
add_executable(labjack_do_node src/labjack_do_node.cpp)
add_executable(labjack_hmi_control src/labjack_hmi_control_node.cpp)
add_executable(labjack_dio_reader src/labjack_dio_state_node.cpp)
add_executable(labjack_dac_writer src/labjack_dac_writer.cpp)
add_executable(time_synchronziation_node src/time_synchronziation_node.cpp)

include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/include/labjack_t8_ros)
ament_target_dependencies(labjack_ain_streamer rclcpp std_msgs modaq_messages)
ament_target_dependencies(labjack_ain_reader rclcpp std_msgs modaq_messages)

ament_target_dependencies(labjack_reader_test rclcpp modaq_messages)
ament_target_dependencies(labjack_do_node rclcpp std_msgs modaq_messages)
ament_target_dependencies(labjack_hmi_control rclcpp std_msgs modaq_messages)
ament_target_dependencies(labjack_dio_reader rclcpp std_msgs modaq_messages)
ament_target_dependencies(labjack_dac_writer rclcpp std_msgs modaq_messages)
ament_target_dependencies(time_synchronziation_node rclcpp std_msgs modaq_messages)

target_link_libraries(labjack_ain_streamer /usr/local/lib/libLabJackM.so)
target_link_libraries(labjack_ain_reader /usr/local/lib/libLabJackM.so)
target_link_libraries(labjack_reader_test "${cpp_typesupport_target}")
target_link_libraries(labjack_do_node /usr/local/lib/libLabJackM.so)
target_link_libraries(labjack_dio_reader /usr/local/lib/libLabJackM.so)
target_link_libraries(labjack_dio_reader "${cpp_typesupport_target}")
target_link_libraries(labjack_dac_writer /usr/local/lib/libLabJackM.so)
target_link_libraries(labjack_dac_writer "${cpp_typesupport_target}")
target_link_libraries(time_synchronziation_node /usr/local/lib/libLabJackM.so.1.23.4)
target_link_libraries(time_synchronziation_node "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)
install(TARGETS
  #lj_t8
  labjack_ain_reader
  labjack_ain_streamer
  labjack_reader_test
  labjack_do_node
  labjack_hmi_control
  labjack_dio_reader
  labjack_dac_writer
  time_synchronziation_node
  #stream_callback  
  #simple_t8
  DESTINATION lib/${PROJECT_NAME})# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
