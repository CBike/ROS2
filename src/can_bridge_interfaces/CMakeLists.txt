cmake_minimum_required(VERSION 3.8)
project(can_bridge_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsReport.msg"
  "msg/BrakeReport.msg"
  "msg/GearReport.msg"
  "msg/ParkReport.msg"
  "msg/SteerReport.msg"
  "msg/ThrottleReport.msg"
  "msg/VcuReport.msg"
  "msg/WheelSpeedReport.msg"
  "msg/ThrottleCommand.msg"
  "msg/BrakeCommand.msg"
  "msg/GearCommand.msg"
  "msg/SteerCommand.msg"
  "msg/VehicleModeCommand.msg"

  DEPENDENCIES std_msgs
 )

ament_export_dependencies(rosidl_default_runtime)
ament_package()

