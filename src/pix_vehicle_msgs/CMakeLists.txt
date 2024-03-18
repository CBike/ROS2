cmake_minimum_required(VERSION 3.8)
project(pix_vehicle_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Report msg
  "report_msgs/BmsReport.msg"
  "report_msgs/BrakeReport.msg"
  "report_msgs/GearReport.msg"
  "report_msgs/ParkReport.msg"
  "report_msgs/SteerReport.msg"
  "report_msgs/ThrottleReport.msg"
  "report_msgs/VcuReport.msg"
  "report_msgs/WheelSpeedReport.msg"
  # Command msg
  "command_msgs/ThrottleCommand.msg"
  "command_msgs/BrakeCommand.msg"
  "command_msgs/GearCommand.msg"
  "command_msgs/ParkCommand.msg"
  "command_msgs/SteeringCommand.msg"
  "command_msgs/VehicleModeCommand.msg"

  DEPENDENCIES std_msgs
 )

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

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()
