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
  # fb_msgs
  "fb_msgs/V2A_BrakeStaFb.msg"
  "fb_msgs/V2A_ChassisWheelRpmFb.msg"
  "fb_msgs/V2A_ChassisWheelTorqueFb.msg"
  "fb_msgs/V2A_DriveStaFb.msg"
  "fb_msgs/V2A_PowerStaFb.msg"
  "fb_msgs/V2A_SteerStaFb.msg"
  "fb_msgs/V2A_VehicleFltStaFb.msg"
  "fb_msgs/V2A_VehicleStaFb.msg"
  "fb_msgs/V2A_VehicleWorkStaFb.msg"
  # Command msg
  "ctrl_msgs/A2V_BrakeCtrl.msg"
  "ctrl_msgs/A2V_DriveCtrl.msg"
  "ctrl_msgs/A2V_SteerCtrl.msg"
  "ctrl_msgs/A2V_VehicleCtrl.msg"
  "ctrl_msgs/A2V_WheelTorqueCtrl.msg"

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
