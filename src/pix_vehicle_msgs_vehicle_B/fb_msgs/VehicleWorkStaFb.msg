std_msgs/Header header

# [] [0|3] 0:standby 1:self driving 2:remote 3:man
int8 drive_mode_fb 0

# [] [0|3] 0:init 1:on acc 2:ready 3:off
int8 vehicle_power_on_status_fb 0

# [] [0|2] 0:off 1:on 2:standby
int8 dc_working_status 0

# [] [0|1] 0:default 1:limit
int8 vehicle_speed_limit_status 0

# [m/s] [0|50] Current speed limit value feedback, which can be set by autonomous driving in autonomous driving mode.
float32 vehicle_speed_limit_val_fb 0.0

# [V] [0|25] When the voltage is too low, you need to drive carefully
float32 low_voltage_battery_voltage 0.0

# [] [0|3] 0:no 1:chassis estop 2:remote estop 3:chassis err estop
int8 emergency_stop_status_fb 0

# [] [0|1] 0:off 1:collide
int8 vehicle_power_battery 0

# [] [0|1] 0:off 1:collide
int8 vehicle_rear_crash_sensor_feedback

# [] [0|15]
int8 vcu_cycle_count 0

# [] [] byte0 xor byte1 xor...byte6
int32 vcu_checksum 0