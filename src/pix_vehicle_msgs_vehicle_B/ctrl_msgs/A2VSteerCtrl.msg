std_msgs/Header header

# [] [0|1] 0:disable 1:enable
int8 vehicle_steering_control_enable 0

# [] [0|4] 0:front ackerman 1:same front and back 2:front different back 3:back ackerman 4:front back
int8 steering_mode_control 0

# [deg] [-500|500] Front Steering motor steering angle control
int32 vehicle_steering_control_front 0

# [deg] [-500|500] Rear Steering motor steering angle control
int32 vehicle_steering_control_rear 0

# [deg/s] [0|500]
int32 vehicle_steering_wheel_speed_control 0

# [] [0|15] Add 1 for each cycle, 0~15 cycles
int8 cycle_count 0

# [] [] byte0 xor byte1 xor...byte6
int8 checksum 0
