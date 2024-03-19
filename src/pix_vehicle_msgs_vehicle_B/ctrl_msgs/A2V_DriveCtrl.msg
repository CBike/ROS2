std_msgs/Header header

# [] [0|1] 0:disable 1:enable
int8 vehicle_drive_control_enable 0

# [] [0|3] 0:speed ctrl mode 1:throttle ctrl mode 2:reserve1 3:reserve2
int8 drive_mode_control 0

# [] [0|3] 0:default N 1:D 2:N 3:R
int8 gear_control 0

# [m/s] [0|50] It is valid in speed mode, and the chassis is controlled by sending speed commands
int8 vehicle_speed_control 0

# [%] [0|100] It is valid in torque mode, and the chassis is controlled by sending the throttle opening value
int8 vehicle_throttle_control 0

# [] [0|15] Add 1 for each cycle, 0~15 cycles
int8 cycle_count 0

# [] [] sum=byte0 xor byte1 xor...byte6
int8 checksum 0