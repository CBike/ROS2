std_msgs/Header header

# [] [0|1] 0:disable 1:enable
int8 vehicle_brake_control_enable 0

# [] [0|1] 0:off（Relationship with whether the VCU controls the brake light is yes or no） 1:on
int8 vehicle_brake_light_control 0

# [%] [0|100] Brake control 0~100%, representing 0~maximum brake pressure control
float32 vehicle_brake_control 0.0

# [] [0|2] 0:default 1:brake 2:release
int8 parking_control 0

# [] [0|15] Add 1 for each cycle, 0~15 cycles
int8 cycle_count 0

# [] [] byte0 xor byte1 xor...byte6
int32 checksum 0
