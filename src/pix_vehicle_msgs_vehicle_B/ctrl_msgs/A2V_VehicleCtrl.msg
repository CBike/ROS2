std_msgs/Header header

# [] [0|1] 0:off 1:on
int8 position_light_control 0

# [] [0|1] 0:off 1:on
int8 low_light_control 0

# [] [0|1] 0:off 1:on
int8 left_turn_light control 0

# [] [0|1] 0:off 1:on
int8 right_turn_light control 0

# [] [0|1] 0:default 1:limit
int8 speed_limit_control 0

# [m/s] [1|20] Speed limit value, the maximum value does not exceed the speed limit value of the chassis software
float32 speed_limit 1.0

# [] [0|1] (reserved) 0:enable 1:disable
int8 check_mode_enable 0
