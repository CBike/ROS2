std_msgs/Header header

# [] [0|1] 0:disable 1:enable
int8 brake_en_state 0

# [] [0|1] 0:off 0:on
int8 brake_light_en_state 0

# [] [0|3] 0:release 1:brake 2:releasing 3:braking
int8 parking_state 0

# [%] [0|100] The brake pedal value feedback is 0~100%
float32 brake_pedal_val_fb 0.0

# [bar] [0|100] Actual brake pressure feedback
int8 brake_pressure_fb 0

# [] [0|15] VCU cycle count
int8 vcu_cycle_count 0
