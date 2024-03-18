std_msgs/Header header
# [deg/s] [0|255]
int32 steer_angle_spd_actual 0
# Steer system communication fault [] [0|255] 0-no_fault 1-steer_system_communication_fault
int8 steer_flt2 0
# Steer system hardware fault [] [0|255] 0-no_fault 1-steer_system_hardware_fault
int8 steer_flt1 0
# [] [0|3] 0-Manual 1-Auto 2-Takeover 3-Standby
int8 steer_en_state 0
# [deg] [-360|360]
int32 steer_angle_actual 0
