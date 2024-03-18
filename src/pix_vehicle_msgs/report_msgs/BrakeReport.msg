std_msgs/Header header
# [%] [0|100]
float32 brake_pedal_actual 0.0
# Brake system communication fault [] [0|1] 0-no_fault 1-brake_system_communication_fault
int8 brake_flt2 0
# Brake system hardware fault [] [0|1] 0-no_fault 1-brake_hardware_fault
int8 brake_flt1 0
# [] [0|3] 0-manual 1-auto 2-takeover 3-standby
int8 brake_en_state 0
