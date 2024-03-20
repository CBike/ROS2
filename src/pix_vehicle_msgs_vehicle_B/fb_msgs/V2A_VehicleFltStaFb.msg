std_msgs/Header header

# [] [0|1] 0:normal 1:over temp
int8 motor_system_overheating 0

# [] [0|1] 0:normal 1:over temp
int8 battery_system_overheating 0

# [] [0|1] 0:normal 1:over temp
int8 brake_system_overheating 0

# [] [0|1] 0:normal 1:over temp
int8 steering_system_overheating 0

# [] [0|1] 0:normal 1:under volt
int8 battery_voltage_is_too_low 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 system_error 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 brake_system_failure 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 parking_system_failure 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 front_steering_system_fault 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 rear_steering_system_failure 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 left_front_motor_system_failure 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 right_front_motor_system_failure 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 left_rear_motor_system_failure 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 right_rear_motor_system_failure 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 bms_system_failure 0

# [] [0|4] 0:normal 1:fault level 1 2:fault level 2 3:fault level 3 4:fault level 4
int8 dc_system_failure 0
