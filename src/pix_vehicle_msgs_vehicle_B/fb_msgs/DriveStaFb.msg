std_msgs/Header header

# [] [0|1] 0:disable 1:enable(Automated Driving Enabled Drive Control)
int8 drive_en_state 0

# [] [0|1] 0:normal 1:over slop(More than 5% is out of bounds)
int8 reminder_for_drive_control_out_of_bounds 0

# [] [0|3] 0:speed ctrl mode 1:throttle ctrl mode 2:reserve 3:reserve
int8 drive_mode_fb 0

# [] [0|3] 0:no use 1:D 2:N 3:R
int8 gear_status 0

# [m/s] [-50|50] Feedback is relative to actual speed
float32 actual_speed_fb 0.00

# [%] [0|100] Throttle request value 0~100%
float32 throttle_request_val_fb 0.0

# [m/s^2] [-20|20] Reserved
float32 vehicle_accel 0.00

# [] [0|15]
int8 vcu_cycle_count 0
