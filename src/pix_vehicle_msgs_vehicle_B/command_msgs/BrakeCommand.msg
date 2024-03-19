std_msgs/Header header
# Control Message
#[] [0|1] 0-disable 1-enable
int8 aeb_en_ctrl 0
#[m/s^2] [0|10] brake deceleration
float32 brake_dec 0.0
#[] [0|255]
int32 checksum_101 0
#[%] [0|100]
float32 brake_pedal_target 0.0
#[] [0|1] 0-disable 1-enable
int8 brake_en_ctrl 0
