std_msgs/Header header

# [] [0|2] 0:no charge 1:charge 2:charge full
int8 vehicle_charge_status 0

# [%] [0|100]
int8 vehicle_power_battery_electricity_amount 0

# [V] [0|1000]
float32 vehicle_power_battery_voltage 0.0

# [A] [-1000|1000]
float32 vehicle_power_battery_current 0.0

# [℃] [-40|80]
int8 bms_maximum_monomer_temperature 0
