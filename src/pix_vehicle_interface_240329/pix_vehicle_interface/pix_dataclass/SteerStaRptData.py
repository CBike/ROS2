from dataclasses import dataclass, fields
from struct import unpack


@dataclass
class SteerStaRptData:
    steering_en_state: int = 0
    steering_control_out_of_bounds_reminder: int = 0
    steering_mode_fb: int = 0
    front_steering_angle_fb: int = 0
    rear_steering_angle_fb: int = 0
    set_steering_angle_speed_fb: int = 0
    vcu_cycle_count: int = 0

    def parsing_law_data_and_update_value(self, law_data: bytearray) -> None:
        self.steering_en_state = unpack('<B', law_data[0:1])[0] & 0b00000001
        self.steering_control_out_of_bounds_reminder = (unpack('<B', law_data[0:1])[0] >> 1) & 0b00000001
        self.steering_mode_fb = (unpack('<B', law_data[0:1])[0] >> 4) & 0b00001111
        self.front_steering_angle_fb = unpack('<h', law_data[1:3])[0]
        self.rear_steering_angle_fb = unpack('<h', law_data[3:5])[0]
        self.set_steering_angle_speed_fb = unpack('<B', law_data[5:6])[0] * 2
        self.vcu_cycle_count = unpack('<B', law_data[6:7])[0] & 0b00001111

    @staticmethod
    def validate_steering_en_state(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_steering_control_out_of_bounds_reminder(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_steering_mode_fb(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_front_steering_angle_fb(val):
        return -500 <= val <= 500

    @staticmethod
    def validate_rear_steering_angle_fb(val):
        return -500 <= val <= 500

    @staticmethod
    def validate_set_steering_angle_speed_fb(val):
        return 0 <= val <= 500

    @staticmethod
    def validate_data_vcu_cycle_count(val):
        return 0 <= val <= 15
