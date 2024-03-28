from dataclasses import dataclass, fields
from struct import unpack


@dataclass
class ChassisWheelRpmRptData:
    left_front_wheel_speed: int = 0
    right_front_wheel_speed: int = 0
    left_rear_wheel_speed: int = 0
    right_rear_wheel_speed: int = 0

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def parsing_law_data_and_update_value(self, law_data: bytearray) -> None:
        self.left_front_wheel_speed = unpack('<h', law_data[0:2])[0]
        self.right_front_wheel_speed = unpack('<h', law_data[2:4])[0]
        self.left_rear_wheel_speed = unpack('<h', law_data[4:6])[0]
        self.right_rear_wheel_speed = unpack('<h', law_data[6:8])[0]
    @staticmethod
    def validate_left_front_wheel_speed(val):
        return -2000 <= val <= 2000

    @staticmethod
    def validate_right_front_wheel_speed(val):
        return -2000 <= val <= 2000

    @staticmethod
    def validate_left_rear_wheel_speed(val):
        return -2000 <= val <= 2000

    @staticmethod
    def validate_right_rear_wheel_speed(val):
        return -2000 <= val <= 2000

