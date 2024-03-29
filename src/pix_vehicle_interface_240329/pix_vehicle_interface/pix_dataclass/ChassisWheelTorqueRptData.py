from dataclasses import dataclass, fields
from struct import unpack


@dataclass
class ChassisWheelTorqueRptData:
    left_front_wheel_torque: float = 0.0
    right_front_wheel_torque: float = 0.0
    left_rear_wheel_torque: float = 0.0
    right_rear_wheel_torque: float = 0.0

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def parsing_law_data_and_update_value(self, law_data: bytearray) -> None:
        self.left_front_wheel_torque = float(unpack('<h', law_data[0:2])[0] * 0.1)
        self.right_front_wheel_torque = float(unpack('<h', law_data[2:4])[0] * 0.1)
        self.left_rear_wheel_torque = float(unpack('<h', law_data[4:6])[0] * 0.1)
        self.right_rear_wheel_torque = float(unpack('<h', law_data[6:8])[0] * 0.1)

    @staticmethod
    def validate_left_front_wheel_torque(val):
        return -200 <= val <= 200

    @staticmethod
    def validate_right_front_wheel_torque(val):
        return -200 <= val <= 200

    @staticmethod
    def validate_left_rear_wheel_torque(val):
        return -200 <= val <= 200

    @staticmethod
    def validate_right_rear_wheel_torque(val):
        return -200 <= val <= 200
