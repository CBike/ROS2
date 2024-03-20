from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array


@dataclass
class WheelTorqueCtrlData:
    left_front_motor_torque: float = 0.0
    right_front_motor_torque: float = 0.0
    left_rear_motor_torque: float = 0.0
    right_rear_motor_torque: float = 0.0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "left_front_motor_torque" and not self.validate_left_front_motor_torque(value):
                    value = 0.0
                elif field.name == "right_front_motor_torque" and not self.validate_right_front_motor_torque(value):
                    value = 0.0
                elif field.name == "left_rear_motor_torque" and not self.validate_left_rear_motor_torque(value):
                    value = 0.0
                elif field.name == "right_rear_motor_torque" and not self.validate_right_rear_motor_torque(value):
                    value = 0.0

                setattr(self, field.name, value)
        self.last_update_time = time_ns()

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def reset_data(self):

        command_data = {
            'left_front_motor_torque': 0,
            'right_front_motor_torque': 0,
            'left_rear_motor_torque': 0,
            'right_rear_motor_torque': 0
        }

        self.update_value(**command_data)

    def get_bytearray(self):
        left_front_motor_torque_lower = (self.left_front_motor_torque, 0, 7)
        left_front_motor_torque_upper = (self.left_front_motor_torque, 8, 15)
        right_front_motor_torque_lower = (self.right_front_motor_torque, 16, 23)
        right_front_motor_torque_upper = (self.right_front_motor_torque, 24, 31)
        left_rear_motor_torque_lower = (self.left_rear_motor_torque, 32, 39)
        left_rear_motor_torque_upper = (self.left_rear_motor_torque, 40, 47)
        right_rear_motor_torque_lower = (self.right_rear_motor_torque, 48, 55)
        right_rear_motor_torque_upper = (self.right_rear_motor_torque, 56, 63)

        return generate_byte_array(8, (left_front_motor_torque_lower,
                                       left_front_motor_torque_upper,
                                       right_front_motor_torque_lower,
                                       right_front_motor_torque_upper,
                                       left_rear_motor_torque_lower,
                                       left_rear_motor_torque_upper,
                                       right_rear_motor_torque_lower,
                                       right_rear_motor_torque_upper), checksum=False)

    @staticmethod
    def validate_left_front_motor_torque(val):
        return -200 <= val <= 200

    @staticmethod
    def validate_right_front_motor_torque(val):
        return -200 <= val <= 200

    @staticmethod
    def validate_left_rear_motor_torque(val):
        return -200 <= val <= 200

    @staticmethod
    def validate_right_rear_motor_torque(val):
        return -200 <= val <= 2000


