from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array, split_data


@dataclass
class ThrottleCommandData:
    throttle_en_ctrl: int = 0
    throttle_acc: float = 0.0
    throttle_pedal_target: float = 0.0
    vel_target: float = 0.0
    _checksum_100: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "throttle_en_ctrl" and not self.validate_throttle_en_ctrl(value):
                    value = 0
                elif field.name == "throttle_acc" and not self.validate_throttle_acc(value):
                    value = 0.0
                elif field.name == "throttle_pedal_target" and not self.validate_throttle_pedal_target(value):
                    value = 0.0
                elif field.name == "vel_target" and not self.validate_throttle_vel_target(value):
                    value = 0.0
                setattr(self, field.name, value)
        self.last_update_time = time_ns()

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_checksum(self):
        if self._checksum_100 >= 255:
            self._checksum_100 = 0
        self._checksum_100 += 1

    def reset_data(self):

        command_data = {
            'throttle_en_ctrl': 0,
            'throttle_acc': 0.0,
            'throttle_pedal_target': 0.0,
            'vel_target': 0.0,
            '_checksum_100': 0
        }
        self.update_value(**command_data)

    def get_bytearray(self):
        throttle_en_ctrl = (self.throttle_en_ctrl, 0, 0)
        throttle_acc_upper_byte = (((int(self.throttle_acc / 0.01) >> 2) & 0xFF), 8, 15)
        throttle_acc_lower_byte = ((int(self.throttle_acc / 0.01) & 0b11), 22, 23)

        throttle_pedal_target_upper_byte = (((int(self.throttle_pedal_target / 0.1) >> 8) & 0xFF), 24, 31)
        throttle_pedal_target_lower_byte = ((int(self.throttle_pedal_target / 0.1) & 0xFF), 32, 39)

        throttle_vel_target_upper_byte = (((int(self.vel_target / 0.01) >> 2) & 0xFF), 40, 47)
        throttle_vel_target_lower_byte = ((int(self.vel_target / 0.01) & 0b11), 54, 55)






        checksum = (self._checksum_100, 56, 63)

        return generate_byte_array(8,  throttle_en_ctrl, throttle_acc_upper_byte,
                                   throttle_acc_lower_byte, throttle_pedal_target_upper_byte,
                                   throttle_pedal_target_lower_byte, throttle_vel_target_upper_byte,
                                   throttle_vel_target_lower_byte, checksum)

    @staticmethod
    def validate_throttle_en_ctrl(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_throttle_acc(val):
        return 0 <= val <= 10

    @staticmethod
    def validate_throttle_pedal_target(val):
        return 0 <= val <= 100

    @staticmethod
    def validate_throttle_vel_target(val):
        return 0 <= val <= 10.23