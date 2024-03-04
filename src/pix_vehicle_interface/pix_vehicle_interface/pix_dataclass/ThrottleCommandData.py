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
        if self._checksum_100 >= 255 or self._checksum_100 < 0:
            self._checksum_100 = 0
        self._checksum_100 += 1

    def reset_data(self):
        self.throttle_en_ctrl = 0
        self.throttle_acc = 0.0
        self.throttle_pedal_target = 0.0
        self.vel_target = 0
        self._checksum_100 = 0

    def get_bytearray(self):
        en_ctrl = (self.throttle_en_ctrl, 0, 0)
        acc = (self.throttle_acc, 14, 23)
        pedal_target = (self.throttle_pedal_target, 24, 31)

        split_vel_1 = split_data(int(self.vel_target), 0, 7)
        split_vel_2 = split_data(int(self.vel_target), 8, 11)
        vel_1 = (split_vel_1, 40, 47)
        vel_2 = (split_vel_2, 52, 55)

        checksum = (self._checksum_100, 56, 63)

        return generate_byte_array(8, en_ctrl, acc, pedal_target, vel_1, vel_2, checksum)

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
