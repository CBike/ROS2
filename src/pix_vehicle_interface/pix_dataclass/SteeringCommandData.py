from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array


@dataclass
class SteeringCommandData:
    steer_en_ctrl: int = 0
    steer_angle_spd: int = 0
    steer_angle_target: int = 0
    _checksum_102: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "steer_en_ctrl" and not self.validate_steer_en_ctrl(value):
                    value = 0
                elif field.name == "steer_angle_spd" and not self.validate_steer_angle_spd(value):
                    value = 0
                elif field.name == "steer_angle_target" and not self.validate_steer_angle_target(value):
                    value = 0

                setattr(self, field.name, value)
        self.last_update_time = time_ns()

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_checksum(self):
        if self._checksum_102 >= 255:
            self._checksum_102 = 0
        self._checksum_102 += 1

    def reset_data(self):

        command_data = {
            'steer_en_ctrl': 0,
            'steer_angle_spd': 0,
            'steer_angle_target': 0,
            '_checksum_102': 0
        }

        self.update_value(**command_data)

    def get_bytearray(self):
        steer_en_ctrl = (self.steer_en_ctrl, 0, 0)
        steer_angle_spd = (self.steer_angle_spd, 8, 15)


        steer_angle_target_upper_data = ((((self.steer_angle_target + 500) >> 8) & 0xFF), 24, 31)
        steer_angle_target_lower_data = (((self.steer_angle_target + 500) & 0xFF), 32, 39)

        checksum_102 = (self._checksum_102, 56, 63)

        return generate_byte_array(8, steer_en_ctrl, steer_angle_spd, steer_angle_target_upper_data,
                                   steer_angle_target_lower_data, checksum_102)

    @staticmethod
    def validate_steer_en_ctrl(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_steer_angle_spd(val):
        return 0 <= val <= 250

    @staticmethod
    def validate_steer_angle_target(val):
        return -500 <= val <= 500