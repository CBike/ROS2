from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array

@dataclass
class GearCommandData:
    gear_en_ctrl: int = 0
    gear_target: int = 0
    _checksum_103: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "gear_en_ctrl" and not self.validate_gear_en_ctrl(value):
                    value = 0
                elif field.name == "gear_target" and not self.validate_gear_target(value):
                    value = 0

                setattr(self, field.name, value)

        self.last_update_time = time_ns()

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_checksum(self):
        if self._checksum_103 >= 255:
            self._checksum_103 = 0
        self._checksum_103 += 1

    def reset_data(self):
        self.gear_en_ctrl = 0
        self.gear_target = 0
        self._checksum_103 = 0

    def get_bytearray(self):
        gear_en_ctrl = (self.gear_en_ctrl, 0, 0)
        gear_target = (self.gear_target, 8, 10)
        checksum_103 = (self._checksum_103, 56, 63)

        return generate_byte_array(8, gear_en_ctrl, gear_target, checksum_103)

    @staticmethod
    def validate_gear_en_ctrl(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_gear_target(val):
        return 0 <= val <= 4
