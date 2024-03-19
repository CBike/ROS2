from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array


@dataclass
class ParkCommandData:
    park_en_ctrl: int = 0
    park_target: int = 0
    _checksum_104: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "park_en_ctrl" and not self.validate_park_en_ctrl(value):
                    value = 0
                elif field.name == "park_target" and not self.validate_park_target(value):
                    value = 0

                setattr(self, field.name, value)
        self.last_update_time = time_ns()

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_checksum(self):
        if self._checksum_104 >= 255:
            self._checksum_104 = 0
        self._checksum_104 += 1

    def reset_data(self):

        reset_data = {
            'park_en_ctrl': 0,
            'park_target': 0,
            '_checksum_104': 0
        }
        self.update_value(**reset_data)

    def get_bytearray(self):
        park_en_ctrl = (self.park_en_ctrl, 0, 0)
        park_target = (self.park_target, 8, 8)
        checksum_104 = (self._checksum_104, 56, 63)

        return generate_byte_array(8, park_en_ctrl, park_target, checksum_104)

    @staticmethod
    def validate_park_en_ctrl(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_park_target(val):
        return 0 <= val <= 1