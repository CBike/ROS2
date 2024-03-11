from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array


@dataclass
class BrakeCommandData:
    brake_en_ctrl: int = 0
    aeb_en_ctrl: int = 0
    brake_dec: float = 0.0
    brake_pedal_target: float = 0.0
    _checksum_101: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "brake_en_ctrl" and not self.validate_brake_en_ctrl(value):
                    value = 0
                elif field.name == "aeb_en_ctrl" and not self.validate_brake_aeb_en_ctrl(value):
                    value = 0
                elif field.name == "brake_dec" and not self.validate_brake_dec(value):
                    value = 0.0
                elif field.name == "brake_pedal_target" and not self.validate_brake_pedal_target(value):
                    value = 0.0
                setattr(self, field.name, value)
        self.last_update_time = time_ns()

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_checksum(self):
        if self._checksum_101 >= 255:
            self._checksum_101 = 0
        self._checksum_101 += 1

    def reset_data(self):

        command_data = {
            'brake_en_ctrl': 0,
            'aeb_en_ctrl': 0,
            'brake_dec': 0.0,
            'brake_pedal_target': 0.0,
            '_checksum_101': 0
        }
        self.update_value(**command_data)
    def get_bytearray(self):
        brake_en_ctrl = (self.brake_en_ctrl, 0, 0)
        aeb_en_ctrl = (self.aeb_en_ctrl, 1, 1)

        brake_dec_upper_byte = (((int(self.brake_dec / 0.01) >> 8) & 0xFF), 8, 15)
        brake_dec_lower_byte = ((int(self.brake_dec / 0.01) & 0b11), 22, 23)

        brake_pedal_target_upper_byte = (((int(self.brake_pedal_target / 0.1) >> 8) & 0xFF), 24, 31)
        brake_pedal_target_lower_byte = ((int(self.brake_pedal_target / 0.1) & 0xFF), 32, 39)

        checksum_101 = (self._checksum_101, 56, 63)

        return generate_byte_array(8, brake_en_ctrl, aeb_en_ctrl, brake_dec_upper_byte, brake_dec_lower_byte,
                                   brake_pedal_target_upper_byte, brake_pedal_target_lower_byte, checksum_101)

    @staticmethod
    def validate_brake_en_ctrl(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_brake_aeb_en_ctrl(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_brake_dec(val):
        return 0 <= val <= 10

    @staticmethod
    def validate_brake_pedal_target(val):
        return 0 <= val <= 100


