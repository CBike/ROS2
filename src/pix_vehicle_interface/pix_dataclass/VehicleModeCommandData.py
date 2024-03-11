from dataclasses import dataclass, fields
from time import time_ns
from pix_dataclass.data_utils import generate_byte_array


@dataclass
class VehicleModeCommandData:
    steer_mode_ctrl: int = 0
    drive_mode_ctrl: int = 0
    turn_light_ctrl: int = 0
    _checksum_105: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "steer_mode_ctrl" and not self.validate_vmc_steer_mode_ctrl(value):
                    value = 0
                elif field.name == "drive_mode_ctrl" and not self.validate_vmc_drive_mode_ctrl(value):
                    value = 0
                elif field.name == "turn_light_ctrl" and not self.validate_vmc_turn_light_ctrl(value):
                    value = 0

                setattr(self, field.name, value)
        self.last_update_time = time_ns()

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_checksum(self):
        if self._checksum_105 >= 255:
            self._checksum_105 = 0
        self._checksum_105 += 1

    def reset_data(self):

        command_data = {
            'steer_mode_ctrl': 0,
            'drive_mode_ctrl': 0,
            'turn_light_ctrl': 0,
            '_checksum_105': 0
        }

        self.update_value(**command_data)

    def get_bytearray(self):

        steer_mode_ctrl = (self.steer_mode_ctrl, 0, 2)
        drive_mode_ctrl = (self.drive_mode_ctrl, 8, 10)
        turn_light_ctrl = (self.turn_light_ctrl, 16, 17)
        checksum_105 = (self._checksum_105, 56, 63)

        return generate_byte_array(8, steer_mode_ctrl, drive_mode_ctrl, turn_light_ctrl, checksum_105)

    @staticmethod
    def validate_vmc_steer_mode_ctrl(val):
        return 0 <= val <= 7

    @staticmethod
    def validate_vmc_drive_mode_ctrl(val):
        return 0 <= val <= 7

    @staticmethod
    def validate_vmc_turn_light_ctrl(val):
        return 0 <= val <= 3