from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array


@dataclass
class VehicleCtrlData:
    position_light_control: int = 0
    low_light_control: int = 0
    left_turn_light_control: int = 0
    right_turn_light_control: int = 0
    speed_limit_control: int = 0
    speed_limit: float = 0
    check_mode_enable: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "position_light_control" and not self.validate_position_light_control(value):
                    value = 0
                elif field.name == "low_light_control" and not self.validate_low_light_control(value):
                    value = 0
                elif field.name == "left_turn_light_control" and not self.validate_left_turn_light_control(value):
                    value = 0
                elif field.name == "right_turn_light_control" and not self.validate_right_turn_light_control(value):
                    value = 0
                elif field.name == "speed_limit_control" and not self.validate_speed_limit_control(value):
                    value = 0
                elif field.name == "speed_limit" and not self.validate_speed_limit(value):
                    value = 1.0
                elif field.name == "check_mode_enable" and not self.validate_check_mode_enable(value):
                    value = 0
                setattr(self, field.name, value)
        self.last_update_time = time_ns()

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def reset_data(self):

        command_data = {
            'position_light_control': 0,
            'low_light_control': 0,
            'left_turn_light_control': 0,
            'right_turn_light_control': 0,
            'speed_limit_control': 0,
            'speed_limit': 1.0,
            'check_mode_enable': 0,
        }

        self.update_value(**command_data)

    def get_bytearray(self):
        position_light_control = (self.position_light_control, 0, 0)
        low_light_control = (self.low_light_control, 1, 1)
        left_turn_light_control = (self.left_turn_light_control, 2, 2)
        right_turn_light_control = (self.right_turn_light_control, 3, 3)
        speed_limit_control = (self.speed_limit_control, 24, 24)
        speed_limit = (self.speed_limit, 32, 39)
        check_mode_enable = (self.check_mode_enable, 48, 48)

        return generate_byte_array(8, (position_light_control,
                                       low_light_control,
                                       left_turn_light_control,
                                       right_turn_light_control,
                                       speed_limit_control,
                                       speed_limit,
                                       check_mode_enable), checksum=False)

    @staticmethod
    def validate_position_light_control(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_low_light_control(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_left_turn_light_control(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_right_turn_light_control(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_speed_limit_control(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_speed_limit(val):
        return 1 <= val <= 20

    @staticmethod
    def validate_check_mode_enable(val):
        return 0 <= val <= 1
