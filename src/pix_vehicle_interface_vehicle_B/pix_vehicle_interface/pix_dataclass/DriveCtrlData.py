from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array


@dataclass
class DriveCtrlData:
    vehicle_drive_control_enable: int = 0
    drive_mode_control: int = 0
    gear_control: int = 0
    vehicle_speed_control: float = 0.00
    vehicle_throttle_control: float = 0.0
    cycle_count: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "vehicle_drive_control_enable" and not self.validate_vehicle_drive_control_enable(
                        value):
                    value = 0
                elif field.name == "drive_mode_control" and not self.validate_drive_mode_control(value):
                    value = 0
                elif field.name == "gear_control" and not self.validate_gear_control(value):
                    value = 0
                elif field.name == "vehicle_speed_control" and not self.validate_vehicle_speed_control(value):
                    value = 0.00
                elif field.name == "vehicle_throttle_control" and not self.validate_vehicle_throttle_control(value):
                    value = 0.0
                setattr(self, field.name, value)
        self.last_update_time = time_ns()

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_cycle_count(self):
        if self.cycle_count >= 15:
            self.cycle_count = 0
        self.cycle_count += 1

    def reset_data(self):

        command_data = {
            'vehicle_drive_control_enable': 0,
            'drive_mode_control': 0,
            'gear_control': 0,
            'vehicle_speed_control': 0.00,
            'vehicle_throttle_control': 0.0,
            'cycle_count': 0
        }

        self.update_value(**command_data)

    def get_bytearray(self):
        vehicle_drive_control_enable = (self.vehicle_drive_control_enable, 0, 0)
        drive_mode_control = (self.drive_mode_control, 2, 3)
        gear_control = (self.gear_control, 4, 5)
        vehicle_speed_control_lower_data = ((int(self.vehicle_speed_control / 0.01) >> 8) & 0xFF, 8, 15)
        vehicle_speed_control_upper_data = (int(self.vehicle_speed_control / 0.01) & 0xFF, 16, 23)
        vehicle_throttle_control_lower_data = ((int(self.vehicle_throttle_control / 0.1) >> 2) & 0xFF, 24, 31)
        vehicle_throttle_control_upper_data = (int(self.vehicle_throttle_control / 0.1) & 0b11, 32, 33)
        cycle_count = (self.cycle_count, 48, 51)

        return generate_byte_array(8, (vehicle_drive_control_enable,
                                       drive_mode_control,
                                       gear_control,
                                       vehicle_speed_control_lower_data,
                                       vehicle_speed_control_upper_data,
                                       vehicle_throttle_control_lower_data,
                                       vehicle_throttle_control_upper_data,
                                       cycle_count), checksum=True)

    @staticmethod
    def validate_vehicle_drive_control_enable(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_drive_mode_control(val):
        return 0 <= val <= 3

    @staticmethod
    def validate_gear_control(val):
        return 0 <= val <= 3

    @staticmethod
    def validate_vehicle_speed_control(val):
        return 0 <= val <= 50

    @staticmethod
    def validate_vehicle_throttle_control(val):
        return 0 <= val <= 100
