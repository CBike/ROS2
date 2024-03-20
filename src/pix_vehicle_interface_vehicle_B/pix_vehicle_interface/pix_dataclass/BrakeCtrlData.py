from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array


@dataclass
class BrakeCtrlData:
    vehicle_brake_control_enable: int = 0
    vehicle_brake_light_control: int = 0
    vehicle_brake_control: float = 0.0
    parking_control: int = 0
    cycle_count: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "vehicle_brake_control_enable" and not self.validate_vehicle_brake_control_enable(
                        value):
                    value = 0
                elif field.name == "vehicle_brake_light_control" and not self.validate_vehicle_brake_light_control(
                        value):
                    value = 0
                elif field.name == "vehicle_brake_control" and not self.validate_vehicle_brake_control(value):
                    value = 0.0
                elif field.name == "parking_control" and not self.validate_parking_control(value):
                    value = 0

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
            'vehicle_brake_control_enable': 0,
            'vehicle_brake_light_control': 0,
            'vehicle_brake_control': 0.0,
            'parking_control': 0,
            'cycle_count': 0,
        }

        self.update_value(**command_data)

    def get_bytearray(self):
        vehicle_brake_control_enable = (self.vehicle_brake_control_enable, 0, 0)
        vehicle_brake_light_control = (self.vehicle_brake_light_control, 1, 1)

        vehicle_brake_control_lower = ((int(self.vehicle_brake_control / 0.1) >> 2) & 0xFF, 8, 15)
        vehicle_brake_control_upper = (int(self.vehicle_brake_control / 0.1) & 0b11, 16, 17)

        parking_control = (self.parking_control, 24, 25)
        cycle_count = (self.cycle_count, 48, 51)

        return generate_byte_array(8, (vehicle_brake_control_enable,
                                       vehicle_brake_light_control,
                                       vehicle_brake_control_lower,
                                       vehicle_brake_control_upper,
                                       parking_control, cycle_count), checksum=True)

    @staticmethod
    def validate_vehicle_brake_control_enable(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_vehicle_brake_light_control(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_vehicle_brake_control(val):
        return 0 <= val <= 100

    @staticmethod
    def validate_parking_control(val):
        return 0 <= val <= 2

