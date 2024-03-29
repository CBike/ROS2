from dataclasses import dataclass, fields
from time import time_ns

from pix_dataclass.data_utils import generate_byte_array


@dataclass
class SteerCtrlData:
    vehicle_steering_control_enable: int = 0
    steering_mode_control: int = 0
    vehicle_steering_control_front: int = 0
    vehicle_steering_control_rear: int = 0
    vehicle_steering_wheel_speed_control: int = 480
    cycle_count: int = 0
    last_update_time: int = 0

    def update_value(self, **kwargs) -> None:
        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                if field.name == "vehicle_steering_control_enable" and not self.validate_vehicle_steering_control_enable(
                        value):
                    value = 0
                elif field.name == "steering_mode_control" and not self.validate_steering_mode_control(value):
                    value = 0
                elif field.name == "vehicle_steering_control_front" and not self.validate_vehicle_steering_control_front(
                        value):
                    value = 0
                elif field.name == "vehicle_steering_control_rear" and not self.validate_vehicle_steering_control_rear(
                        value):
                    value = 0
                elif field.name == "vehicle_steering_wheel_speed_control" and not self.validate_vehicle_steering_wheel_speed_control(
                        value):
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
            'vehicle_steering_control_enable': 0,
            'steering_mode_control': 0,
            'vehicle_steering_control_front': 0,
            'vehicle_steering_control_rear': 0,
            'vehicle_steering_wheel_speed_control': 0,
            'cycle_count': 0,
        }

        self.update_value(**command_data)

    def get_bytearray(self):
        vehicle_steering_control_enable = (self.vehicle_steering_control_enable, 0, 0)
        steering_mode_control = (self.steering_mode_control, 4, 7)

        vehicle_steering_control_front_lower = (int(self.vehicle_steering_control_front) & 0xFF, 8, 15)
        vehicle_steering_control_front_upper = ((int(self.vehicle_steering_control_front) >> 8) & 0xFF, 16, 23)

        vehicle_steering_control_rear_lower = (int(self.vehicle_steering_control_rear) & 0xFF, 24, 32)
        vehicle_steering_control_rear_upper = ((int(self.vehicle_steering_control_rear) >> 8) & 0xFF, 32, 39)

        vehicle_steering_wheel_speed_control = (self.vehicle_steering_wheel_speed_control / 2, 40, 47)
        cycle_count = (self.cycle_count, 48, 51)

        return generate_byte_array(8, vehicle_steering_control_enable,
                                   steering_mode_control,
                                   vehicle_steering_control_front_lower,
                                   vehicle_steering_control_front_upper,
                                   vehicle_steering_control_rear_lower,
                                   vehicle_steering_control_rear_upper,
                                   vehicle_steering_wheel_speed_control,
                                   cycle_count, checksum=True)

    @staticmethod
    def validate_vehicle_steering_control_enable(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_steering_mode_control(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_vehicle_steering_control_front(val):
        return -500 <= val <= 500

    @staticmethod
    def validate_vehicle_steering_control_rear(val):
        return -500 <= val <= 500

    @staticmethod
    def validate_vehicle_steering_wheel_speed_control(val):
        return 0 <= val <= 500
