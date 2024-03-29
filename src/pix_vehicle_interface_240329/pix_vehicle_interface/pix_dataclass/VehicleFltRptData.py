from dataclasses import dataclass, fields
from struct import unpack


@dataclass
class VehicleFltRptData:
    motor_system_overheating: int = 0
    battery_system_overheating: int = 0
    brake_system_overheating: int = 0
    steering_system_overheating: int = 0
    battery_voltage_is_too_low: int = 0
    system_error: int = 0
    brake_system_failure: int = 0
    parking_system_failure: int = 0
    front_steering_system_fault: int = 0
    rear_steering_system_failure: int = 0
    left_front_motor_system_failure: int = 0
    right_front_motor_system_failure: int = 0
    left_rear_motor_system_failure: int = 0
    right_rear_motor_system_failure: int = 0
    bms_system_failure: int = 0
    dc_system_failure: int = 0

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def parsing_law_data_and_update_value(self, law_data: bytearray) -> None:
        # TODO Bitwise operation required
        self.motor_system_overheating = unpack('<B', law_data[0:1])[0]
        self.battery_system_overheating = unpack('<B', law_data[0:1])[0]
        self.brake_system_overheating = unpack('<B', law_data[0:1])[0]
        self.steering_system_overheating = unpack('<B', law_data[0:1])[0]
        self.battery_voltage_is_too_low = unpack('<B', law_data[0:1])[0]
        self.system_error = unpack('<B', law_data[1:2])[0]
        self.brake_system_failure = unpack('<B', law_data[1:2])[0]
        self.parking_system_failure = unpack('<B', law_data[2:3])[0]
        self.front_steering_system_fault = unpack('<B', law_data[2:3])[0]
        self.rear_steering_system_failure = unpack('<B', law_data[3:4])[0]
        self.left_front_motor_system_failure = unpack('<B', law_data[3:4])[0]
        self.right_front_motor_system_failure = unpack('<B', law_data[4:5])[0]
        self.left_rear_motor_system_failure = unpack('<B', law_data[4:5])[0]
        self.right_rear_motor_system_failure = unpack('<B', law_data[5:6])[0]
        self.bms_system_failure = unpack('<B', law_data[5:6])[0]
        self.dc_system_failure = unpack('<B', law_data[6:7])[0]

    @staticmethod
    def validate_motor_system_overheating(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_battery_system_overheating(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_brake_system_overheating(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_steering_system_overheating(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_battery_voltage_is_too_low(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_system_error(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_brake_system_failure(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_parking_system_failure(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_front_steering_system_fault(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_rear_steering_system_failure(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_left_front_motor_system_failure(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_right_front_motor_system_failure(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_left_rear_motor_system_failure(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_right_rear_motor_system_failure(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_bms_system_failure(val):
        return 0 <= val <= 4

    @staticmethod
    def validate_dc_system_failure(val):
        return 0 <= val <= 4
