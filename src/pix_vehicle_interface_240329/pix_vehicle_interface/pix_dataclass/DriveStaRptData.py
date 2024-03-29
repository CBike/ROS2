from dataclasses import dataclass, fields
from struct import unpack


@dataclass
class DriveStaRptData:
    drive_en_state: int = 0
    reminder_for_drive_control_out_of_bounds: int = 0
    drive_mode_fb: int = 0
    gear_status: int = 0
    actual_speed_fb: float = 0.00
    throttle_request_val_fb: float = 0
    vehicle_accel: float = 0.00
    vcu_cycle_count: int = 0

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def parsing_law_data_and_update_value(self, law_data: bytearray) -> None:

        self.drive_en_state = unpack('<B', law_data[0:1])[0] & 0b00000001
        self.reminder_for_drive_control_out_of_bounds = (unpack('<B', law_data[0:1])[0] >> 1) & 0b00000001
        self.drive_mode_fb = (unpack('<B', law_data[0:1])[0] >> 2) & 0b00000011
        self.gear_status = (unpack('<B', law_data[0:1])[0] >> 4) & 0b00000011
        self.actual_speed_fb = unpack('<H', law_data[1:3])[0] * 0.01

        self.throttle_request_val_fb = float(((unpack('<B', law_data[3:4])[0])
                                              | ((unpack('<B', law_data[4:5])[0] & 0b00000011) << 8)) * 0.1)

        self.vehicle_accel = unpack('<h', law_data[5:7])[0] * 0.01
        self.vcu_cycle_count = unpack('<B', law_data[7:8])[0] & 0b00001111

    @staticmethod
    def validate_drive_en_state(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_reminder_for_drive_control_out_of_bounds(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_drive_mode_fb(val):
        return 0 <= val <= 3

    @staticmethod
    def validate_gear_status(val):
        return 0 <= val <= 3

    @staticmethod
    def validate_actual_speed_fb(val):
        return -50 <= val <= 50

    @staticmethod
    def validate_throttle_request_val_fb(val):
        return 0 <= val <= 100

    @staticmethod
    def validate_vehicle_accel(val):
        return -20 <= val <= 20

    @staticmethod
    def validate_vcu_cycle_count(val):
        return 0 <= val <= 15
