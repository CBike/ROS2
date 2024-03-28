from dataclasses import dataclass, fields
from struct import unpack


@dataclass
class VehicleWorkStaRptData:
    drive_mode_fb: int = 0
    vehicle_power_on_status_fb: int = 0
    dc_working_status: int = 0
    vehicle_speed_limit_status: int = 0
    vehicle_speed_limit_val_fb: int = 0
    low_voltage_battery_voltage: int = 0
    emergency_stop_status_fb: int = 0
    vehicle_power_battery: int = 0
    vehicle_rear_crash_sensor_feedback: int = 0
    vcu_cycle_count: int = 0
    vcu_checksum: int = 0

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def parsing_law_data_and_update_value(self, law_data: bytearray) -> None:

        self.drive_mode_fb = unpack('<B', law_data[0:1])[0] & 0b00000011
        self.vehicle_power_on_status_fb = (unpack('<B', law_data[0:1])[0] >> 2) & 0b00000011
        self.dc_working_status = (unpack('<B', law_data[0:1])[0] >> 4) & 0b00000011
        self.vehicle_speed_limit_status = unpack('<B', law_data[1:2])[0] & 0b00000001
        self.vehicle_speed_limit_val_fb = unpack('<H', law_data[2:4])[0] * 0.1
        self.low_voltage_battery_voltage = unpack('<B', law_data[4:5])[0] * 0.1
        self.emergency_stop_status_fb = unpack('<B', law_data[5:6])[0] & 0b00001111
        self.vehicle_power_battery = (unpack('<B', law_data[5:6])[0] >> 4) & 0b00000001
        self.vehicle_rear_crash_sensor_feedback = (unpack('<B', law_data[5:6])[0] >> 5) & 0b00000001
        self.vcu_cycle_count = unpack('<B', law_data[6:7])[0] & 0b00001111
        self.vcu_checksum = unpack('<B', law_data[7:8])[0]

    @staticmethod
    def validate_driving_mode_fb(val):
        return 0 <= val <= 3

    @staticmethod
    def validate_vehicle_power_on_status_fb(val):
        return 0 <= val <= 3

    @staticmethod
    def validate_dc_working_status(val):
        return 0 <= val <= 2

    @staticmethod
    def validate_vehicle_speed_limit_status(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_vehicle_speed_limit_val_fb(val):
        return 0 <= val <= 50

    @staticmethod
    def validate_low_voltage_battery_voltage(val):
        return 0 <= val <= 25

    @staticmethod
    def validate_emergency_stop_status_fb(val):
        return 0 <= val <= 3

    @staticmethod
    def validate_vehicle_power_battery(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_vehicle_rear_crash_sensor_feedback(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_vcu_cycle_count(val):
        return 0 <= val <= 15
