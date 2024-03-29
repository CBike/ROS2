from dataclasses import dataclass, fields
from struct import unpack


@dataclass
class BrakeStaRptData:
    brake_en_state: int = 0
    brake_light_en_state: int = 0
    parking_state: int = 0
    brake_pedal_val_fb: float = 0.0
    brake_pressure_fb: int = 0
    vcu_cycle_count: int = 0

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def parsing_law_data_and_update_value(self, law_data: bytearray) -> None:
        self.brake_en_state = unpack('<B', law_data[0:1])[0] & 0b00000001
        self.brake_light_en_state = (unpack('<B', law_data[0:1])[0] >> 2) & 0b00000001
        self.parking_state = (unpack('<B', law_data[0:1])[0] >> 4) & 0b00000011

        self.brake_pedal_val_fb = float(((unpack('<B', law_data[1:2])[0]) | (
                (unpack('<B', law_data[2:3])[0] & 0b00000011) << 8)) * 0.1)

        self.brake_pressure_fb = unpack('<B', law_data[3:4])[0]
        self.vcu_cycle_count = unpack('<B', law_data[6:7])[0] & 0b00001111

    @staticmethod
    def validate_brake_en_state(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_brake_light_en_state(val):
        return 0 <= val <= 1

    @staticmethod
    def validate_parking_state(val):
        return 0 <= val <= 3

    @staticmethod
    def validate_brake_pedal_val_fb(val):
        return 0 <= val <= 100

    @staticmethod
    def validate_brake_pressure_fb(val):
        return 0 <= val <= 100

    @staticmethod
    def validate_vcu_cycle_count(val):
        return 0 <= val <= 15
