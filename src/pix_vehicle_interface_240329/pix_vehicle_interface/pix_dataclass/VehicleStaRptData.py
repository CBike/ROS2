from dataclasses import dataclass, fields
from struct import unpack


@dataclass
class VehicleStaRptData:
    position_light_status_fb: int = 0
    low_light_status_fb: int = 0
    left_turning_light_status_fb: int = 0
    right_turning_light_status_fb: int = 0
    hazard_warning_light_switch_status: int = 0

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def parsing_law_data_and_update_value(self, law_data: bytearray) -> None:

        self.position_light_status_fb = unpack('<B', law_data[0:1])[0] & 0b00000001
        self.low_light_status_fb = (unpack('<B', law_data[0:1])[0] >> 1) & 0b00000001
        self.left_turning_light_status_fb = (unpack('<B', law_data[0:1])[0] >> 2) & 0b00000001
        self.right_turning_light_status_fb = (unpack('<B', law_data[0:1])[0] >> 3) & 0b00000001
        self.hazard_warning_light_switch_status = (unpack('<B', law_data[0:1])[0] >> 6) & 0b00000001

    @staticmethod
    def validate_status_fb(val):
        return 0 <= val <= 1
