from dataclasses import dataclass, fields
from struct import unpack

@dataclass
class PowerStaRptData:
    vehicle_charge_status: int = 0
    vehicle_power_battery_electricity_amount: int = 0
    vehicle_power_battery_voltage: int = 0
    vehicle_power_battery_current: float = 0.0
    bms_maximum_monomer_temperature: int = 0

    def get_value(self, field_name):
        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCommandData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def parsing_law_data_and_update_value(self, law_data: bytearray) -> None:
        self.vehicle_charge_status = (unpack('<B', law_data[0:1])[0] >> 4) & 0b00000011
        self.vehicle_power_battery_electricity_amount = unpack('<B', law_data[1:2])[0]
        self.vehicle_power_battery_voltage = (unpack('<H', law_data[2:4])[0] * 0.1) - 1000
        self.vehicle_power_battery_current = (unpack('<H', law_data[4:6])[0] * 0.1) - 40
        self.bms_maximum_monomer_temperature = unpack('<B', law_data[6:7])[0]

    @staticmethod
    def validate_vehicle_charge_status(val):
        return 0 <= val <= 2

    @staticmethod
    def validate_vehicle_power_battery_electricity_amount(val):
        return 0 <= val <= 100

    @staticmethod
    def validate_vehicle_power_battery_voltage(val):
        return 0 <= val <= 1000

    @staticmethod
    def validate_vehicle_power_battery_current(val):
        return -1000 <= val <= 1000

    @staticmethod
    def validate_bms_maximum_monomer_temperature(val):
        return -40 <= val <= 80
