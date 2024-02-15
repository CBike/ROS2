from rclpy.node import Node
from CANSender import CANSender
from std_msgs.msg import String
import rclpy
import struct

from can_bridge_interfaces.msg import (ThrottleCommand, BrakeCommand, GearCommand, ParkCommand, SteerCommand,
                                       VehicleModeCommand)


class CANCommandNode(Node):

    def __init__(self):
        super().__init__('CANCommandNode')
        self.can_channel = 'can0'
        self.can_sender = CANSender(self.can_channel)

        self.sub_throttle_cmd = self.create_subscription(ThrottleCommand, 'actuator/Throttle_Command',
                                                         self.dispatch_command, 10)
        self.sub_brake_cmd = self.create_subscription(BrakeCommand, 'actuator/Brake_Command',
                                                      self.dispatch_command, 10)
        self.sub_steer_cmd = self.create_subscription(SteerCommand, 'actuator/Steer_Command',
                                                      self.dispatch_command, 10)
        self.sub_gear_cmd = self.create_subscription(GearCommand, 'actuator/Gear_Command',
                                                     self.dispatch_command, 10)
        self.sub_park_cmd = self.create_subscription(ParkCommand, 'actuator/Park_Command',
                                                     self.dispatch_command, 10)
        self.sub_vmc_cmd = self.create_subscription(VehicleModeCommand, 'actuator/Vehicle_Mode_Command',
                                                    self.dispatch_command, 10)

    def dispatch_command(self, msg):
        empty_array = bytearray(8)
        if isinstance(msg, ThrottleCommand):
            empty_array[0:1] = CANCommandNode.generate_byte_array()
            empty_array[1:3] = CANCommandNode.generate_byte_array()
            empty_array[3:5] = CANCommandNode.generate_byte_array()
            empty_array[5:7] = CANCommandNode.generate_byte_array()
            empty_array[7:8] = CANCommandNode.generate_byte_array()
        elif isinstance(msg, BrakeCommand):
            pass
        elif isinstance(msg, SteerCommand):
            pass
        elif isinstance(msg, GearCommand):
            pass
        elif isinstance(msg, ParkCommand):
            pass
        elif isinstance(msg, VehicleModeCommand):
            pass

        else:
            pass

    @staticmethod
    def split_data(data: int, start_bit: int, end_bit: int) -> int:
        """Extracts data within the specified bit range from an integer.

        Args:
            data (int): The integer from which data will be extracted.
            start_bit (int): The starting bit index (inclusive) of the data to extract.
            end_bit (int): The ending bit index (inclusive) of the data to extract.

        Returns:
            int: The extracted data within the specified bit range.

        This function extracts data within the specified bit range from the given integer 'data'.
        It first calculates the bitmask for data extraction based on the start and end bit indices.
        Then, it applies the bitmask to the input data and performs a bit shift to obtain the
        extracted data. The extracted data is returned as an integer.
        """
        # Calculate the bitmask for data extraction
        mask = ((1 << (end_bit - start_bit + 1)) - 1) << (data.bit_length() - 1 - end_bit)
        # Extract data using the bitmask
        extracted_data = (data & mask) >> (data.bit_length() - 1 - end_bit)
        return extracted_data

    @staticmethod
    def generate_byte_array(data: int, start_bit: int, end_bit: int) -> bytearray:
        """Generates a byte array with specified data bits.

        Args:
            data (int): The integer data to be packed into the byte array.
            start_bit (int): The starting bit index of the data within the byte array.
            end_bit (int): The ending bit index of the data within the byte array.

        Returns:
            bytearray: A bytearray containing the specified data bits packed into bytes.

        The function generates a bytearray and fills it with the specified data bits
        within the given range of bit indices. The data is extracted from the input
        integer 'data' and packed into bytes, with each byte representing 8 bits.
        """
        # Calculate start_byte, start_bit_offset, end_byte, end_bit_offset
        start_byte, start_bit_offset = divmod(start_bit, 8)
        end_byte, end_bit_offset = divmod(end_bit, 8)

        # Create an empty byte array
        byte_array = bytearray(end_byte + 1)

        # Fill data bits into the byte array
        for byte_index in range(start_byte, end_byte + 1):
            byte = 0
            # Iterate through each bit in the byte
            for bit_offset in range(8):
                # Calculate the bit index in the byte array
                current_bit = byte_index * 8 + (7 - bit_offset)
                # Check if the current bit falls within the specified range
                if start_bit <= current_bit <= end_bit:
                    # Extract the bit value from the data and set it in the byte
                    bit_value = (data >> (end_bit - current_bit)) & 1
                    byte |= bit_value << bit_offset
            # Store the byte in the byte array
            byte_array[byte_index] = byte

        return byte_array

def main(args=None):
    pass

if __name__ == '__main__':
    pass

