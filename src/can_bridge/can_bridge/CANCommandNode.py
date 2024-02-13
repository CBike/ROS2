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

        self.sub_throttle_cmd = self.create_subscription(ThrottleCommand, 'actuator/Throttle_Command', self.process_command, 10)
        self.sub_brake_cmd = self.create_subscription(BrakeCommand, 'actuator/Brake_Command', self.process_command, 10)
        self.sub_steer_cmd = self.create_subscription(SteerCommand, 'actuator/Steer_Command', self.process_command, 10)
        self.sub_gear_cmd = self.create_subscription(GearCommand, 'actuator/Gear_Command', self.process_command, 10)
        self.sub_park_cmd = self.create_subscription(ParkCommand, 'actuator/Park_Command', self.process_command, 10)
        self.sub_vmc_cmd = self.create_subscription(VehicleModeCommand, 'actuator/Vehicle_Mode_Command', self.process_command, 10)

    def process_command(self, msg):

        if isinstance(msg, ThrottleCommand):
            throttle_en_ctrl = struct.pack('>B', ThrottleCommand.throttle_en_ctrl)[0] & 0b00000001
            throttle_acc = struct.pack('>H', ThrottleCommand.throttle_acc)[0]

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

        else: pass

    def send_command(self):
        pass

    def parsing_command(self):
        pass


def main(args=None):
    pass

if __name__ == '__main__':
    pass

