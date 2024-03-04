from rclpy.node import Node
import rclpy
from time import time_ns

from can_utils.can_sender import CANSender

from pix_vehicle_msgs.msg import (ThrottleCommand, BrakeCommand, GearCommand, ParkCommand, SteeringCommand,
                                  VehicleModeCommand)

from pix_dataclass.ThrottleCommandData import ThrottleCommandData
from pix_dataclass.BrakeCommandData import BrakeCommandData
from pix_dataclass.SteeringCommandData import SteeringCommandData
from pix_dataclass.GearCommandData import GearCommandData
from pix_dataclass.ParkCommandData import ParkCommandData
from pix_dataclass.VehicleModeCommandData import VehicleModeCommandData


class CANCommandNode(Node):

    def __init__(self):
        super().__init__('CANCommandNode')
        self.can_channel = 'can0'
        self.can_sender = CANSender(self.can_channel)

        self.throttle_data = ThrottleCommandData()
        self.brake_data = BrakeCommandData()
        self.steering_data = SteeringCommandData()
        self.gear_data = GearCommandData()
        self.park_data = ParkCommandData()
        self.vm_data = VehicleModeCommandData()

        self.sub_throttle_cmd = self.create_subscription(ThrottleCommand, 'Vehicle/Pix/Throttle_Command',
                                                         self.dispatch_command, 10)

        self.sub_brake_cmd = self.create_subscription(BrakeCommand, 'Vehicle/Pix/Brake_Command',
                                                      self.dispatch_command, 10)

        self.sub_steer_cmd = self.create_subscription(SteeringCommand, 'Vehicle/Pix/Steer_Command',
                                                      self.dispatch_command, 10)

        self.sub_gear_cmd = self.create_subscription(GearCommand, 'Vehicle/Pix/Gear_Command',
                                                     self.dispatch_command, 10)

        self.sub_park_cmd = self.create_subscription(ParkCommand, 'Vehicle/Pix/Park_Command',
                                                     self.dispatch_command, 10)

        self.sub_vmc_cmd = self.create_subscription(VehicleModeCommand, 'Vehicle/Pix/Vehicle_Mode_Command',
                                                    self.dispatch_command, 10)

        self.can_send_timer = self.create_timer(0.02, self.can_send_timer_callback)

    def dispatch_command(self, msg):

        if isinstance(msg, ThrottleCommand):
            command_data = {
                'throttle_en_ctrl': msg.throttle_en_ctrl,
                'throttle_acc': msg.throttle_acc,
                'throttle_pedal_target': msg.throttle_pedal_target,
                'vel_target': msg.vel_target,
            }

            self.throttle_data.update_value(**command_data)

        elif isinstance(msg, BrakeCommand):
            command_data = {
                'brake_en_ctrl': msg.brake_en_ctrl,
                'aeb_en_ctrl': msg.aeb_en_ctrl,
                'brake_dec': msg.brake_dec,
                'brake_pedal_target': msg.brake_pedal_target,
            }

            self.brake_data.update_value(**command_data)

        elif isinstance(msg, SteeringCommand):
            command_data = {
                'steer_en_ctrl': msg.steer_en_ctrl,
                'steer_angle_spd': msg.steer_angle_spd,
                'steer_angle_target': msg.steer_angle_target,
            }

            self.steering_data.update_value(**command_data)

        elif isinstance(msg, GearCommand):

            command_data = {
                'gear_en_ctrl': msg.gear_en_ctrl,
                'gear_target': msg.gear_target,
            }

            self.gear_data.update_value(**command_data)
        elif isinstance(msg, ParkCommand):

            command_data = {
                'park_en_ctrl': msg.park_en_ctrl,
                'park_target': msg.park_target,
            }

            self.park_data.update_value(**command_data)
        elif isinstance(msg, VehicleModeCommand):
            command_data = {
                'steer_mode_ctrl': msg.steer_mode_ctrl,
                'drive_mode_ctrl': msg.drive_mode_ctrl,
                'turn_light_ctrl': msg.turn_light_ctrl,
            }

            self.vm_data.update_value(**command_data)

    def can_send_timer_callback(self):
        def check_communication_timeout():
            now = time_ns()
            if now - self.throttle_data.get_value('last_update_time') > 300000000:
                self.throttle_data.reset_data()

            elif now - self.brake_data.get_value('last_update_time') > 300000000:
                self.brake_data.reset_data()

            elif now - self.steering_data.get_value('last_update_time') > 300000000:
                self.steering_data.reset_data()

            elif now - self.gear_data.get_value('last_update_time') > 300000000:
                self.gear_data.reset_data()

            elif now - self.park_data.get_value('last_update_time') > 300000000:
                self.park_data.reset_data()

            elif now - self.vm_data.get_value('last_update_time') > 300000000:
                self.vm_data.reset_data()

        check_communication_timeout()

        self.throttle_data.add_checksum()
        self.brake_data.add_checksum()
        self.steering_data.add_checksum()
        self.gear_data.add_checksum()
        self.park_data.add_checksum()
        self.vm_data.add_checksum()

        self.can_sender.send(0x100, self.throttle_data.get_bytearray())
        self.can_sender.send(0x101, self.brake_data.get_bytearray())
        self.can_sender.send(0x102, self.steering_data.get_bytearray())
        self.can_sender.send(0x103, self.gear_data.get_bytearray())
        self.can_sender.send(0x104, self.park_data.get_bytearray())
        self.can_sender.send(0x105, self.vm_data.get_bytearray())


def main(args=None):
    rclpy.init(args=args)
    node = CANCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
