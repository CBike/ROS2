from rclpy.node import Node
import rclpy
from time import time_ns

from can_utils.can_sender import CANSender

from pix_vehicle_msgs.msg import (BrakeCtrl, DriveCtrl, SteerCtrl, VehicleCtrl, WheelTorqueCtrl)

from pix_dataclass.BrakeCtrlData import BrakeCtrlData
from pix_dataclass.DriveCtrlData import DriveCtrlData
from pix_dataclass.SteerCtrlData import SteerCtrlData
from pix_dataclass.VehicleCtrlData import VehicleCtrlData
from pix_dataclass.WheelTorqueCtrlData import WheelTorqueCtrlData


class CANCommandNode(Node):

    def __init__(self):
        super().__init__('CANCommandNode')
        self.can_channel = 'can0'
        self.can_sender = CANSender(self.can_channel)

        self.drive_ctrl_data = BrakeCtrlData()
        self.brake_ctrl_data = DriveCtrlData()
        self.steering_data = SteerCtrlData()
        self.vehicle_ctrl_data = VehicleCtrlData()
        self.wheel_ctrl_data = WheelTorqueCtrlData()

        self.sub_drive_ctrl_data = self.create_subscription(DriveCtrl, 'Vehicle/Pix/drive_ctrl_data',
                                                            self.dispatch_command, 10)

        self.sub_brake_ctrl_data = self.create_subscription(BrakeCtrl, 'Vehicle/Pix/brake_ctrl_data',
                                                            self.dispatch_command, 10)

        self.sub_steering_data = self.create_subscription(SteerCtrl, 'Vehicle/Pix/steering_data',
                                                          self.dispatch_command, 10)

        self.sub_vehicle_ctrl_data = self.create_subscription(VehicleCtrl, 'Vehicle/Pix/vehicle_ctrl_data',
                                                              self.dispatch_command, 10)

        self.sub_wheel_ctrl_data = self.create_subscription(WheelTorqueCtrl, 'Vehicle/Pix/wheel_ctrl_data',
                                                            self.dispatch_command, 10)

        self.throttle_cmd_send_timer = self.create_timer(0.02, self.drive_ctrl_data_timer_callback)
        self.brake_cmd_send_timer = self.create_timer(0.02, self.brake_ctrl_data_timer_callback)
        self.steer_cmd_send_timer = self.create_timer(0.02, self.steering_data_timer_callback)
        self.gear_cmd_send_timer = self.create_timer(0.2, self.vehicle_ctrl_data_timer_callback)
        self.park_cmd_send_timer = self.create_timer(0.02, self.wheel_ctrl_data_timer_callback)

    def dispatch_command(self, msg):

        if isinstance(msg, DriveCtrl):
            command_data = {
                'vehicle_drive_control_enable': msg.vehicle_drive_control_enable,
                'drive_mode_control': msg.drive_mode_control,
                'gear_control': msg.gear_control,
                'vehicle_speed_control': msg.vehicle_speed_control,
                'vehicle_throttle_control': msg.vehicle_throttle_control,
            }

            self.drive_ctrl_data.update_value(**command_data)

        elif isinstance(msg, BrakeCtrl):
            command_data = {
                'vehicle_brake_control_enable': msg.vehicle_brake_control_enable,
                'vehicle_brake_light_control': msg.vehicle_brake_light_control,
                'vehicle_brake_control': msg.vehicle_brake_control,
                'parking_control': msg.parking_control,
            }

            self.brake_ctrl_data.update_value(**command_data)

        elif isinstance(msg, SteerCtrl):
            command_data = {
                'vehicle_steering_control_enable': msg.vehicle_steering_control_enable,
                'steering_mode_control': msg.steering_mode_control,
                'vehicle_steering_control_front': msg.vehicle_steering_control_front,
                'vehicle_steering_control_rear': msg.vehicle_steering_control_rear,
                'vehicle_steering_wheel_speed_control': msg.vehicle_steering_wheel_speed_control,
            }

            self.steering_data.update_value(**command_data)

        elif isinstance(msg, VehicleCtrl):

            command_data = {
                'position_light_control': msg.position_light_control,
                'low_light_control': msg.low_light_control,
                'left_turn_light_control': msg.left_turn_light_control,
                'right_turn_light_control': msg.right_turn_light_control,
                'speed_limit_control': msg.speed_limit_control,
                'speed_limit': msg.speed_limit,
                'check_mode_enable': msg.check_mode_enable,
            }
            self.vehicle_ctrl_data.update_value(**command_data)

        elif isinstance(msg, WheelTorqueCtrl):

            command_data = {
                'left_front_motor_torque': msg.left_front_motor_torque,
                'right_front_motor_torque': msg.right_front_motor_torque,
                'left_rear_motor_torque': msg.left_rear_motor_torque,
                'right_rear_motor_torque': msg.right_rear_motor_torque,
            }

            self.wheel_ctrl_data.update_value(**command_data)

    def drive_ctrl_data_timer_callback(self):
        now = time_ns()

        if (now - self.drive_ctrl_data.get_value('last_update_time')) > 300000000:
            self.drive_ctrl_data.reset_data()

        self.drive_ctrl_data.add_cycle_count()
        self.can_sender.send(0x130, self.drive_ctrl_data.get_bytearray())

    def brake_ctrl_data_timer_callback(self):
        now = time_ns()

        if (now - self.brake_ctrl_data.get_value('last_update_time')) > 300000000:
            self.brake_ctrl_data.reset_data()

        self.brake_ctrl_data.add_cycle_count()
        self.can_sender.send(0x131, self.brake_ctrl_data.get_bytearray())

    def steering_data_timer_callback(self):
        now = time_ns()

        if (now - self.steering_data.get_value('last_update_time')) > 300000000:
            self.steering_data.reset_data()

        self.steering_data.add_cycle_count()
        self.can_sender.send(0x132, self.steering_data.get_bytearray())

    def vehicle_ctrl_data_timer_callback(self):
        now = time_ns()

        if (now - self.vehicle_ctrl_data.get_value('last_update_time')) > 300000000:
            self.gear_data.reset_data()

        self.can_sender.send(0x133, self.vehicle_ctrl_data.get_bytearray())

    def wheel_ctrl_data_timer_callback(self):
        now = time_ns()

        if (now - self.wheel_ctrl_data.get_value("last_update_time")) > 300000000:
            self.wheel_ctrl_data.reset_data()

        self.can_sender.send(0x135, self.wheel_ctrl_data.get_bytearray())


def main(args=None):
    rclpy.init(args=args)
    node = CANCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
