from rclpy.node import Node
import rclpy
from time import time_ns

from can_utils.can_sender import CANSender

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand
from tier4_control_msgs.msg import GateMode
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_auto_vehicle_msgs.msg import HazardLightsCommand
from tier4_vehicle_msgs.msg import ActuationCommandStamped

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

        self.drive_ctrl_data = DriveCtrlData()
        self.brake_ctrl_data = BrakeCtrlData()
        self.steer_ctrl_data = SteerCtrlData()
        self.vehicle_ctrl_data = VehicleCtrlData()
        self.wheel_ctrl_data = WheelTorqueCtrlData()

        self.sub_ackermann_ctrl_cmd = self.create_subscription(AckermannControlCommand, '/control/command/control_cmd',
                                                               self.dispatch_command, 10)

        self.sub_gear_ctrl_cmd = self.create_subscription(GearCommand, '/control/command/gear_cmd',
                                                          self.dispatch_command, 10)

        self.sub_gate_mode_ctrl_cmd = self.create_subscription(GateMode, '/control/current_gate_mode',
                                                               self.dispatch_command, 10)

        self.sub_vehicle_emergency_ctrl_cmd = self.create_subscription(VehicleEmergencyStamped,
                                                                       '/control/command/emergency_cmd',
                                                                       self.dispatch_command, 10)

        self.sub_turn_indicators_ctrl_cmd = self.create_subscription(TurnIndicatorsCommand,
                                                                     '/control/command/turn_indicators_cmd',
                                                                     self.dispatch_command, 10)

        self.sub_turn_hazard_lights_ctrl_cmd = self.create_subscription(HazardLightsCommand,
                                                                        '/control/command/hazard_lights_cmd',
                                                                        self.dispatch_command, 10)

        self.sub_actuation_ctrl_cmd = self.create_subscription(ActuationCommandStamped,
                                                               '/control/command/actuation_cmd',
                                                               self.dispatch_command, 10)

        self.drive_ctrl_send_timer = self.create_timer(0.02, self.drive_ctrl_data_timer_callback)
        self.brake_ctrl_send_timer = self.create_timer(0.02, self.brake_ctrl_data_timer_callback)
        self.steer_ctrl_send_timer = self.create_timer(0.02, self.steering_data_timer_callback)
        self.vehicle_ctrl_send_timer = self.create_timer(0.2, self.vehicle_ctrl_data_timer_callback)
        self.wheel_ctrl_send_timer = self.create_timer(0.02, self.wheel_ctrl_data_timer_callback)

    def dispatch_command(self, msg):

        if isinstance(msg, AckermannControlCommand):
            command_data = {
                'vehicle_steering_control_enable': 1,
                # only use front ackermann
                'steering_mode_control': 0,
                'vehicle_steering_control_front': msg.lateral.steering_tire_angle,
                'vehicle_steering_control_rear': 0,
                'vehicle_steering_wheel_speed_control': 480,
            }

            self.steer_ctrl_data.update_value(**command_data)

        elif isinstance(msg, GearCommand):
            gear_command = 0

            if msg.command == 1:
                gear_command = 2
            elif 2 <= msg.command <= 19:
                gear_command = 1
            elif 20 <= msg.command <= 21:
                gear_command = 3
            else:
                gear_command = 0

            command_data = {
                'vehicle_drive_control_enable': 1,
                'drive_mode_control': 1,
                'gear_control': gear_command,
                'vehicle_speed_control': 0.00,
                'vehicle_throttle_control': 0.0,
            }

            self.drive_ctrl_data.update_value(**command_data)
            self.get_logger().info(f'GearCommand, drive_ctrl_data : {self.drive_ctrl_data}')

        elif isinstance(msg, GateMode):
            pass

        elif isinstance(msg, VehicleEmergencyStamped):
            # TODO: Apply emergency braking system and parking brake using BrakeCtrlDataclass and DriveCtrlDataClass.
            pass

        elif isinstance(msg, TurnIndicatorsCommand):

            left_turn_light_control = 0
            right_turn_light_control = 0

            if msg.command == 2:
                left_turn_light_control = 1
            elif msg.command == 3:
                right_turn_light_control = 1

            command_data = {
                'position_light_control': 1,
                'low_light_control': 0,
                'left_turn_light_control': left_turn_light_control,
                'right_turn_light_control': right_turn_light_control,
                'speed_limit_control': 0,
                'speed_limit': 0,
                'check_mode_enable': 0,
            }

            self.vehicle_ctrl_data.update_value(**command_data)

        elif isinstance(msg, HazardLightsCommand):

            left_turn_light_control = 0
            right_turn_light_control = 0

            if msg.command == 2:
                left_turn_light_control = 1
                right_turn_light_control = 1

            command_data = {
                'position_light_control': 0,
                'low_light_control': 0,
                'left_turn_light_control': left_turn_light_control,
                'right_turn_light_control': right_turn_light_control,
                'speed_limit_control': 0,
                'speed_limit': 0,
                'check_mode_enable': 0,
            }

            self.vehicle_ctrl_data.update_value(**command_data)

        elif isinstance(msg, ActuationCommandStamped):

            command_data_acc = {
                'vehicle_drive_control_enable': 1,
                'drive_mode_control': 1,
                'vehicle_speed_control': float(msg.actuation.accel_cmd),
            }
            command_data_brake = {
                'vehicle_brake_control_enable': 1,
                'vehicle_brake_light_control': 1 if msg.actuation.brake_cmd > 0 else 0,
                'vehicle_brake_control': float(msg.actuation.brake_cmd),
                'parking_control': 0,
            }
            command_data_steering = {
                # only use front ackermann
                'vehicle_steering_control_enable': 1,
                'steering_mode_control': 0,
                'vehicle_steering_control_front': int(msg.actuation.steer_cmd),
            }

            self.drive_ctrl_data.update_value(**command_data_acc)
            self.brake_ctrl_data.update_value(**command_data_brake)
            self.wheel_ctrl_data.update_value(**command_data_steering)

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

        if (now - self.steer_ctrl_data.get_value('last_update_time')) > 300000000:
            self.steer_ctrl_data.reset_data()

        self.steer_ctrl_data.add_cycle_count()
        self.can_sender.send(0x132, self.steer_ctrl_data.get_bytearray())

    def vehicle_ctrl_data_timer_callback(self):
        now = time_ns()

        if (now - self.vehicle_ctrl_data.get_value('last_update_time')) > 500000000:
            self.vehicle_ctrl_data.reset_data()

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
