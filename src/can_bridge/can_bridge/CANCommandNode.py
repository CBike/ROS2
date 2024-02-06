from rclpy.node import Node
from CANSender import CANSender
import rclpy


class CanSenderNode(Node):
    def __init__(self):
        super().__init__('CANCommandNode')
        self.can_channel = 'can0'
        self.can_sender = CANSender(self.can_channel)
        self.send_report_throttle = self.create_publisher()

        self.Throttle_Command_timer = self.create_timer(0.02, '')
        self.Brake_Command_timer = self.create_timer(0.02, '')
        self.Steering_Command_timer = self.create_timer(0.02, '')
        self.Gear_Command_timer = self.create_timer(0.05, '')
        self.Park_Command_timer = self.create_timer(0.02, '')
        self.Vehicle_Mode_Command_timer = self.create_timer(0.1, '')

