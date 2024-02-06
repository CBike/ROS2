from rclpy.node import Node
from CANSender import CANSender
from std_msgs.msg import String
import rclpy


class CanSenderNode(Node):

    def __init__(self):
        super().__init__('CANCommandNode')
        self.can_channel = 'can0'
        self.can_sender = CANSender(self.can_channel)
        self.sub_throttle_cmd = self.create_subscription(String, 'actuator/Throttle_Command', self.listener_callback, 10)
        self.sub_brake_cmd = self.create_subscription(String, 'actuator/Brake_Command', self.listener_callback, 10)
        self.sub_steer_cmd = self.create_subscription(String, 'actuator/Steer_Command', self.listener_callback, 10)
        self.sub_gear_cmd = self.create_subscription(String, 'actuator/Gear_Command', self.listener_callback, 10)
        self.sub_park_cmd = self.create_subscription(String, 'actuator/Park_Command', self.listener_callback, 10)
        self.sub_vmc_cmd = self.create_subscription(String, 'actuator/Vehicle_Mode_Command', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

    def send_command(self):
        pass

    def parsing_command(self):
        pass


def main(args=None):
    pass

if __name__ == '__main__':
    pass

