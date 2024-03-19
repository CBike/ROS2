import time

from rclpy.node import Node
import rclpy

from pix_vehicle_msgs.msg import (ThrottleCommand, BrakeCommand, GearCommand, ParkCommand, SteeringCommand,
                                  VehicleModeCommand)


class TestControlNode(Node):

    def __init__(self):
        super().__init__('test_vehicle_cmd')

        self.timer = self.create_timer(0.02, self.timer_call_back)

        self.throttle_control_publisher = self.create_publisher(ThrottleCommand, 'Vehicle/Pix/Throttle_Command', 10)
        self.brake_control_publisher = self.create_publisher(BrakeCommand, 'Vehicle/Pix/Brake_Command', 10)
        self.steer_control_publisher = self.create_publisher(SteeringCommand, 'Vehicle/Pix/Steer_Command', 10)
        self.gear_control_publisher = self.create_publisher(GearCommand, 'Vehicle/Pix/Gear_Command', 10)
        self.park_control_publisher = self.create_publisher(ParkCommand, 'Vehicle/Pix/Park_Command', 10)
        self.vcu_control_publisher = self.create_publisher(VehicleModeCommand, 'Vehicle/Pix/Vehicle_Mode_Command', 10)

    def timer_call_back(self):
        self.get_logger().info('Control test')
        t_msg = ThrottleCommand()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = ''
        t_msg.throttle_en_ctrl = 1
        t_msg.vel_target = 0.0
        t_msg.throttle_acc = 0.0
        t_msg.throttle_pedal_target = 0.0

        b_msg = BrakeCommand()
        b_msg.header.stamp = self.get_clock().now().to_msg()
        b_msg.header.frame_id = ''
        b_msg.aeb_en_ctrl = 1
        b_msg.brake_en_ctrl = 1
        b_msg.brake_dec = 0.1
        b_msg.brake_pedal_target = 100.0

        g_msg = GearCommand()
        g_msg.header.stamp = self.get_clock().now().to_msg()
        g_msg.header.frame_id = ''
        g_msg.gear_en_ctrl = 1
        # 0x00 INVALID , 0x01 PARK 0x02 REVERSE 0x03 NEUTRAL 0x04 DRIVE
        g_msg.gear_target = 4

        p_msg = ParkCommand()
        p_msg.header.stamp = self.get_clock().now().to_msg()
        p_msg.header.frame_id = ''
        p_msg.park_en_ctrl = 1
        p_msg.park_target = 3

        s_msg = SteeringCommand()
        s_msg.header.stamp = self.get_clock().now().to_msg()
        s_msg.header.frame_id = ''
        s_msg.steer_en_ctrl = 1
        s_msg.steer_angle_target = 500
        s_msg.steer_angle_spd = 125

        v_msg = VehicleModeCommand()
        v_msg.header.stamp = self.get_clock().now().to_msg()
        v_msg.header.frame_id = ''
        # [] [0|7] 0-off 1-left on 2-right on 3- hazard warning
        v_msg.turn_light_ctrl = 1
        # [] [0|7] 0-throttle_pedal_drive 1-speed_drive
        v_msg.drive_mode_ctrl = 1
        # [] [0|7] 0-standard_steer 1-non_direction_steer 2-sync_direction_steer
        v_msg.steer_mode_ctrl = 1

        self.throttle_control_publisher.publish(t_msg)
        self.brake_control_publisher.publish(b_msg)
        self.steer_control_publisher.publish(s_msg)
        self.gear_control_publisher.publish(g_msg)
        self.vcu_control_publisher.publish(v_msg)
        self.park_control_publisher.publish(p_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()