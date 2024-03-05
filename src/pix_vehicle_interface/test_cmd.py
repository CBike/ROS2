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
        b_msg.brake_dec = 0.0
        b_msg.brake_pedal_target = 3.0

        g_msg = GearCommand()
        g_msg.header.stamp = self.get_clock().now().to_msg()
        g_msg.header.frame_id = ''
        g_msg.gear_en_ctrl = 1
        g_msg.gear_target = 4

        p_msg = ParkCommand()
        p_msg.header.stamp = self.get_clock().now().to_msg()
        p_msg.header.frame_id = ''
        p_msg.park_en_ctrl = 1
        p_msg.park_target = 1

        s_msg = SteeringCommand()
        s_msg.header.stamp = self.get_clock().now().to_msg()
        s_msg.header.frame_id = ''
        s_msg.steer_en_ctrl = 1
        s_msg.steer_angle_target = 100
        s_msg.steer_angle_spd = 0

        v_msg = VehicleModeCommand()
        v_msg.header.stamp = self.get_clock().now().to_msg()
        v_msg.header.frame_id = ''
        v_msg.turn_light_ctrl = 1
        v_msg.drive_mode_ctrl = 1
        v_msg.steer_mode_ctrl = 1

        self.throttle_control_publisher.publish(t_msg)
        self.brake_control_publisher.publish(b_msg)
        self.steer_control_publisher.publish(s_msg)
        self.gear_control_publisher.publish(g_msg)
        self.park_control_publisher.publish(p_msg)
        self.vcu_control_publisher.publish(v_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
