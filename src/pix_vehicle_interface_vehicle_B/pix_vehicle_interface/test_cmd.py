import time

from rclpy.node import Node
import rclpy

from pix_vehicle_msgs.msg import (DriveCtrl, BrakeCtrl, SteerCtrl, VehicleCtrl, WheelTorqueCtrl)


class TestControlNode(Node):

    def __init__(self):
        super().__init__('test_vehicle_cmd')

        self.timer = self.create_timer(0.02, self.timer_call_back)

        self.drive_ctrl_publisher = self.create_publisher(DriveCtrl, 'Vehicle/Pix/drive_ctrl_data', 10)
        self.brake_ctrl_publisher = self.create_publisher(BrakeCtrl, 'Vehicle/Pix/brake_ctrl_data', 10)
        self.steer_ctrl_publisher = self.create_publisher(SteerCtrl, 'Vehicle/Pix/steer_ctrl_data', 10)
        self.vehicle_ctrl_publisher = self.create_publisher(VehicleCtrl, 'Vehicle/Pix/vehicle_ctrl_data', 10)
        self.wheel_ctrl_publisher = self.create_publisher(WheelTorqueCtrl, 'Vehicle/Pix/wheel_ctrl_data', 10)

    def timer_call_back(self):
        self.get_logger().info('Control test')
        d_msg = DriveCtrl()
        d_msg.header.stamp = self.get_clock().now().to_msg()
        d_msg.header.frame_id = 'pix vehicle driving control'
        d_msg.vehicle_drive_control_enable = 0
        d_msg.drive_mode_control = 0
        d_msg.gear_control = 0
        d_msg.vehicle_speed_control = 0
        d_msg.vehicle_throttle_control = 0

        b_msg = BrakeCtrl()
        b_msg.header.stamp = self.get_clock().now().to_msg()
        b_msg.header.frame_id = 'pix vehicle brake control'
        b_msg.vehicle_brake_control_enable = 0
        b_msg.vehicle_brake_light_control = 0
        b_msg.vehicle_brake_control = 0
        b_msg.parking_control = 0

        s_msg = SteerCtrl()
        s_msg.header.stamp = self.get_clock().now().to_msg()
        s_msg.header.frame_id = 'pix vehicle steer control'
        s_msg.vehicle_steering_control_enable = 0
        s_msg.steering_mode_control = 0
        s_msg.vehicle_steering_control_front = 0
        s_msg.vehicle_steering_control_rear = 0
        s_msg.vehicle_steering_wheel_speed_control = 0

        v_msg = VehicleCtrl()
        v_msg.header.stamp = self.get_clock().now().to_msg()
        v_msg.header.frame_id = 'pix vehicle vehicle control'
        v_msg.position_light_control = 0
        v_msg.low_light_control = 0
        v_msg.left_turn_light_control = 0
        v_msg.right_turn_light_control = 0
        v_msg.speed_limit_control = 0
        v_msg.speed_limit = 0
        v_msg.check_mode_enable = 0

        wt_msg = WheelTorqueCtrl()
        wt_msg.header.stamp = self.get_clock().now().to_msg()
        wt_msg.header.frame_id = 'pix vehicle wheel torque control'
        wt_msg.left_front_motor_torque = 0
        wt_msg.right_front_motor_torque = 0
        wt_msg.left_rear_motor_torque = 0
        wt_msg.right_rear_motor_torque = 0

        self.drive_ctrl_publisher.publish(d_msg)
        self.brake_ctrl_publisher.publish(b_msg)
        self.steer_ctrl_publisher.publish(s_msg)
        self.vehicle_ctrl_publisher.publish(v_msg)
        self.wheel_ctrl_publisher.publish(wt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
