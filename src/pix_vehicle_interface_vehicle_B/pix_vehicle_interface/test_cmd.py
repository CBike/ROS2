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
        d_msg.vehicle_drive_control_enable = 1
        # 0: speed ctrl mode 1: throttle ctrl mode 2: reserve1 3: reserve2
        d_msg.drive_mode_control = 1
        # 0: default N 1: D 2: N 3: R
        d_msg.gear_control = 1
        # float32 [m/s] [0|50]
        d_msg.vehicle_speed_control = 0.5
        # float32 [%] [0 | 100]
        d_msg.vehicle_throttle_control = 100.0

        b_msg = BrakeCtrl()
        b_msg.header.stamp = self.get_clock().now().to_msg()
        b_msg.header.frame_id = 'pix vehicle brake control'
        b_msg.vehicle_brake_control_enable = 1
        b_msg.vehicle_brake_light_control = 1
        # float32 [0|100]
        b_msg.vehicle_brake_control = 100.0
        # [0|2] 0:default 1:brake 2:release
        b_msg.parking_control = 0

        s_msg = SteerCtrl()
        s_msg.header.stamp = self.get_clock().now().to_msg()
        s_msg.header.frame_id = 'pix vehicle steer control'
        s_msg.vehicle_steering_control_enable = 1
        # [] [0|4] 0:front ackerman 1:same front and back 2:front different back 3:back ackerman 4:front back
        s_msg.steering_mode_control = 0
        # [deg] [-500|500] Front Steering motor steering angle control
        s_msg.vehicle_steering_control_front = 500
        # [deg] [-500|500] Rear Steering motor steering angle control
        s_msg.vehicle_steering_control_rear = 500
        # [deg/s] [0|500]
        s_msg.vehicle_steering_wheel_speed_control = 500

        v_msg = VehicleCtrl()
        v_msg.header.stamp = self.get_clock().now().to_msg()
        v_msg.header.frame_id = 'pix vehicle vehicle control'

        # [] [0|1] 0:off 1:on
        v_msg.position_light_control = 0

        # [] [0|1] 0:off 1:on
        v_msg.low_light_control = 0

        # [] [0|1] 0:off 1:on
        v_msg.left_turn_light_control = 0

        # [] [0|1] 0:off 1:on
        v_msg.right_turn_light_control = 0

        # [] [0|1] 0:default 1:limit
        v_msg.speed_limit_control = 0

        # [m/s] [1|20] Speed limit value, the maximum value does not exceed the speed limit value of the chassis software
        v_msg.speed_limit = 0

        # [] [0|1] (reserved) 0:enable 1:disable
        v_msg.check_mode_enable = 0

        wt_msg = WheelTorqueCtrl()
        wt_msg.header.stamp = self.get_clock().now().to_msg()
        wt_msg.header.frame_id = 'pix vehicle wheel torque control'

        # [Nm] [-200|200] torque control (reserved)
        wt_msg.left_front_motor_torque = 0

        # [Nm] [-200|200] torque control (reserved)
        wt_msg.right_front_motor_torque = 0

        # [Nm] [-200|200] torque control (reserved)
        wt_msg.left_rear_motor_torque = 0

        # [Nm] [-200|200] torque control (reserved)
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
