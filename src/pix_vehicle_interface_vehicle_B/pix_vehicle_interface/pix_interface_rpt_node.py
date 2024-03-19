from rclpy.node import Node
from std_msgs.msg import Header
from can_utils.can_receiver import CANReceiver
from pix_vehicle_msgs.msg import (BmsReport, BrakeReport, GearReport, ParkReport, SteerReport,
                                  ThrottleReport, VcuReport, WheelSpeedReport)
import rclpy


class CANReceiverNode(Node):
    """
    Node responsible for receiving CAN messages and publishing them as ROS2 messages.

    TODO: Check publisher queue size.
    """

    def __init__(self):
        super().__init__('CANReportNode')
        self.can_channel = 'can0'
        self.can_receiver = CANReceiver(self.can_channel)
        self.can_receiver.start()

        self.throttle_report_publisher = self.create_publisher(ThrottleReport, 'pix_vehicle_report/throttle_report', 10)
        self.brake_report_publisher = self.create_publisher(BrakeReport, 'pix_vehicle_report/brake_report', 10)
        self.steer_report_publisher = self.create_publisher(SteerReport, 'pix_vehicle_report/steer_report', 10)
        self.gear_report_publisher = self.create_publisher(GearReport, 'pix_vehicle_report/gear_report', 10)
        self.park_report_publisher = self.create_publisher(ParkReport, 'pix_vehicle_report/park_report', 10)
        self.vcu_report_publisher = self.create_publisher(VcuReport, 'pix_vehicle_report/vcu_report', 10)
        self.wheel_speed_report_publisher = self.create_publisher(WheelSpeedReport, 'pix_vehicle_report/wheel_speed',
                                                                  10)
        self.bms_report_publisher = self.create_publisher(BmsReport, 'pix_vehicle_report/bms_report', 10)

        self.throttle_report_timer = self.create_timer(0.02, self.pub_throttle_report)
        self.brake_report_timer = self.create_timer(0.02, self.pub_brake_report)
        self.steering_report_timer = self.create_timer(0.02, self.pub_steer_report)
        self.gear_report_timer = self.create_timer(0.05, self.pub_gear_report)
        self.park_report_timer = self.create_timer(0.05, self.pub_park_report)
        self.vcu_report_timer = self.create_timer(0.02, self.pub_vcu_report)
        self.wheel_speed_report_timer = self.create_timer(0.02, self.pub_wheel_speed_report)
        self.bms_report_timer = self.create_timer(0.02, self.pub_bms_report)

    def pub_throttle_report(self):
        """
        Publishes ThrottleReport messages based on parsed CAN data.
        """

        get_msg = self.can_receiver.get_report('throttle')
        if get_msg is not None:
            pub_msg = ThrottleReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "vehicle_report_throttle"
            pub_msg.throttle_en_state = get_msg['throttle_en_state']
            pub_msg.throttle_flt1 = get_msg['throttle_flt1']
            pub_msg.throttle_flt2 = get_msg['throttle_flt2']
            pub_msg.throttle_pedal_actual = get_msg['throttle_pedal_actual']
            self.throttle_report_publisher.publish(pub_msg)
        else:
            pass

    def pub_brake_report(self):
        """
         Publishes BrakeReport messages based on parsed CAN data.
         """
        get_msg = self.can_receiver.get_report('brake')
        if get_msg is not None:
            pub_msg = BrakeReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "vehicle_report_brake"
            pub_msg.brake_en_state = get_msg['brake_en_state']
            pub_msg.brake_flt1 = get_msg['brake_flt1']
            pub_msg.brake_flt2 = get_msg['brake_flt2']
            pub_msg.brake_pedal_actual = get_msg['brake_pedal_actual']
            self.brake_report_publisher.publish(pub_msg)

        else:
            pass

    def pub_steer_report(self):
        """
        Publishes SteerReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('steer')
        if get_msg is not None:
            pub_msg = SteerReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "vehicle_report_steer"
            pub_msg.steer_en_state = get_msg['steer_en_state']
            pub_msg.steer_flt1 = get_msg['steer_flt1']
            pub_msg.steer_flt2 = get_msg['steer_flt2']
            pub_msg.steer_angle_actual = get_msg['steer_angle_actual']
            pub_msg.steer_angle_spd_actual = get_msg['steer_angle_speed_actual']

            self.steer_report_publisher.publish(pub_msg)

        else:
            pass

    def pub_gear_report(self):
        """
        Publishes GearReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('gear')
        if get_msg is not None:
            pub_msg = GearReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "vehicle_report_gear"
            pub_msg.gear_actual = get_msg['gear_actual']
            pub_msg.gear_flt = get_msg['gear_flt']
            self.gear_report_publisher.publish(pub_msg)

    def pub_park_report(self):
        """
        Publishes ParkReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('park')
        if get_msg is not None:
            pub_msg = ParkReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "vehicle_report_park"
            pub_msg.parking_actual = get_msg['parking_actual']
            pub_msg.park_flt = get_msg['park_flt']
            self.park_report_publisher.publish(pub_msg)

    def pub_vcu_report(self):
        """
        Publishes VcuReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('vcu')
        if get_msg is not None:
            pub_msg = VcuReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "vehicle_report_vcu"
            pub_msg.steer_mode_sts = get_msg['steer_mode_sts']
            pub_msg.brake_light_actual = get_msg['brake_light_actual']
            pub_msg.acc = get_msg['acc']
            pub_msg.speed = get_msg['speed']
            pub_msg.aeb_state = get_msg['aeb_state']
            pub_msg.front_crash_state = get_msg['front_crash_state']
            pub_msg.back_crash_state = get_msg['back_crash_state']
            pub_msg.vehicle_mode_state = get_msg['vehicle_mode_state']
            pub_msg.drive_mode_sts = get_msg['driver_mode_state']
            pub_msg.chassis_errcode = get_msg['chassis_errcode']
            pub_msg.turn_light_actual = get_msg['turn_light_actual']
            self.vcu_report_publisher.publish(pub_msg)

        else:
            pass

    def pub_wheel_speed_report(self):
        """
        Publishes WheelSpeedReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('wheel_speed')
        if get_msg is not None:
            pub_msg = WheelSpeedReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "vehicle_report_wheel_speed"
            pub_msg.fl = get_msg['fl']
            pub_msg.fr = get_msg['fr']
            pub_msg.rl = get_msg['rl']
            pub_msg.rr = get_msg['rr']
            self.wheel_speed_report_publisher.publish(pub_msg)

    def pub_bms_report(self):
        """
        Publishes BmsReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('bms')
        if get_msg is not None:
            pub_msg = BmsReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "vehicle_report_bms"
            pub_msg.battery_voltage = get_msg['battery_voltage']
            pub_msg.battery_current = get_msg['battery_current']
            pub_msg.battery_soc = get_msg['battery_soc']
            self.bms_report_publisher.publish(pub_msg)

        else:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
