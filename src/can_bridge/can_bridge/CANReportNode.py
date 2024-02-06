from rclpy.node import Node
from std_msgs.msg import Header
from CANReceiver import CANReceiver
from can_bridge_interfaces.msg import (BmsReport, BrakeReport, GearReport, ParkReport,  SteerReport,
                                       ThrottleReport, VcuReport, WheelSpeedReport)
import rclpy


class CANReceiverNode(Node):
    def __init__(self):
        super().__init__('CANReportNode')
        self.can_channel = 'can0'
        self.can_receiver = CANReceiver(self.can_channel)
        self.can_receiver.start()

        self.publishers = {
            'throttle': self.create_publisher(ThrottleReport, '/throttle_report', 10),
            'brake': self.create_publisher(BrakeReport, '/brake_report', 10),
            'steer': self.create_publisher(SteerReport, '/steer_report', 10),
            'gear': self.create_publisher(GearReport, '/gear_report', 10),
            'park': self.create_publisher(ParkReport, '/park_report', 10),
            'vcu': self.create_publisher(VcuReport, '/vcu_report', 10),
            'wheel_speed': self.create_publisher(WheelSpeedReport, '/wheel_speed', 10),
            'bms': self.create_publisher(BmsReport, '/bms_report', 10),
        }

        self.timers = {
            'throttle': self.create_timer(0.02, self.publish_throttle_report),
            'brake': self.create_timer(0.02, self.publish_brake_report),
            'steer': self.create_timer(0.02, self.publish_steer_report),
            'gear': self.create_timer(0.05, self.publish_gear_report),
            'park': self.create_timer(0.05, self.publish_park_report),
            'vcu': self.create_timer(0.02, self.publish_vcu_report),
            'wheel_speed': self.create_timer(0.02, self.publish_wheel_speed_report),
            'bms': self.create_timer(0.02, self.publish_bms_report),
        }

    def publish_throttle_report(self):
        self.publish_report('throttle', ThrottleReport,
                            ['throttle_en_state', 'throttle_flt1', 'throttle_flt2', 'throttle_pedal_actual'])

    def publish_brake_report(self):
        self.publish_report('brake', BrakeReport,
                            ['brake_en_state', 'brake_flt1', 'brake_flt2', 'brake_pedal_actual'])

    def publish_steer_report(self):
        self.publish_report('steer', SteerReport,
                            ['steer_en_state', 'steer_flt1', 'steer_flt2', 'steer_angle_actual',
                             'steer_angle_speed_actual'])

    def publish_gear_report(self):
        self.publish_report('gear', GearReport, ['gear_actual', 'gear_flt'])

    def publish_park_report(self):
        self.publish_report('park', ParkReport, ['parking_actual', 'park_flt'])

    def publish_vcu_report(self):
        self.publish_report('vcu', VcuReport,
                            ['steer_mode_stst', 'brake_light_actual', 'acc', 'speed', 'aeb_state', 'front_crash_state',
                             'back_crash_state', 'vehicle_mode_state', 'driver_mode_state', 'chassis_errcode',
                             'turn_light_actual'])

    def publish_wheel_speed_report(self):
        self.publish_report('wheel_speed', WheelSpeedReport, ['fl', 'fr', 'rl', 'rr'])

    def publish_bms_report(self):
        self.publish_report('bms', BmsReport, ['battery_voltage', 'battery_current', 'battery_soc'])

    def publish_report(self, report_type, msg_type, fields):
        get_msg = self.can_receiver.get_report(report_type)
        if get_msg is not None:
            pub_msg = msg_type()
            for field in fields:
                setattr(pub_msg, field, get_msg[field])
            self.publishers[report_type].publish(pub_msg)

    def destroy_node(self):
        self.can_receiver.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
