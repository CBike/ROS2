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
        Name(type) : value-Description  (Unit / Init / Factor / Offset) [range min,max]

        throttle_pedal_actual(float32, unsigned): - Throttle pedal pressure percentage(%/0/0.1/0)[0,100]
        throttle_flt2(int8, unsigned): 0-No fault 1-Drive System Communication Fault(-/0/1/0)[0,1]
        throttle_flt1(int8, unsigned): 0-No fault 1-Drive System Hardware Fault(-/0/1/0)[0,1]
        throttle_en_state(int8, unsigned): 0-Manual 1-Auto 2-Takeover 3-Standby (-/0/1/0)[0,3]
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
         Name(type) : value-Description  (Unit / Init / Factor / Offset) [range min,max]

         brake_pedal_actual(float32, unsigned): Brake pedal pressure percentage(%/0/0.1/0)[0,100]
         brake_flt2(int8, unsigned): 0x00-No fault 0x01-Drive system Communication fault(-/0/1/0)[0,1]
         brake_flt1(int8, unsigned): 0x00-No fault 0x01-Drive system hardware fault(-/0/1/0)[0,1]
         brake_en_state(int8, unsigned): 0x00-Manual 0x01-Auto 0x02-Takeover 0x03-Standby(-/0/1/0)[0,3]
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
        Name(type) : value-Description  (Unit / Init / Factor / Offset) [range min,max]

        steer_en_state(int8, unsigned): 0x00-Manual 0x01-Auto 0x02-Takeover 0x03-Standby (-/0/1/0)[0,3]
        steer_flt1(int8, unsigned): 0x00-No fault 0x01-Drive system hardware fault(-/0/1/0)[0,1]
        steer_flt2(int8, unsigned): 0x00-No fault 0x01-Drive system Communication fault(-/0/1/0)[0,1]
        steer_angle_actual(int32, unsigned): steering angle value(deg/0/1/-500)[-360,360]
        steer_angle_spd_actual(int32, unsigned): Steering rotation speed value((deg/s)/0/1/0)[0,255]
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
        Name(type) : value-Description  (Unit / Init / Factor / Offset) [range min,max]

        gear_flt(int8, unsigned): 0x00-No fault 0x01-Fault(-/0/1/0)[0,1]
        gear_actual(int8, unsigned): 0x00-INVALID 0x01-PARK 0x02-REVERSE 0x03-NEUTRAL 0x04-Drive(-/0/1/0)[0,4]
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
        Name(type) : value-Description  (Unit / Init / Factor / Offset) [range min,max]

        parking_actual(int8, unsigned):0x00-Release 0x01-Parking_trigger (-/0/1/0)[0,1]
        park_flt(int8, unsigned):0x00-No Fault 0x01-Fault (-/0/1/0)[0,1]
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
        Name(type) : value-Description  (Unit / Init / Factor / Offset) [range min,max]


        brake_light_actual(int8, unsigned): 0x00-BrakeLight_OFF 0x01-BrakeLight_ON (-/0/1/0)[0,1]
        turn_light_actual(int8, unsigned): 0x00-Turn lamp sts_OFF 0x01-Left_Turn lamp sts_ON 0x02-Right_Turn lamp sts_ON
                                           0x03-Hazard_Warning_lamp sts_ON (-/0/1/0) [0, 3]
        chassis_errcode(int8, unsigned): - (-/0/1/0) [0, 255]
        drive_mode_sts(int8, unsigned): 0x00-Throttle Paddle Drive Mode 0x01-Speed Drive Mode (-/0/1/0)[0,7]
        steer_mode_sts(int8, unsigned): 0x00-Standard 0x01-Non Direction 0x02-Sync Direction (-/0/1/0)[0,2]
        vehicle_mode_state(int8, unsigned): 0x00-Manual Remote 0x01-Auto 0x02-Emergency 0x03-Standby(-/0/1/0)[0, 3]
        front_crash_state(int8, unsigned): 0x00-No Event 0x01-Crash Event(-/0/1/0)[0,1]
        back_crash_state(int8, unsigned): 0x00-No Event 0x01-Crash Event(-/0/1/0)[0,1]
        aeb_state(int8, unsigned): 0x00-Inactive 0x01-Active(-/0/1/0)[0, 1]
        acc(float32, signed): Acceleration ((m/s^2)/0/0.01/0) [-10,10]
        speed(float32, signed): velocity ((m/s)/0/0.001/0) [-32.768,32.768]
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
        Name(type) : value-Description  (Unit / Init / Factor / Offset) [range min,max]

        rr(float32): rear right wheel speed (m/s/0/0.001/0) [0, 65.535]
        rl(float32): rear left wheel speed (m/s/0/0.001/0) [0, 65.535]
        fr(float32): front right wheel speed (m/s/0/0.001/0) [0, 65.535]
        fl(float32): front left wheel speed (m/s/0/0.001/0) [0, 65.535]
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
        Name(type) : value-Description  (Unit / Init / Factor / Offset) [range min,max]

        battery_current(float32): (V/0/0.01/0) [0,300]
        battery_voltage(float32): (A/0/0.1/-3200) [-3200, 3353.5]
        battery_soc(int32): (%/0/1/0)[0, 100]
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
