from rclpy.node import Node
from std_msgs.msg import Header
from can_utils.can_receiver import CANReceiver

from tier4_vehicle_msgs.msg import BatteryStatus
from autoware_auto_vehicle_msgs.msg import (ControlModeReport, GearReport, HazardLightsReport, TurnIndicatorsReport,
                                            SteeringReport, VelocityReport)

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

        self.battery_rpt_publisher = self.create_publisher(BatteryStatus, '/vehicle/status/battery_charge', 10)
        self.control_mode_rpt_publisher = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 10)
        self.gear_rpt_publisher = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10)

        self.hazard_lights_rpt_publisher = self.create_publisher(HazardLightsReport,
                                                                 '/vehicle/status/hazard_lights_status', 10)

        self.turn_indicators_rpt_publisher = self.create_publisher(TurnIndicatorsReport,
                                                                   '/vehicle/status/turn_indicators_status', 10)

        self.steering_rpt_publisher = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)

        self.velocity_rpt_publisher = self.create_publisher(VelocityReport, '/vehicle/status/velocity_Status', 10)

        self.battery_rpt_publisher_timer = self.create_timer(0.2, self.battery_rpt_timer_call_back)
        self.control_mode_rpt_publisher_timer = self.create_timer(0.02, self.control_mode_rpt_timer_call_back)
        self.gear_rpt_publisher_timer = self.create_timer(0.02, self.gear_rpt_timer_call_back)
        self.hazard_lights_rpt_publisher_timer = self.create_timer(0.2, self.hazard_lights_rpt_timer_call_back)
        self.turn_indicators_rpt_publisher_timer = self.create_timer(0.2, self.turn_indicators_rpt_timer_call_back)
        self.steering_rpt_publisher_timer = self.create_timer(0.2, self.steering_rpt_timer_call_back)
        self.velocity_rpt_publisher_timer = self.create_timer(0.2, self.velocity_rpt_timer_call_back)

    def battery_rpt_timer_call_back(self):
        """
        Publishes ThrottleReport messages based on parsed CAN data.
        """

        get_msg = self.can_receiver.get_report('power_sta_fb')
        if get_msg is not None:
            pub_msg = BatteryStatus()
            pub_msg.energy_level = float(get_msg['vehicle_power_battery_electricity_amount'])
            self.battery_rpt_publisher.publish(pub_msg)
        else:
            pass

    def control_mode_rpt_timer_call_back(self):
        """
         Publishes BrakeReport messages based on parsed CAN data.
               const uint8 NO_COMMAND = 0;
               const uint8 AUTONOMOUS = 1;
               const uint8 MANUAL = 4;
               const uint8 DISENGAGED = 5;
               const uint8 NOT_READY = 6;

               pix vehicle CAN data(driving_mode) Description: 0:standby, 1:self driving, 2:remote, 3:man
         """

        get_msg = self.can_receiver.get_report('vehicle_work_sta_fb')
        if get_msg is not None:
            pub_msg = ControlModeReport()
            if get_msg['driving_mode_fb'] == 2 or get_msg['driving_mode_fb'] == 3:
                pub_msg.mode = 4
            else:
                pub_msg.mode = get_msg['driving_mode_fb']

            self.control_mode_rpt_publisher.publish(pub_msg)

    def gear_rpt_timer_call_back(self):
        """
        Publishes SteerReport messages based on parsed CAN data.
            module GearReport_Constants {
              const uint8 NONE = 0;
              const uint8 NEUTRAL = 1;
              const uint8 DRIVE = 2;
              const uint8 REVERSE = 20;
              const uint8 PARK = 22;
            };
        pix vehicle CAN data(gear) Description: 0:default N, 1:D, 2:N, 3:R

        """
        get_msg = self.can_receiver.get_report('drive_sta_fb')
        if get_msg is not None:
            pub_msg = GearReport()
            # N and Default N
            if get_msg['gear_status'] == 0 or get_msg['gear_status'] == 2:
                pub_msg.report = 1
            # Drive
            elif get_msg['gear_status'] == 1:
                pub_msg.report = 2
            # Reverse
            elif get_msg['gear_status'] == 3:
                pub_msg.report = 20
            # TODO: Parking Data From Brake Report

            self.gear_rpt_publisher.publish(pub_msg)

    def hazard_lights_rpt_timer_call_back(self):
        """
        Publishes GearReport messages based on parsed CAN data.
            module HazardLightsReport_Constants {
              const uint8 DISABLE = 1;
              const uint8 ENABLE = 2;
            }
            pix vehicle CAN data(Hazard lights) Description: 0:off 1:on
        """
        get_msg = self.can_receiver.get_report('vehicle_sta_fb')
        if get_msg is not None:
            pub_msg = HazardLightsReport()
            pub_msg.report = get_msg['hazard_warning_light_switch_status'] + 1
            self.hazard_lights_rpt_publisher.publish(pub_msg)

    def turn_indicators_rpt_timer_call_back(self):
        """
        Publishes ParkReport messages based on parsed CAN data.
            module TurnIndicatorsReport_Constants {
              const uint8 DISABLE = 1;
              const uint8 ENABLE_LEFT = 2;
              const uint8 ENABLE_RIGHT = 3;
            };
            pix vehicle CAN data(turn_right) Description: 0:off 1:on
            pix vehicle CAN data(turn_left) Description: 0:off 1:on

        """
        get_msg = self.can_receiver.get_report('vehicle_sta_fb')
        if get_msg is not None:
            pub_msg = TurnIndicatorsReport()
            pub_msg.report = 0
            # Right
            if get_msg['right_turning_light_status_fb'] == 1:
                pub_msg.report = 3
            # Left
            elif get_msg['left_turning_light_status_fb'] == 1:
                pub_msg.report = 2

            self.turn_indicators_rpt_publisher.publish(pub_msg)

    def steering_rpt_timer_call_back(self):
        """
        Publishes VcuReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('steer_sta_fb')
        if get_msg is not None:
            pub_msg = SteeringReport()
            pub_msg.steering_tire_angle = float(get_msg['front_steering_angle_fb'])
            self.steering_rpt_publisher.publish(pub_msg)

    def velocity_rpt_timer_call_back(self):
        """
        Publishes WheelSpeedReport messages based on parsed CAN data.
            struct VelocityReport {
              std_msgs::msg::Header header;
              @default (value=0.0)
              float longitudinal_velocity;

              @default (value=0.0)
              float lateral_velocity;

              @default (value=0.0)
              float heading_rate;
              }
        """
        get_msg = self.can_receiver.get_report('drive_sta_fb')
        if get_msg is not None:
            pub_msg = VelocityReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "velocity_rpt"
            pub_msg.longitudinal_velocity = float(get_msg['actual_speed_fb'])
            pub_msg.lateral_velocity = 0.0
            pub_msg.heading_rate = 0.0

            self.velocity_rpt_publisher.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()