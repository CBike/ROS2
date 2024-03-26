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

        self.battery_rpt_publisher = self.create_timer(0.2, self.battery_rpt_timer_call_back)
        self.control_mode_rpt_publisher = self.create_timer(0.02, self.control_mode_rpt_timer_call_back)
        self.gear_rpt_publisher = self.create_timer(0.02, self.gear_rpt_timer_call_back)
        self.hazard_lights_rpt_publisher = self.create_timer(0.2, self.hazard_lights_rpt_timer_call_back)
        self.turn_indicators_rpt_publisher = self.create_timer(0.2, self.turn_indicators_rpt_timer_call_back)
        self.steering_rpt_publisher = self.create_timer(0.2, self.steering_rpt_timer_call_back)
        self.velocity_rpt_publisher = self.create_timer(0.2, self.velocity_rpt_timer_call_back)

    def battery_rpt_timer_call_back(self):
        """
        Publishes ThrottleReport messages based on parsed CAN data.
        """

        get_msg = self.can_receiver.get_report('battery_rpt')
        if get_msg is not None:
            pub_msg = BatteryStatus()
            pub_msg.energy_level = get_msg['energy_level']
            self.battery_rpt_publisher.publish(pub_msg)
        else:
            pass

    def control_mode_rpt_timer_call_back(self):
        """
         Publishes BrakeReport messages based on parsed CAN data.
               const uint8 NO_COMMAND = 0;
               const uint8 AUTONOMOUS = 1;
               const uint8 AUTONOMOUS_STEER_ONLY = 2;
               const uint8 AUTONOMOUS_VELOCITY_ONLY = 3;
               const uint8 MANUAL = 4;
               const uint8 DISENGAGED = 5;
               const uint8 NOT_READY = 6;
         """
        get_msg = self.can_receiver.get_report('control_mode_rpt')
        if get_msg is not None:
            pub_msg = ControlModeReport()
            pub_msg.mode = get_msg['mode']

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
        """
        get_msg = self.can_receiver.get_report('gear_rpt')
        if get_msg is not None:
            pub_msg = GearReport()
            pub_msg.report = get_msg['report']
            self.gear_rpt_publisher.publish(pub_msg)

    def hazard_lights_rpt_timer_call_back(self):
        """
        Publishes GearReport messages based on parsed CAN data.
            module HazardLightsReport_Constants {
              const uint8 DISABLE = 1;
              const uint8 ENABLE = 2;
            }
        """
        get_msg = self.can_receiver.get_report('hazard_lights_rpt')
        if get_msg is not None:
            pub_msg = HazardLightsReport()
            pub_msg.report = get_msg['report']
            self.hazard_lights_rpt_publisher.publish(pub_msg)

    def turn_indicators_rpt_timer_call_back(self):
        """
        Publishes ParkReport messages based on parsed CAN data.
            module TurnIndicatorsReport_Constants {
              const uint8 DISABLE = 1;
              const uint8 ENABLE_LEFT = 2;
              const uint8 ENABLE_RIGHT = 3;
            };
        """
        get_msg = self.can_receiver.get_report('turn_indicators_rpt')
        if get_msg is not None:
            pub_msg = TurnIndicatorsReport()
            pub_msg.report = get_msg['report']
            self.turn_indicators_rpt_publisher.publish(pub_msg)

    def steering_rpt_timer_call_back(self):
        """
        Publishes VcuReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('steering_rpt')
        if get_msg is not None:
            pub_msg = SteeringReport()
            pub_msg.steering_tire_angle = get_msg['steering_tire_angle']
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
        get_msg = self.can_receiver.get_report('velocity_rpt')
        if get_msg is not None:
            pub_msg = VelocityReport()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "velocity_rpt"
            pub_msg.longitudinal_velocity = get_msg['longitudinal_velocity']
            pub_msg.lateral_velocity = get_msg['lateral_velocity']
            pub_msg.heading_rate = get_msg['heading_rate']

            self.velocity_rpt_publisher.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
