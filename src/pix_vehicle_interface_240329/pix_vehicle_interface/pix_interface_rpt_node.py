from rclpy.node import Node

from tier4_vehicle_msgs.msg import BatteryStatus
from autoware_auto_vehicle_msgs.msg import (ControlModeReport, GearReport, HazardLightsReport, TurnIndicatorsReport,
                                            SteeringReport, VelocityReport)
import can
import rclpy
import threading
from struct import unpack


class CANReceiverNode(Node):
    """
    Node responsible for receiving CAN messages and publishing them as ROS2 messages.

    TODO: Check publisher queue size.
    """

    def __init__(self):
        super().__init__('CANReportNode')

        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.running = False
        self.receive_thread = threading.Thread(target=self.receive_data)

        self.msg_obj_battery_rpt = BatteryStatus()
        self.msg_obj_control_mode_rpt = ControlModeReport()
        self.msg_obj_gear_rpt = GearReport()
        self.msg_obj_hazardLights_rpt = HazardLightsReport()
        self.msg_obj_indicators_rpt = TurnIndicatorsReport()
        self.msg_obj_steering_rpt = SteeringReport()
        self.msg_obj_velocity_rpt = VelocityReport()

        self.battery_rpt_publisher = self.create_publisher(BatteryStatus, '/vehicle/status/battery_charge', 10)
        self.control_mode_rpt_publisher = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 10)
        self.gear_rpt_publisher = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10)
        self.hazard_lights_rpt_publisher = self.create_publisher(HazardLightsReport,
                                                                 '/vehicle/status/hazard_lights_status', 10)
        self.turn_indicators_rpt_publisher = self.create_publisher(TurnIndicatorsReport,
                                                                   '/vehicle/status/turn_indicators_status', 10)
        self.steering_rpt_publisher = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        self.velocity_rpt_publisher = self.create_publisher(VelocityReport, '/vehicle/status/velocity_Status', 10)
        self.start()

    def start(self):
        self.running = True
        self.receive_thread.start()

    def stop(self):
        self.running = False
        self.receive_thread.join()

    def receive_data(self):
        """
        Continuously receives CAN message and processes them.
        """
        while self.running:
            try:
                message = self.bus.recv()
                can_id = message.arbitration_id
                data = message.data

                if message.is_error_frame:
                    pass
                # TODO : If the message contains an error frame, additional code for data processing is required.
                if message.is_remote_frame:
                    pass
                # TODO : Requires processing when the message contains a remote request frame
                if not message.is_extended_id and message.dlc == 8 and not message.is_remote_frame:
                    self.process_can_data_and_publish(can_id, data)
            except can.CanError as _:
                pass

    def process_can_data_and_publish(self, can_id, data):
        if can_id == 0x530:
            gear_status = (unpack('<B', data[0:1])[0] >> 4) & 0b00000011

            if gear_status == 0 or gear_status == 2:
                self.msg_obj_gear_rpt.report = 1
            elif gear_status == 1:
                self.msg_obj_gear_rpt.report = 2
            elif gear_status == 3:
                self.msg_obj_gear_rpt.report = 20
            self.msg_obj_velocity_rpt.header.stamp = self.get_clock().now().to_msg()
            self.msg_obj_velocity_rpt.header.frame_id = "velocity_rpt"
            self.msg_obj_velocity_rpt.longitudinal_velocity = float(unpack('<H', data[1:3])[0] * 0.01)
            self.msg_obj_velocity_rpt.lateral_velocity = float(0.00)
            self.msg_obj_velocity_rpt.heading_rate = float(0.0)

            self.gear_rpt_publisher.publish(self.msg_obj_gear_rpt)
            self.velocity_rpt_publisher.publish(self.msg_obj_velocity_rpt)

        elif can_id == 0x531:
            pass
        elif can_id == 0x532:
            self.msg_obj_steering_rpt.steering_tire_angle = float(unpack('<h', data[1:3])[0])
            self.steering_rpt_publisher.publish(self.msg_obj_steering_rpt)
        elif can_id == 0x534:

            control_mode = unpack('<B', data[0:1])[0] & 0b00000011

            if control_mode == 2 or control_mode == 3:
                self.msg_obj_control_mode_rpt.mode = 4
            else:
                self.msg_obj_control_mode_rpt.mode = control_mode

            self.control_mode_rpt_publisher.publish(self.msg_obj_control_mode_rpt)
        elif can_id == 0x535:
            self.msg_obj_battery_rpt.energy_level = float(unpack('<B', data[1:2])[0])
            self.battery_rpt_publisher.publish(self.msg_obj_battery_rpt)

        elif can_id == 0x536:

            left_turning_light_status = (unpack('<B', data[0:1])[0] >> 2) & 0b00000001
            right_turning_light_status = (unpack('<B', data[0:1])[0] >> 3) & 0b00000001
            self.msg_obj_hazardLights_rpt.report = ((unpack('<B', data[0:1])[0] >> 6) & 0b00000001) + 1

            if left_turning_light_status == 1:
                self.msg_obj_indicators_rpt.report = 2
            elif right_turning_light_status == 1:
                self.msg_obj_indicators_rpt.report = 3
            else:
                self.msg_obj_indicators_rpt.report = 0

            self.turn_indicators_rpt_publisher.publish(self.msg_obj_indicators_rpt)
            self.hazard_lights_rpt_publisher.publish(self.msg_obj_hazardLights_rpt)
        elif can_id == 0x537:
            pass
        elif can_id == 0x539:
            pass
        elif can_id == 0x542:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
