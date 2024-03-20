from rclpy.node import Node
from std_msgs.msg import Header
from can_utils.can_receiver import CANReceiver
from pix_vehicle_msgs.msg import (V2ABrakeStaFb, V2AChassisWheelRpmFb, V2AChassisWheelTorqueFb, V2ADriveStaFb,
                                  V2APowerStaFb, V2ASteerStaFb, V2AVehicleFltStaFb, V2AVehicleStaFb,
                                  V2AVehicleWorkStaFb)
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

        self.drive_sta_fb_publisher = self.create_publisher(V2ADriveStaFb, 'pix_vehicle_report/drive_sta_fb', 10)
        self.brake_sta_fb_publisher = self.create_publisher(V2ABrakeStaFb, 'pix_vehicle_report/brake_sta_fb', 10)
        self.steer_sta_fb_publisher = self.create_publisher(V2ASteerStaFb, 'pix_vehicle_report/steer_sta_fb', 10)

        self.vehicle_work_sta_fb_publisher = self.create_publisher(V2AVehicleWorkStaFb,
                                                                   'pix_vehicle_report/vehicle_work_sta_fb', 10)

        self.power_sta_fb_publisher = self.create_publisher(V2APowerStaFb, 'pix_vehicle_report/power_sta_fb', 10)

        self.vehicle_sta_fb_publisher = self.create_publisher(V2AVehicleStaFb, 'pix_vehicle_report/vehicle_sta_fb', 10)

        self.vehicle_flt_sta_fb_publisher = self.create_publisher(V2AVehicleFltStaFb,
                                                                  'pix_vehicle_report/vehicle_flt_sta_fb', 10)

        self.chassis_wheel_rpm_fb_publisher = self.create_publisher(V2AChassisWheelRpmFb,
                                                                    'pix_vehicle_report/chassis_wheel_rpm_fb', 10)

        self.chassis_wheel_torque_fb_publisher = self.create_publisher(V2AChassisWheelTorqueFb,
                                                                       'pix_vehicle_report/chassis_wheel_torque_fb', 10)

        self.drive_sta_fb_timer = self.create_timer(0.02, self.drive_sta_fb_timer_call_back)
        self.brake_sta_fb_timer = self.create_timer(0.02, self.brake_sta_fb_timer_call_back)
        self.steer_sta_fb_timer = self.create_timer(0.02, self.steer_sta_fb_timer_call_back)
        self.vehicle_work_sta_fb_timer = self.create_timer(0.2, self.vehicle_work_sta_fb_timer_call_back)
        self.power_sta_fb_timer = self.create_timer(0.2, self.power_sta_fb_timer_call_back)
        self.vehicle_sta_fb_timer = self.create_timer(0.2, self.vehicle_sta_fb_timer_call_back)
        self.vehicle_flt_sta_fb_timer = self.create_timer(0.2, self.vehicle_flt_sta_fb_timer_call_back)
        self.chassis_wheel_rpm_fb_timer = self.create_timer(0.02, self.chassis_wheel_rpm_fb_timer_call_back)
        self.chassis_wheel_torque_fb_timer = self.create_timer(0.02, self.chassis_wheel_torque_fb_timer_call_back)

    def drive_sta_fb_timer_call_back(self):
        """
        Publishes ThrottleReport messages based on parsed CAN data.
        """

        get_msg = self.can_receiver.get_report('drive_sta_fb')
        if get_msg is not None:
            pub_msg = V2ADriveStaFb()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "report_drive_sta_fb"

            pub_msg.drive_en_state = get_msg['drive_en_state']
            pub_msg.reminder_for_drive_control_out_of_bounds = get_msg['reminder_for_drive_control_out_of_bounds']
            pub_msg.drive_mode_fb = get_msg['drive_mode_fb']
            pub_msg.gear_status = get_msg['gear_status']
            pub_msg.actual_speed_fb = get_msg['actual_speed_fb']
            pub_msg.throttle_request_val_fb = get_msg['throttle_request_val_fb']
            pub_msg.vehicle_accel = get_msg['vehicle_accel']
            pub_msg.vcu_cycle_count = get_msg['vcu_cycle_count']

            self.drive_sta_fb_publisher.publish(pub_msg)
        else:
            pass

    def brake_sta_fb_timer_call_back(self):
        """
         Publishes BrakeReport messages based on parsed CAN data.
         """
        get_msg = self.can_receiver.get_report('brake_sta_fb')
        if get_msg is not None:
            pub_msg = V2ABrakeStaFb()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "report_brake_sta_fb"
            pub_msg.brake_en_state = get_msg['brake_en_state']
            pub_msg.brake_light_en_state = get_msg['brake_light_en_state']
            pub_msg.parking_state = get_msg['parking_state']
            pub_msg.brake_pedal_val_fb = get_msg['brake_pedal_val_fb']
            pub_msg.brake_pressure_fb = get_msg['brake_pressure_fb']
            pub_msg.vcu_cycle_count = get_msg['vcu_cycle_count']
            self.brake_sta_fb_publisher.publish(pub_msg)

    def steer_sta_fb_timer_call_back(self):
        """
        Publishes SteerReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('steer_sta_fb')
        if get_msg is not None:
            pub_msg = V2ASteerStaFb()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "report_steer_sta_fb"
            pub_msg.steering_en_state = get_msg['steering_en_state']
            pub_msg.steering_control_out_of_bounds_reminder = get_msg['steering_control_out_of_bounds_reminder']
            pub_msg.steering_mode_fb = get_msg['steering_mode_fb']
            pub_msg.front_steering_angle_fb = get_msg['front_steering_angle_fb']
            pub_msg.rear_steering_angle_fb = get_msg['rear_steering_angle_fb']
            pub_msg.set_steering_angle_speed_fb = get_msg['set_steering_angle_speed_fb']
            pub_msg.vcu_cycle_count = get_msg['vcu_cycle_count']

            self.steer_sta_fb_publisher.publish(pub_msg)

    def vehicle_work_sta_fb_timer_call_back(self):
        """
        Publishes GearReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('vehicle_work_sta_fb')
        if get_msg is not None:
            pub_msg = V2AVehicleWorkStaFb()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "report_vehicle_work_sta_fb"
            pub_msg.drive_mode_fb = get_msg['drive_mode_fb']
            pub_msg.vehicle_power_on_status_fb = get_msg['vehicle_power_on_status_fb']
            pub_msg.dc_working_status = get_msg['dc_working_status']
            pub_msg.vehicle_speed_limit_status = get_msg['vehicle_speed_limit_status']
            pub_msg.vehicle_speed_limit_val_fb = get_msg['vehicle_speed_limit_val_fb']
            pub_msg.low_voltage_battery_voltage = get_msg['low_voltage_battery_voltage']
            pub_msg.emergency_stop_status_fb = get_msg['emergency_stop_status_fb']
            pub_msg.vehicle_power_battery = get_msg['vehicle_power_battery']
            pub_msg.vehicle_rear_crash_sensor_feedback = get_msg['vehicle_rear_crash_sensor_feedback']
            pub_msg.vcu_cycle_count = get_msg['vcu_cycle_count']
            pub_msg.vcu_checksum = get_msg['vcu_checksum']
            self.vehicle_work_sta_fb_publisher.publish(pub_msg)

    def power_sta_fb_timer_call_back(self):
        """
        Publishes ParkReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('power_sta_fb')
        if get_msg is not None:
            pub_msg = V2APowerStaFb()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "report_power_sta_fb"
            pub_msg.vehicle_charge_status = get_msg['vehicle_charge_status']
            pub_msg.vehicle_power_battery_electricity_amount = get_msg['vehicle_power_battery_electricity_amount']
            pub_msg.vehicle_power_battery_voltage = get_msg['vehicle_power_battery_voltage']
            pub_msg.vehicle_power_battery_current = get_msg['vehicle_power_battery_current']
            pub_msg.bms_maximum_monomer_temperature = get_msg['bms_maximum_monomer_temperature']
            self.power_sta_fb_publisher.publish(pub_msg)

    def vehicle_sta_fb_timer_call_back(self):
        """
        Publishes VcuReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('vehicle_sta_fb')
        if get_msg is not None:
            pub_msg = V2AVehicleStaFb()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "report_vehicle_sta_fb"
            pub_msg.position_light_status_fb = get_msg['position_light_status_fb']
            pub_msg.low_light_status_fb = get_msg['low_light_status_fb']
            pub_msg.left_turning_light_status_fb = get_msg['left_turning_light_status_fb']
            pub_msg.light_turning_light_status_fb = get_msg['light_turning_light_status_fb']
            pub_msg.hazard_warning_light_switch_status = get_msg['hazard_warning_light_switch_status']
            self.vehicle_sta_fb_publisher.publish(pub_msg)

    def vehicle_flt_sta_fb_timer_call_back(self):
        """
        Publishes WheelSpeedReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('vehicle_flt_sta_fb')
        if get_msg is not None:
            pub_msg = V2AVehicleFltStaFb()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "report_vehicle_flt_sta_fb"
            pub_msg.motor_system_overheating = get_msg['motor_system_overheating']
            pub_msg.battery_system_overheating = get_msg['battery_system_overheating']
            pub_msg.brake_system_overheating = get_msg['brake_system_overheating']
            pub_msg.steering_system_overheating = get_msg['steering_system_overheating']
            pub_msg.battery_voltage_is_too_low = get_msg['battery_voltage_is_too_low']
            pub_msg.system_error = get_msg['system_error']
            pub_msg.brake_system_failure = get_msg['brake_system_failure']
            pub_msg.parking_system_failure = get_msg['parking_system_failure']
            pub_msg.front_steering_system_fault = get_msg['front_steering_system_fault']
            pub_msg.rear_steering_system_failure = get_msg['rear_steering_system_failure']
            pub_msg.left_front_motor_system_failure = get_msg['left_front_motor_system_failure']
            pub_msg.right_front_motor_system_failure = get_msg['right_front_motor_system_failure']
            pub_msg.left_rear_motor_system_failure = get_msg['left_rear_motor_system_failure']
            pub_msg.right_rear_motor_system_failure = get_msg['right_rear_motor_system_failure']
            pub_msg.bms_system_failure = get_msg['bms_system_failure']
            pub_msg.dc_system_failure = get_msg['dc_system_failure']
            self.vehicle_flt_sta_fb_publisher.publish(pub_msg)

    def chassis_wheel_rpm_fb_timer_call_back(self):
        """
        Publishes BmsReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('chassis_wheel_rpm_fb')
        if get_msg is not None:
            pub_msg = V2AChassisWheelRpmFb()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.left_front_wheel_speed = get_msg['left_front_wheel_speed']
            pub_msg.right_front_wheel_speed = get_msg['right_front_wheel_speed']
            pub_msg.left_rear_wheel_speed = get_msg['left_rear_wheel_speed']
            pub_msg.right_rear_wheel_speed = get_msg['right_rear_wheel_speed']
            pub_msg.header.frame_id = "report_chassis_wheel_rpm_fb"

            self.chassis_wheel_rpm_fb_publisher.publish(pub_msg)

    def chassis_wheel_torque_fb_timer_call_back(self):
        """
        Publishes BmsReport messages based on parsed CAN data.
        """
        get_msg = self.can_receiver.get_report('chassis_wheel_torque_fb')
        if get_msg is not None:
            pub_msg = V2AChassisWheelTorqueFb()
            pub_msg.header = Header()
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "report_chassis_wheel_torque_fb"
            pub_msg.left_front_wheel_torque = get_msg['left_front_wheel_torque']
            pub_msg.right_front_wheel_torque = get_msg['right_front_wheel_torque']
            pub_msg.left_rear_wheel_torque = get_msg['left_rear_wheel_torque']
            pub_msg.right_rear_wheel_torque = get_msg['right_rear_wheel_torque']
            self.chassis_wheel_torque_fb_publisher.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
