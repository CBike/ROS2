import can
import threading
import queue
import struct


class CANReceiver:
    def __init__(self, channel='can0'):
        """
        Initializes a CanReceiver object.

        Parameters:
        - channel (str): CAN channel name, default is 'can0'.

        TODO : Code that runs a shell that opens the can interface and Code is needed to automatically find the can interface and pass it to the 'channel' parameter of the can.interface.Bus method.
        """
        self.channel = channel
        self.bus = can.interface.Bus(channel=self.channel, bustype='socketcan')
        self.running = False

        # Queue for storing received messages of different types
        self.report_message_queues = {
            'drive_sta_fb': queue.Queue(),
            'brake_sta_fb': queue.Queue(),
            'steer_sta_fb': queue.Queue(),
            'vehicle_work_sta_fb': queue.Queue(),
            'power_sta_fb': queue.Queue(),
            'vehicle_sta_fb': queue.Queue(),
            'vehicle_flt_sta_fb': queue.Queue(),
            'chassis_wheel_rpm_fb': queue.Queue(),
            'chassis_wheel_torque_fb': queue.Queue(),
        }

        # Thread for receiving CAN message in the background
        self.receive_thread = threading.Thread(target=self.receive_data)

    def start(self):
        """
        Starts the background thread for receiving CAN message.
        """
        self.running = True
        self.receive_thread.start()

    def stop(self):
        """
        Stops the background thread for receiving CAN message.
        """
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
                    self.process_can_data(can_id, data)
            except can.CanError as _:
                pass

    def process_can_data(self, can_id, data):
        """
        Processes received CAN message based on their ID.

        Parameters:
        - can_id (int): CAN message ID.
        - data (bytearray): Raw data of the CAN message.
        """
        report_type = self.get_report_type(can_id)
        if report_type:
            parsed_data = getattr(self, f'parsing_can_data_{report_type}')(data)
            self.report_message_queues[report_type].put(parsed_data)
        else:
            # TODO: Exception handling for undefined ID values must be added.
            pass

    def get_report_type(self, can_id):
        """
        Maps CAN ID to report types.

        Parameters:
        - can_id (int): CAN message ID.

        Returns:
        - str: Report type or None if not found.
        """
        report_types = {
            0x530: 'drive_sta_fb',
            0x531: 'brake_sta_fb',
            0x532: 'steer_sta_fb',
            0x534: 'vehicle_work_sta_fb',
            0x535: 'power_sta_fb',
            0x536: 'vehicle_sta_fb',
            0x537: 'vehicle_flt_sta_fb',
            0x539: 'chassis_wheel_rpm_fb',
            0x542: 'chassis_wheel_torque_fb',
        }
        return report_types.get(can_id)

    def get_report(self, report_type):
        """
        Gets the latest report of a specific type.

        Parameters:
        - report_type (str): Type of report.

        Returns:
        - dict: Parsed report data or None if the queue is empty.
        """
        if report_type in self.report_message_queues and not self.report_message_queues[report_type].empty():
            return self.report_message_queues[report_type].get()
        else:
            return None

    @staticmethod
    def parsing_can_data_drive_sta_fb(data):
        def validate_drive_en_state(val):
            return 0 <= val <= 1

        def validate_reminder_for_drive_control_out_of_bounds(val):
            return 0 <= val <= 1

        def validate_drive_mode_fb(val):
            return 0 <= val <= 3

        def validate_gear_status(val):
            return 0 <= val <= 3

        def validate_actual_speed_fb(val):
            return -50 <= val <= 50

        def validate_throttle_request_val_fb(val):
            return 0 <= val <= 100

        def validate_vehicle_accel(val):
            return -20 <= val <= 20

        def validate_vcu_cycle_count(val):
            return 0 <= val <= 15

        drive_en_state = struct.unpack('<B', data[0:1])[0] & 0b00000001
        reminder_for_drive_control_out_of_bounds = (struct.unpack('<B', data[0:1])[0] >> 1) & 0b00000001
        drive_mode_fb = (struct.unpack('<B', data[0:1])[0] >> 2) & 0b00000011
        gear_status = (struct.unpack('<B', data[0:1])[0] >> 4) & 0b00000011
        actual_speed_fb = struct.unpack('<H', data[1:3])[0] * 0.01

        throttle_request_val_fb = ((struct.unpack('<B', data[3:4])[0])
                                   | ((struct.unpack('<B', data[4:5])[0] & 0b00000011) << 8)) * 0.1

        vehicle_accel = struct.unpack('<h', data[5:7])[0] * 0.01
        vcu_cycle_count = struct.unpack('<B', data[7:8])[0] & 0b00001111

        parsed_data = dict()
        parsed_data['drive_en_state'] = drive_en_state \
            if validate_drive_en_state(drive_en_state) else 0

        parsed_data['reminder_for_drive_control_out_of_bounds'] = reminder_for_drive_control_out_of_bounds \
            if validate_reminder_for_drive_control_out_of_bounds(reminder_for_drive_control_out_of_bounds) else 0

        parsed_data['drive_mode_fb'] = drive_mode_fb \
            if validate_drive_mode_fb(drive_mode_fb) else 0

        parsed_data['gear_status'] = gear_status \
            if validate_gear_status(gear_status) else 0

        parsed_data['actual_speed_fb'] = actual_speed_fb \
            if validate_actual_speed_fb(actual_speed_fb) else 0.00

        parsed_data['throttle_request_val_fb'] = throttle_request_val_fb \
            if validate_throttle_request_val_fb(throttle_request_val_fb) else 0.0

        parsed_data['vehicle_accel'] = vehicle_accel \
            if validate_vehicle_accel(vehicle_accel) else 0.00

        parsed_data['vcu_cycle_count'] = vcu_cycle_count \
            if validate_vcu_cycle_count(vcu_cycle_count) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_brake_sta_fb(data):
        def validate_data_brake_en_state(val):
            return 0 <= val <= 1

        def validate_data_brake_light_en_state(val):
            return 0 <= val <= 1

        def validate_data_parking_state(val):
            return 0 <= val <= 3

        def validate_data_brake_pedal_val_fb(val):
            return 0 <= val <= 100

        def validate_data_brake_pressure_fb(val):
            return 0 <= val <= 100

        def validate_data_vcu_cycle_count(val):
            return 0 <= val <= 15

        brake_en_state = struct.unpack('<B', data[0:1])[0] & 0b00000001
        brake_light_en_state = (struct.unpack('<B', data[0:1])[0] >> 2) & 0b00000001
        parking_state = (struct.unpack('<B', data[0:1])[0] >> 4) & 0b00000011

        brake_pedal_val_fb = ((struct.unpack('<B', data[1:2])[0])
                              | ((struct.unpack('<B', data[2:3])[0] & 0b00000011) << 8)) * 0.1

        brake_pressure_fb = struct.unpack('<B', data[3:4])[0]
        vcu_cycle_count = struct.unpack('<B', data[6:7])[0] & 0b00001111

        parsed_data = dict()
        parsed_data['brake_en_state'] = brake_en_state if validate_data_brake_en_state(brake_en_state) else 0
        parsed_data['brake_light_en_state'] = brake_light_en_state if (
            validate_data_brake_light_en_state(brake_light_en_state)) else 0
        parsed_data['parking_state'] = parking_state if validate_data_parking_state(parking_state) else 0
        parsed_data['brake_pedal_val_fb'] = brake_pedal_val_fb if validate_data_brake_pedal_val_fb(brake_pedal_val_fb) \
            else 0.0
        parsed_data['brake_pressure_fb'] = brake_pressure_fb if validate_data_brake_pressure_fb(brake_pressure_fb) \
            else 0
        parsed_data['vcu_cycle_count'] = vcu_cycle_count if validate_data_vcu_cycle_count(vcu_cycle_count) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_steer_sta_fb(data):
        def validate_steering_en_state(val):
            return 0 <= val <= 1

        def validate_steering_control_out_of_bounds_reminder(val):
            return 0 <= val <= 1

        def validate_steering_mode_fb(val):
            return 0 <= val <= 4

        def validate_front_steering_angle_fb(val):
            return -500 <= val <= 500

        def validate_rear_steering_angle_fb(val):
            return -500 <= val <= 500

        def validate_set_steering_angle_speed_fb(val):
            return 0 <= val <= 500

        def validate_data_vcu_cycle_count(val):
            return 0 <= val <= 15

        steering_en_state = struct.unpack('<B', data[0:1])[0] & 0b00000001
        steering_control_out_of_bounds_reminder = (struct.unpack('<B', data[0:1])[0] >> 1) & 0b00000001
        steering_mode_fb = (struct.unpack('<B', data[0:1])[0] >> 4) & 0b00001111

        front_steering_angle_fb = struct.unpack('<h', data[1:3])[0]

        rear_steering_angle_fb = struct.unpack('<h', data[3:5])[0]

        set_steering_angle_speed_fb = struct.unpack('<B', data[5:6])[0] * 2
        vcu_cycle_count = struct.unpack('<B', data[6:7])[0] & 0b00001111

        parsed_data = dict()
        parsed_data['steering_en_state'] = steering_en_state if validate_steering_en_state(steering_en_state) else 0
        parsed_data['steering_control_out_of_bounds_reminder'] = steering_control_out_of_bounds_reminder \
            if validate_steering_control_out_of_bounds_reminder(steering_control_out_of_bounds_reminder) else 0
        parsed_data['steering_mode_fb'] = steering_mode_fb if validate_steering_mode_fb(steering_mode_fb) else 0
        parsed_data['front_steering_angle_fb'] = front_steering_angle_fb \
            if validate_front_steering_angle_fb(front_steering_angle_fb) else 0
        parsed_data['rear_steering_angle_fb'] = rear_steering_angle_fb \
            if validate_rear_steering_angle_fb(rear_steering_angle_fb) else 0
        parsed_data['set_steering_angle_speed_fb'] = set_steering_angle_speed_fb \
            if validate_set_steering_angle_speed_fb(set_steering_angle_speed_fb) else 0
        parsed_data['vcu_cycle_count'] = vcu_cycle_count \
            if validate_data_vcu_cycle_count(vcu_cycle_count) else 0
        return parsed_data

    @staticmethod
    def parsing_can_data_vehicle_work_sta_fb(data):
        def validate_driving_mode_fb(val):
            return 0 <= val <= 3

        def validate_vehicle_power_on_status_fb(val):
            return 0 <= val <= 3

        def validate_dc_working_status(val):
            return 0 <= val <= 2

        def validate_vehicle_speed_limit_status(val):
            return 0 <= val <= 1

        def validate_vehicle_speed_limit_val_fb(val):
            return 0 <= val <= 50

        def validate_low_voltage_battery_voltage(val):
            return 0 <= val <= 25

        def validate_emergency_stop_status_fb(val):
            return 0 <= val <= 3

        def validate_vehicle_power_battery(val):
            return 0 <= val <= 1

        def validate_vehicle_rear_crash_sensor_feedback(val):
            return 0 <= val <= 1

        def validate_vcu_cycle_count(val):
            return 0 <= val <= 15

        driving_mode_fb = struct.unpack('<B', data[0:1])[0] & 0b00000011
        vehicle_power_on_status_fb = (struct.unpack('<B', data[0:1])[0] >> 2) & 0b00000011
        dc_working_status = (struct.unpack('<B', data[0:1])[0] >> 4) & 0b00000011
        vehicle_speed_limit_status = struct.unpack('<B', data[1:2])[0] & 0b00000001
        vehicle_speed_limit_val_fb = struct.unpack('<H', data[2:4])[0] * 0.1
        low_voltage_battery_voltage = struct.unpack('<B', data[4:5])[0] * 0.1
        emergency_stop_status_fb = struct.unpack('<B', data[5:6])[0] & 0b00001111
        vehicle_power_battery = (struct.unpack('<B', data[5:6])[0] >> 4) & 0b00000001
        vehicle_rear_crash_sensor_feedback = (struct.unpack('<B', data[5:6])[0] >> 5) & 0b00000001
        vcu_cycle_count = struct.unpack('<B', data[6:7])[0] & 0b00001111
        vcu_checksum = struct.unpack('<B', data[7:8])[0]

        parsed_data = dict()
        parsed_data['driving_mode_fb'] = driving_mode_fb \
            if validate_driving_mode_fb(driving_mode_fb) else 0
        parsed_data['vehicle_power_on_status_fb'] = vehicle_power_on_status_fb \
            if validate_vehicle_power_on_status_fb(vehicle_power_on_status_fb) else 0
        parsed_data['dc_working_status'] = dc_working_status \
            if validate_dc_working_status(dc_working_status) else 0
        parsed_data['vehicle_speed_limit_status'] = vehicle_speed_limit_status \
            if validate_vehicle_speed_limit_status(vehicle_speed_limit_status) else 0.0
        parsed_data['vehicle_speed_limit_val_fb'] = vehicle_speed_limit_val_fb \
            if validate_vehicle_speed_limit_val_fb(vehicle_speed_limit_val_fb) else 0
        parsed_data['low_voltage_battery_voltage'] = low_voltage_battery_voltage \
            if validate_low_voltage_battery_voltage(low_voltage_battery_voltage) else 0.0
        parsed_data['emergency_stop_status_fb'] = emergency_stop_status_fb \
            if validate_emergency_stop_status_fb(emergency_stop_status_fb) else 0
        parsed_data['vehicle_power_battery'] = vehicle_power_battery \
            if validate_vehicle_power_battery(vehicle_power_battery) else 0
        parsed_data['vehicle_rear_crash_sensor_feedback'] = vehicle_rear_crash_sensor_feedback \
            if validate_vehicle_rear_crash_sensor_feedback(vehicle_rear_crash_sensor_feedback) else 0
        parsed_data['vcu_cycle_count'] = vcu_cycle_count \
            if validate_vcu_cycle_count(vcu_cycle_count) else 0
        parsed_data['vcu_checksum'] = vcu_checksum

        return parsed_data

    @staticmethod
    def parsing_can_data_power_sta_fb(data):
        def validate_vehicle_charge_status(val):
            return 0 <= val <= 2

        def validate_vehicle_power_battery_electricity_amount(val):
            return 0 <= val <= 100

        def validate_vehicle_power_battery_voltage(val):
            return 0 <= val <= 1000

        def validate_vehicle_power_battery_current(val):
            return -1000 <= val <= 1000

        def validate_bms_maximum_monomer_temperature(val):
            return -40 <= val <= 80

        vehicle_charge_status = (struct.unpack('<B', data[0:1])[0] >> 4) & 0b00000011
        vehicle_power_battery_electricity_amount = struct.unpack('<B', data[1:2])[0]
        vehicle_power_battery_voltage = (struct.unpack('<H', data[2:4])[0] * 0.1) - 1000
        vehicle_power_battery_current = (struct.unpack('<H', data[4:6])[0] * 0.1) - 40
        bms_maximum_monomer_temperature = struct.unpack('<B', data[6:7])[0]

        parsed_data = dict()
        parsed_data['vehicle_charge_status'] = vehicle_charge_status \
            if validate_vehicle_charge_status(vehicle_charge_status) else 0
        parsed_data['vehicle_power_battery_electricity_amount'] = vehicle_power_battery_electricity_amount \
            if validate_vehicle_power_battery_electricity_amount(vehicle_power_battery_electricity_amount) else 0
        parsed_data['vehicle_power_battery_voltage'] = vehicle_power_battery_voltage \
            if validate_vehicle_power_battery_voltage(vehicle_power_battery_voltage) else 0.0
        parsed_data['vehicle_power_battery_current'] = vehicle_power_battery_current \
            if validate_vehicle_power_battery_current(vehicle_power_battery_current) else 0.0
        parsed_data['bms_maximum_monomer_temperature'] = bms_maximum_monomer_temperature \
            if validate_bms_maximum_monomer_temperature(bms_maximum_monomer_temperature) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_vehicle_sta_fb(data):
        def validate_status_fb(val):
            return 0 <= val <= 1

        position_light_status_fb = struct.unpack('<B', data[0:1])[0] & 0b00000001
        low_light_status_fb = (struct.unpack('<B', data[0:1])[0] >> 1) & 0b00000001
        left_turning_light_status_fb = (struct.unpack('<B', data[0:1])[0] >> 2) & 0b00000001
        right_turning_light_status_fb = (struct.unpack('<B', data[0:1])[0] >> 3) & 0b00000001
        hazard_warning_light_switch_status = (struct.unpack('<B', data[0:1])[0] >> 6) & 0b00000001

        parsed_data = dict()
        parsed_data['position_light_status_fb'] = position_light_status_fb \
            if validate_status_fb(position_light_status_fb) else 0

        parsed_data['low_light_status_fb'] = low_light_status_fb \
            if validate_status_fb(low_light_status_fb) else 0

        parsed_data['left_turning_light_status_fb'] = left_turning_light_status_fb \
            if validate_status_fb(left_turning_light_status_fb) else 0

        parsed_data['right_turning_light_status_fb'] = right_turning_light_status_fb \
            if validate_status_fb(right_turning_light_status_fb) else 0

        parsed_data['hazard_warning_light_switch_status'] = hazard_warning_light_switch_status \
            if validate_status_fb(hazard_warning_light_switch_status) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_vehicle_flt_sta_fb(data):
        def validate_motor_system_overheating(val):
            return 0 <= val <= 1

        def validate_battery_system_overheating(val):
            return 0 <= val <= 1

        def validate_brake_system_overheating(val):
            return 0 <= val <= 1

        def validate_steering_system_overheating(val):
            return 0 <= val <= 1

        def validate_battery_voltage_is_too_low(val):
            return 0 <= val <= 1

        def validate_system_error(val):
            return 0 <= val <= 4

        def validate_brake_system_failure(val):
            return 0 <= val <= 4

        def validate_parking_system_failure(val):
            return 0 <= val <= 4

        def validate_front_steering_system_fault(val):
            return 0 <= val <= 4

        def validate_rear_steering_system_failure(val):
            return 0 <= val <= 4

        def validate_left_front_motor_system_failure(val):
            return 0 <= val <= 4

        def validate_right_front_motor_system_failure(val):
            return 0 <= val <= 4

        def validate_left_rear_motor_system_failure(val):
            return 0 <= val <= 4

        def validate_right_rear_motor_system_failure(val):
            return 0 <= val <= 4

        def validate_bms_system_failure(val):
            return 0 <= val <= 4

        def validate_dc_system_failure(val):
            return 0 <= val <= 4

        motor_system_overheating = struct.unpack('<B', data[0:1])[0]
        battery_system_overheating = struct.unpack('<B', data[0:1])[0]
        brake_system_overheating = struct.unpack('<B', data[0:1])[0]
        steering_system_overheating = struct.unpack('<B', data[0:1])[0]
        battery_voltage_is_too_low = struct.unpack('<B', data[0:1])[0]
        system_error = struct.unpack('<B', data[1:2])[0]
        brake_system_failure = struct.unpack('<B', data[1:2])[0]
        parking_system_failure = struct.unpack('<B', data[2:3])[0]
        front_steering_system_fault = struct.unpack('<B', data[2:3])[0]
        rear_steering_system_failure = struct.unpack('<B', data[3:4])[0]
        left_front_motor_system_failure = struct.unpack('<B', data[3:4])[0]
        right_front_motor_system_failure = struct.unpack('<B', data[4:5])[0]
        left_rear_motor_system_failure = struct.unpack('<B', data[4:5])[0]
        right_rear_motor_system_failure = struct.unpack('<B', data[5:6])[0]
        bms_system_failure = struct.unpack('<B', data[5:6])[0]
        dc_system_failure = struct.unpack('<B', data[6:7])[0]

        parsed_data = dict()
        parsed_data['motor_system_overheating'] = motor_system_overheating \
            if validate_motor_system_overheating(motor_system_overheating) else 0
        parsed_data['battery_system_overheating'] = battery_system_overheating \
            if validate_battery_system_overheating(battery_system_overheating) else 0
        parsed_data['brake_system_overheating'] = brake_system_overheating \
            if validate_brake_system_overheating(brake_system_overheating) else 0
        parsed_data['steering_system_overheating'] = steering_system_overheating \
            if validate_steering_system_overheating(steering_system_overheating) else 0
        parsed_data['battery_voltage_is_too_low'] = battery_voltage_is_too_low \
            if validate_battery_voltage_is_too_low(battery_voltage_is_too_low) else 0
        parsed_data['system_error'] = system_error \
            if validate_system_error(system_error) else 0
        parsed_data['brake_system_failure'] = brake_system_failure \
            if validate_brake_system_failure(brake_system_failure) else 0
        parsed_data['parking_system_failure'] = parking_system_failure \
            if validate_parking_system_failure(parking_system_failure) else 0
        parsed_data['front_steering_system_fault'] = front_steering_system_fault \
            if validate_front_steering_system_fault(front_steering_system_fault) else 0
        parsed_data['rear_steering_system_failure'] = rear_steering_system_failure \
            if validate_rear_steering_system_failure(rear_steering_system_failure) else 0
        parsed_data['left_front_motor_system_failure'] = left_front_motor_system_failure \
            if validate_left_front_motor_system_failure(left_front_motor_system_failure) else 0
        parsed_data['right_front_motor_system_failure'] = right_front_motor_system_failure \
            if validate_right_front_motor_system_failure(right_front_motor_system_failure) else 0
        parsed_data['left_rear_motor_system_failure'] = left_rear_motor_system_failure \
            if validate_left_rear_motor_system_failure(left_rear_motor_system_failure) else 0
        parsed_data['right_rear_motor_system_failure'] = right_rear_motor_system_failure \
            if validate_right_rear_motor_system_failure(right_rear_motor_system_failure) else 0
        parsed_data['bms_system_failure'] = bms_system_failure \
            if validate_bms_system_failure(bms_system_failure) else 0
        parsed_data['dc_system_failure'] = dc_system_failure \
            if validate_dc_system_failure(dc_system_failure) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_chassis_wheel_rpm_fb(data):
        def validate_left_front_wheel_speed(val):
            return -2000 <= val <= 2000

        def validate_right_front_wheel_speed(val):
            return -2000 <= val <= 2000

        def validate_left_rear_wheel_speed(val):
            return -2000 <= val <= 2000

        def validate_right_rear_wheel_speed(val):
            return -2000 <= val <= 2000

        left_front_wheel_speed = struct.unpack('<h', data[0:2])[0]
        right_front_wheel_speed = struct.unpack('<h', data[2:4])[0]
        left_rear_wheel_speed = struct.unpack('<h', data[4:6])[0]
        right_rear_wheel_speed = struct.unpack('<h', data[6:8])[0]

        parsed_data = dict()
        parsed_data['left_front_wheel_speed'] = left_front_wheel_speed \
            if validate_left_front_wheel_speed(left_front_wheel_speed) else 0
        parsed_data['right_front_wheel_speed'] = right_front_wheel_speed \
            if validate_right_front_wheel_speed(right_front_wheel_speed) else 0
        parsed_data['left_rear_wheel_speed'] = left_rear_wheel_speed \
            if validate_left_rear_wheel_speed(left_rear_wheel_speed) else 0
        parsed_data['right_rear_wheel_speed'] = right_rear_wheel_speed \
            if validate_right_rear_wheel_speed(right_rear_wheel_speed) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_chassis_wheel_torque_fb(data):

        def validate_left_front_wheel_torque(val):
            return -200 <= val <= 200

        def validate_right_front_wheel_torque(val):
            return -200 <= val <= 200

        def validate_left_rear_wheel_torque(val):
            return -200 <= val <= 200

        def validate_right_rear_wheel_torque(val):
            return -200 <= val <= 200

        left_front_wheel_torque = struct.unpack('<h', data[0:2])[0] * 0.1
        right_front_wheel_torque = struct.unpack('<h', data[2:4])[0] * 0.1
        left_rear_wheel_torque = struct.unpack('<h', data[4:6])[0] * 0.1
        right_rear_wheel_torque = struct.unpack('<h', data[6:8])[0] * 0.1

        parsed_data = dict()
        parsed_data['left_front_wheel_torque'] = left_front_wheel_torque \
            if validate_left_front_wheel_torque(left_front_wheel_torque) else 0
        parsed_data['right_front_wheel_torque'] = right_front_wheel_torque \
            if validate_right_front_wheel_torque(right_front_wheel_torque) else 0
        parsed_data['left_rear_wheel_torque'] = left_rear_wheel_torque \
            if validate_left_rear_wheel_torque(left_rear_wheel_torque) else 0
        parsed_data['right_rear_wheel_torque'] = right_rear_wheel_torque \
            if validate_right_rear_wheel_torque(right_rear_wheel_torque) else 0

        return parsed_data


if __name__ == '__main__':
    can_listener = CANReceiver('can0')
    can_listener.start()
