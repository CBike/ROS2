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
            'throttle': queue.Queue(),
            'brake': queue.Queue(),
            'steer': queue.Queue(),
            'gear': queue.Queue(),
            'park': queue.Queue(),
            'vcu': queue.Queue(),
            'wheel_speed': queue.Queue(),
            'bms': queue.Queue(),
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
            parsed_data = getattr(self, f'parsing_can_data_{report_type}_report')(data)
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
            0x500: 'throttle',
            0x501: 'brake',
            0x502: 'steer',
            0x503: 'gear',
            0x504: 'park',
            0x505: 'vcu',
            0x506: 'wheel_speed',
            0x512: 'bms',
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
    def parsing_can_data_throttle_report(data):
        def is_valid_throttle_en_state(value):
            return 0 <= value <= 3

        def is_valid_throttle_flt1(value):
            return 0 <= value <= 1

        def is_valid_throttle_flt2(value):
            return 0 <= value <= 1

        def is_valid_throttle_pedal_actual(value):
            return 0 <= value <= 100

        throttle_en_state = struct.unpack('>B', data[0:1])[0] & 0b00000011
        throttle_flt1 = struct.unpack('>B', data[1:2])[0]
        throttle_flt2 = struct.unpack('>B', data[2:3])[0]
        throttle_pedal_actual = struct.unpack('>B', data[3:4])[0] * 0.1

        parsed_data = dict()
        parsed_data['throttle_en_state'] = throttle_en_state if is_valid_throttle_en_state(throttle_en_state) else 0
        parsed_data['throttle_flt1'] = throttle_flt1 if is_valid_throttle_flt1(throttle_flt1) else 0
        parsed_data['throttle_flt2'] = throttle_flt2 if is_valid_throttle_flt2(throttle_flt2) else 0
        parsed_data['throttle_pedal_actual'] = throttle_pedal_actual if is_valid_throttle_pedal_actual(
            throttle_pedal_actual) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_brake_report(data):
        def is_valid_brake_en_state(value):
            return 0 <= value <= 3

        def is_valid_brake_flt1(value):
            return 0 <= value <= 1

        def is_valid_brake_flt2(value):
            return 0 <= value <= 1

        def is_valid_brake_pedal_actual(value):
            return 0 <= value <= 100

        brake_en_state = struct.unpack('>B', data[0:1])[0] & 0b00000011
        brake_flt1 = struct.unpack('>B', data[1:2])[0]
        brake_flt2 = struct.unpack('>B', data[2:3])[0]
        brake_pedal_actual = float(struct.unpack('>H', data[3:5])[0] * 0.1)

        parsed_data = dict()
        parsed_data['brake_en_state'] = brake_en_state if is_valid_brake_en_state(brake_en_state) else 0
        parsed_data['brake_flt1'] = brake_flt1 if is_valid_brake_flt1(brake_flt1) else 0
        parsed_data['brake_flt2'] = brake_flt2 if is_valid_brake_flt2(brake_flt2) else 0
        parsed_data['brake_pedal_actual'] = brake_pedal_actual if is_valid_brake_pedal_actual(brake_pedal_actual) else 0.0

        return parsed_data

    @staticmethod
    def parsing_can_data_steer_report(data):

        def is_valid_steer_en_state(value):
            return 0 <= value <= 3

        def is_valid_steer_flt1(value):
            return 0 <= value <= 1

        def is_valid_steer_flt2(value):
            return 0 <= value <= 1

        def is_valid_steer_angle_actual(value):
            return -500 <= value <= 500

        def is_valid_steer_angle_speed_actual(value):
            return 0 <= value <= 255

        steer_en_state = struct.unpack('>B', data[0:1])[0] & 0b00000011
        steer_flt1 = struct.unpack('>B', data[1:2])[0]
        steer_flt2 = struct.unpack('>B', data[2:3])[0]
        steer_angle_actual = struct.unpack('>H', data[3:5])[0] - 500
        steer_angle_speed_actual = struct.unpack('>B', data[5:6])[0]

        parsed_data = dict()
        parsed_data['steer_en_state'] = steer_en_state if is_valid_steer_en_state(steer_en_state) else 0
        parsed_data['steer_flt1'] = steer_flt1 if is_valid_steer_flt1(steer_flt1) else 0
        parsed_data['steer_flt2'] = steer_flt2 if is_valid_steer_flt2(steer_flt2) else 0
        parsed_data['steer_angle_actual'] = steer_angle_actual if is_valid_steer_angle_actual(steer_angle_actual) else 0
        parsed_data['steer_angle_speed_actual'] = steer_angle_speed_actual if is_valid_steer_angle_speed_actual(
            steer_angle_speed_actual) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_gear_report(data):
        def is_valid_gear_actual(value):
            return 0 <= value <= 4

        def is_valid_gear_flt(value):
            return 0 <= value <= 1

        gear_actual = struct.unpack('>B', data[0:1])[0] & 0b00000111
        gear_flt = struct.unpack('>B', data[1:2])[0]

        parsed_data = dict()
        parsed_data['gear_actual'] = gear_actual if is_valid_gear_actual(gear_actual) else 0
        parsed_data['gear_flt'] = gear_flt if is_valid_gear_flt(gear_flt) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_park_report(data):
        def is_valid_parking_actual(value):
            return 0 <= value <= 1

        def is_valid_park_flt(value):
            return 0 <= value <= 1

        parking_actual = struct.unpack('>B', data[0:1])[0] & 0b00000001
        park_flt = struct.unpack('>B', data[1:2])[0]

        parsed_data = dict()
        parsed_data['parking_actual'] = parking_actual if is_valid_parking_actual(parking_actual) else 0
        parsed_data['park_flt'] = park_flt if is_valid_park_flt(park_flt) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_vcu_report(data):

        def is_valid_steer_mode_stst(value):
            return 0 <= value <= 2

        def is_valid_brake_light_actual(value):
            return 0 <= value <= 1

        def is_valid_acc(value):
            return -10 <= value <= 10

        def is_valid_speed(value):
            return -32.768 <= value <= 32.768

        def is_valid_aeb_state(value):
            return 0 <= value <= 1

        def is_valid_front_crash_state(value):
            return 0 <= value <= 1

        def is_valid_back_crash_state(value):
            return 0 <= value <= 1

        def is_valid_vehicle_mode_state(value):
            return 0 <= value <= 3

        def is_valid_driver_mode_state(value):
            return 0 <= value <= 7

        def is_valid_chassis_errcode(value):
            return 0 <= value <= 255

        def is_valid_turn_light_actual(value):
            return 0 <= value <= 4

        steer_mode_stst = struct.unpack('>B', data[1:2])[0] & 0b00000111

        brake_light_actual = 1 if ((struct.unpack('>B', data[1:2])[0] & 0b00001000) > 0) else 0

        acc = (struct.unpack('>b', data[0:1])[0] | struct.unpack('>b', data[1:2])[0] & 0b11110000) * 0.001

        speed = struct.unpack('>h', data[2:4])[0] * 0.001
        aeb_state = struct.unpack('>B', data[4:5])[0] & 0b00000001
        front_crash_state = struct.unpack('>B', data[4:5])[0] & 0b00000010
        back_crash_state = struct.unpack('>B', data[4:5])[0] & 0b00000100
        vehicle_mode_state = struct.unpack('>B', data[4:5])[0] & 0b00011000
        driver_mode_state = struct.unpack('>B', data[4:5])[0] & 0b11100000
        chassis_errcode = struct.unpack('>B', data[5:6])[0]
        turn_light_actual = struct.unpack('>B', data[7:8])[0] & 0b00000011

        parsed_data = dict()
        parsed_data['steer_mode_sts'] = steer_mode_stst if is_valid_steer_mode_stst(steer_mode_stst) else 0
        parsed_data['brake_light_actual'] = brake_light_actual if is_valid_brake_light_actual(brake_light_actual) else 0
        parsed_data['acc'] = acc if is_valid_acc(acc) else 0.0
        parsed_data['speed'] = speed if is_valid_speed(speed) else 0.0
        parsed_data['aeb_state'] = aeb_state if is_valid_aeb_state(aeb_state) else 0
        parsed_data['front_crash_state'] = front_crash_state if is_valid_front_crash_state(front_crash_state) else 0
        parsed_data['back_crash_state'] = back_crash_state if is_valid_back_crash_state(back_crash_state) else 0
        parsed_data['vehicle_mode_state'] = vehicle_mode_state if is_valid_vehicle_mode_state(vehicle_mode_state) else 0
        parsed_data['driver_mode_state'] = driver_mode_state if is_valid_driver_mode_state(driver_mode_state) else 0
        parsed_data['chassis_errcode'] = chassis_errcode if is_valid_chassis_errcode(chassis_errcode) else 0
        parsed_data['turn_light_actual'] = turn_light_actual if is_valid_turn_light_actual(turn_light_actual) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_wheel_speed_report(data):
        def is_valid_fl(value):
            return 0 <= value <= 65.535

        def is_valid_fr(value):
            return 0 <= value <= 65.535

        def is_valid_rl(value):
            return 0 <= value <= 65.535

        def is_valid_rr(value):
            return 0 <= value <= 65.535

        fl = struct.unpack('>H', data[0:2])[0] * 0.001
        fr = struct.unpack('>H', data[2:4])[0] * 0.001
        rl = struct.unpack('>H', data[4:6])[0] * 0.001
        rr = struct.unpack('>H', data[6:8])[0] * 0.001

        parsed_data = dict()
        parsed_data['fl'] = fl if is_valid_fl(fl) else 0.0
        parsed_data['fr'] = fr if is_valid_fr(fr) else 0.0
        parsed_data['rl'] = rl if is_valid_rl(rl) else 0.0
        parsed_data['rr'] = rr if is_valid_rr(rr) else 0.0

        return parsed_data

    @staticmethod
    def parsing_can_data_bms_report(data):
        def is_valid_battery_voltage(value):
            return 0 <= value <= 300

        def is_valid_battery_current(value):
            return -3200 <= value <= 3353.5

        def is_valid_battery_soc(value):
            return 0 <= value <= 100

        battery_voltage = struct.unpack('>H', data[0:2])[0] * 0.01
        battery_current = (struct.unpack('>H', data[2:4])[0] * 0.1) - 3200
        battery_soc = struct.unpack('>B', data[4:5])[0]

        parsed_data = dict()
        parsed_data['battery_voltage'] = battery_voltage if is_valid_battery_voltage(battery_voltage) else 0.0
        parsed_data['battery_current'] = battery_current if is_valid_battery_current(battery_current) else 0
        parsed_data['battery_soc'] = battery_soc if is_valid_battery_soc(battery_soc) else 0

        return parsed_data


if __name__ == '__main__':
    can_listener = CANReceiver('can0')
    can_listener.start()