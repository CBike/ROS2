import can
import threading
import queue
import struct


class CanReceiver:
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
        self.throttle_report_message_queue = queue.Queue()
        self.brake_report_message_queue = queue.Queue()
        self.steer_report_message_queue = queue.Queue()
        self.gear_report_message_queue = queue.Queue()
        self.park_report_message_queue = queue.Queue()
        self.vcu_report_message_queue = queue.Queue()
        self.wheel_speed_report_message_queue = queue.Queue()
        self.bms_report_message_queue = queue.Queue()

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
                # TODO: Fields other than the dlc field of the can message frame must also be checked.
                if message.dlc == 8:
                    # Assuming LAW data has an extended ID and DLC of 8 bytes
                    self.process_can_date(can_id, data)
            except can.CanError as _:
                # TODO Can Exception handling
                pass

    def process_can_date(self, can_id, data):
        """
        Processes received CAN message based on their ID.

        Parameters:
        - can_id (int): CAN message ID.
        - data (bytearray): Raw data of the CAN message.
        """
        # Each CAN ID corresponds to a specific type of report, and data is parsed accordingly

        # throttle Report
        if can_id == 0x500:
            parsed_data = CanReceiver.parsing_can_data_throttle_report(data)
            self.throttle_report_message_queue.put(parsed_data)
        # Brake Report
        elif can_id == 0x501:
            parsed_data = CanReceiver.parsing_can_data_brake_report(data)
            self.brake_report_message_queue.put(parsed_data)

        # Steering Report
        elif can_id == 0x502:
            parsed_data = CanReceiver.parsing_can_data_steering_report(data)
            self.steer_report_message_queue.put(parsed_data)

        # Gear Report
        elif can_id == 0x503:
            parsed_data = CanReceiver.parsing_can_data_gear_report(data)
            self.gear_report_message_queue.put(parsed_data)

        # Park Report
        elif can_id == 0x504:
            parsed_data = CanReceiver.parsing_can_data_park_report(data)
            self.park_report_message_queue.put(parsed_data)

        # VCU Report
        elif can_id == 0x505:
            parsed_data = CanReceiver.parsing_can_data_vcu_report(data)
            self.vcu_report_message_queue.put(parsed_data)

        # WheelSpeed Report
        elif can_id == 0x506:
            parsed_data = CanReceiver.parsing_can_data_wheel_speed_report(data)
            self.wheel_speed_report_message_queue.put(parsed_data)

        # BMS Report
        elif can_id == 0x512:
            parsed_data = CanReceiver.parsing_can_data_bms_report(data)
            self.bms_report_message_queue.put(parsed_data)

        else:
            # TODO:  Exception handling for undefined ID values must be added.
            pass


    def get_throttle_report(self):
        if not self.throttle_report_message_queue.empty():
            return self.throttle_report_message_queue.get()
        else:
            return None

    def get_brake_report(self):
        if not self.brake_report_message_queue.empty():
            return self.brake_report_message_queue.get()
        else:
            return None

    def get_steer_report(self):
        if not self.steer_report_message_queue.empty():
            return self.steer_report_message_queue.get()
        else:
            return None

    def get_gear_report(self):
        if not self.steer_report_message_queue.empty():
            return self.gear_report_message_queue.get()
        else:
            return None

    def get_park_report(self):
        if not self.park_report_message_queue.empty():
            return self.park_report_message_queue.get()
        else:
            return None

    def get_vcu_report(self):
        if not self.vcu_report_message_queue.empty():
            return self.vcu_report_message_queue.get()
        else:
            return None

    def get_wheel_speed_report(self):
        if not self.wheel_speed_report_message_queue.empty():
            return self.wheel_speed_report_message_queue.get()
        else:
            return None

    def get_bms_report(self):
        if not self.bms_report_message_queue.empty():
            return self.bms_report_message_queue.get()
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

        throttle_en_state = data[0] & 0b00000011
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

        brake_en_state = data[0] & 0b00000011
        brake_flt1 = struct.unpack('>B', data[1:2])[0]
        brake_flt2 = struct.unpack('>B', data[2:3])[0]
        brake_pedal_actual = struct.unpack('>B', data[3:4])[0] * 0.1

        parsed_data = dict()
        parsed_data['brake_en_state'] = brake_en_state if is_valid_brake_en_state(brake_en_state) else 0
        parsed_data['brake_flt1'] = brake_flt1 if is_valid_brake_flt1(brake_flt1) else 0
        parsed_data['brake_flt2'] = brake_flt2 if is_valid_brake_flt2(brake_flt2) else 0
        parsed_data['brake_pedal_actual'] = brake_pedal_actual if is_valid_brake_pedal_actual(brake_pedal_actual) else 0

        return parsed_data

    @staticmethod
    def parsing_can_data_steering_report(data):

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

        steer_en_state = data[0] & 0b00000011
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

        gear_actual = data[0] & 0b00000111
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

        parking_actual = data[0] & 0b00000001
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

        steer_mode_stst = data[1] & 0b00000111
        brake_light_actual = 1 if ((data[1] & 0b00001000) > 0) else 0

        acc = ((struct.unpack('>h', data[1:3])[0] & 0b1111000000000000) >> 12 |
               struct.unpack('>h', data[1:3])[0] & 0b0000000111111111) * 0.001

        speed = struct.unpack('>h', data[3:5])[0] * 0.001
        aeb_state = data[5] & 0b00000001
        front_crash_state = data[5] & 0b00000010
        back_crash_state = data[5] & 0b00000100
        vehicle_mode_state = data[5] & 0b00011000
        driver_mode_state = data[5] & 0b1110000
        chassis_errcode = struct.unpack('>B', data[6:7])[0]
        turn_light_actual = data[7] & 0b00000011

        parsed_data = dict()
        parsed_data['steer_mode_stst'] = steer_mode_stst if is_valid_steer_mode_stst(steer_mode_stst) else 0
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
    can_listener = CanReceiver('can0')
    can_listener.start()
