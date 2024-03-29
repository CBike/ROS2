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



if __name__ == '__main__':
    can_listener = CANReceiver('can0')
    can_listener.start()
