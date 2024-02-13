import can


class CANSender:
    def __init__(self, channel='can0'):
        self.channel = channel
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan')

    # def send_can_message(self, can_id, data, extended=False):
    #     message = can.Message(
    #         arbitration_id=can_id,
    #         data=data,
    #         is_extended_id=extended,
    #     )
    #     try:
    #         print(f"Sent CAN message: ID={can_id}, Data={data}")
    #     except can.CanError:
    #         print("Error sending CAN message")

    def close(self):
        self.bus.shutdown()
