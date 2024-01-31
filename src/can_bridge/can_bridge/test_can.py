import time
import can

channel = 'vcan0'
bus = can.interface.Bus(channel=channel, bustype='socketcan')
f = open('/home/avg/can_message/can_message_log .txt', 'r')
while True:
    time.sleep(0.02)
    line = f.readline()
    tokens = line.split()
    if len(tokens) >= 4:
        can_id = (f'0x{tokens[1]}')
        can_id = int(can_id, 0)
        index_of_dlc = tokens.index("[8]")
        data = tokens[index_of_dlc + 1:]
        all_data_bytes = ''.join(data)
        byte_array_data = bytearray.fromhex(all_data_bytes)
        message = can.Message(arbitration_id=can_id, data=byte_array_data, is_extended_id=False)
        bus.send(message)
    if not line:
        break

f.close()