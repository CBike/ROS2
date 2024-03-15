def generate_byte_array(array_size: int, *args) -> bytearray:
    """Generates a bytearray filled with data extracted from arguments within specified bit ranges.

    Args:
        array_size (int): The size of the bytearray to be generated.
        *args: Variable length argument list. Each argument should be a tuple containing:
            - The data value (int).
            - The starting bit index (inclusive) of the data within the generated bytearray.
            - The ending bit index (inclusive) of the data within the generated bytearray.

    Returns:
        bytearray: The generated bytearray filled with data extracted from arguments within specified bit ranges.

    This static method generates a bytearray with the specified size and fills it with data extracted
    from the provided arguments, where each argument represents a piece of data to be inserted
    into the bytearray. The data is inserted into the specified bit range within the bytearray.
    """
    byte_array = bytearray(array_size)

    for arg in args:
        data = int(arg[0])
        start_bit = int(arg[1])
        end_bit = int(arg[2])

        # Calculate start_byte, start_bit_offset, end_byte, end_bit_offset
        start_byte, start_bit_offset = divmod(start_bit, 8)
        end_byte, end_bit_offset = divmod(end_bit, 8)

        for byte_offset in range(start_byte, end_byte+1,):
            byte_value = 0
            # Iterate through each bit in the byte
            for bit_offset in range(8):
                # Calculate the bit index in the byte array
                current_bit = byte_offset * 8 + bit_offset

                # Check if the current bit falls within the specified range
                if start_bit <= current_bit <= end_bit:
                    # Extract the bit value from the data and set it in the byte_value
                    bit_value = (data >> (current_bit - start_bit)) & 1
                    byte_value |= bit_value << bit_offset

            # Store the byte_value in the byte array
            byte_array[byte_offset] |= byte_value

    return byte_array


def set_data_in_byte_array(byte_array: bytearray, data: int, start_bit: int, end_bit: int):
    """Sets the data in the specified bit range within the bytearray using big-endian byte order.

    Args:
        byte_array (bytearray): The bytearray to modify.
        data (int): The data value to insert into the specified bit range.
        start_bit (int): The starting bit index (inclusive) within the bytearray.
        end_bit (int): The ending bit index (inclusive) within the bytearray.
    """
    # Calculate start and end byte indices
    start_byte, start_bit_offset = divmod(start_bit, 8)
    end_byte, end_bit_offset = divmod(end_bit, 8)

    # Calculate the number of bits to fill in the first byte
    bits_in_first_byte = min(8 - start_bit_offset, end_bit - start_bit + 1)

    # Calculate the mask for the bits in the first byte
    first_byte_mask = ((1 << bits_in_first_byte) - 1) << (8 - start_bit_offset - bits_in_first_byte)

    # Fill in the bits in the first byte
    byte_array[start_byte] &= ~first_byte_mask  # Clear the bits to be set
    byte_array[start_byte] |= (data >> (end_bit - start_bit + 1 - bits_in_first_byte)) & first_byte_mask

    # Fill in the bits in the middle bytes
    for byte_index in range(start_byte + 1, end_byte):
        byte_array[byte_index] = (data >> (end_bit - start_bit - (byte_index - start_byte) * 8)) & 0xFF

    # Fill in the bits in the last byte
    if start_byte != end_byte:
        last_byte_bits = (end_bit - start_bit + 1) % 8
        last_byte_mask = (1 << last_byte_bits) - 1
        byte_array[end_byte] &= ~last_byte_mask  # Clear the bits to be set
        byte_array[end_byte] |= (data & last_byte_mask) << (8 - last_byte_bits)



def print_byte_array(byte_array):
    hex_string = ' '.join(format(byte, '02x') for byte in byte_array)
    ascii_string = ''.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in byte_array)

    for i in range(0, len(byte_array), 16):
        chunk_hex = hex_string[i:i + 32]
        chunk_ascii = ascii_string[i:i + 16]
        print(f"{chunk_hex.ljust(48)} {chunk_ascii}")


if __name__ == '__main__':
    throttle_en_ctrl = (1, 0, 0)

    throttle_acc = 1.0 / 0.01

    throttle_acc_upper_byte = (((int(throttle_acc) >> 2) & 0xFF), 8, 15)
    throttle_acc_lower_byte = ((int(throttle_acc) & 0b11), 22, 33)

    throttle_pedal_target = 10.0 / 0.1
    throttle_pedal_target_upper_byte = (((int(throttle_pedal_target) >> 8) & 0xFF), 24, 31)
    throttle_pedal_target_lower_byte = ((int(throttle_pedal_target) & 0xFF), 32, 39)

    vel_target = 10.23 / 0.01
    throttle_vel_target_upper_byte = (((int(vel_target) >> 2) & 0xFF), 40, 47)
    throttle_vel_target_lower_byte = ((int(vel_target) & 0b11), 54, 55)

    steer_en_ctrl = (1, 0, 0)
    steer_angle_spd = (125, 8, 15)
    steer_angle_target_upper_data = ((((0 + 500) >> 8) & 0xFF), 24, 31)
    steer_angle_target_lower_data = (((0 + 500) & 0xFF), 32, 39)

    aeb_en_ctrl = (1, 1, 1)
    brake_en_ctrl = (1, 0, 0)
    brake_dec = 0.1 / 0.01
    brake_pedal_target = 10.0 / 0.1
    brake_dec_upper_byte = (((int(brake_dec) >> 2) & 0xFF), 8, 15)
    brake_dec_lower_byte = ((int(brake_dec) & 0b11), 22, 23)
    brake_pedal_target_upper_byte = (((int(brake_pedal_target) >> 8) & 0xFF), 24, 31)
    brake_pedal_target_lower_byte = ((int(brake_pedal_target) & 0xFF), 32, 39)

    print(f'brake_pedal_target : {brake_pedal_target}')
    print(f'brake_dec : {brake_dec}')

    gear_en_ctrl = (1, 0, 0)
    gear_target = (4, 8, 10)

    park_en_ctrl = (1, 0, 0)
    park_target = (1, 8, 8)

    steer_mode_ctrl = (2, 0, 2)
    drive_mode_ctrl = (0, 8, 10)
    turn_light_ctrl = (3, 16, 17)

    ret = generate_byte_array(8, gear_en_ctrl, gear_target)
    print_byte_array(ret)

    # for i in range(1, 256):
    #     checksum = (i, 56, 63)
    #     ret = generate_byte_array(8, checksum)
    #     print_byte_array(ret)