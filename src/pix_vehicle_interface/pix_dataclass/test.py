import struct


def generate_byte_array(array_size: int, *args: object) -> bytearray:

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
        for byte_offset in range(start_byte, end_byte + 1):
            byte_value = 0

            # Iterate through each bit in the byte
            for bit_offset in range(8):
                # Calculate the bit index in the byte array
                current_bit = byte_offset * 8 + bit_offset

                # Check if the current bit falls within the specified range
                if start_bit <= current_bit <= end_bit:
                    # Extract the bit value from the data and set it in the byte_value
                    bit_value = (data >> bit_offset) & 1
                    byte_value |= bit_value << bit_offset

            # Store the byte_value in the byte array
            byte_array[byte_offset] = byte_value

    return byte_array


def split_data(data: int, start_bit: int, end_bit: int) -> int:
    """Extracts data within the specified bit range from an integer.
    Args:
        data (int): The integer from which data will be extracted.
        start_bit (int): The starting bit index (inclusive) of the data to extract.
        end_bit (int): The ending bit index (inclusive) of the data to extract.
    Returns:
        int: The extracted data within the specified bit range.
    This function extracts data within the specified bit range from the given integer 'data'.
    It first calculates the bitmask for data extraction based on the start and end bit indices.
    Then, it applies the bitmask to the input data and performs a bit shift to obtain the
    extracted data. The extracted data is returned as an integer.
    """

    # Calculate the bitmask for data extraction
    mask = ((1 << (end_bit - start_bit + 1)) - 1) << start_bit
    # Extract data using the bitmask
    extracted_data = (data & mask) >> start_bit

    return extracted_data




def format_candump(byte_array):
    hex_representation = ' '.join(format(byte, '02x') for byte in byte_array)
    ascii_representation = ''.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in byte_array)
    return f"{hex_representation}  [{ascii_representation}]"




if __name__ == "__main__":
    data = 500
    upper_byte = (data >> 8) & 0xFF
    lower_byte = data & 0xFF
    print("상위 8비트:", upper_byte)
    print("하위 8비트:", lower_byte)
    steer_en_ctrl = (1, 0, 0)
    steer_angle_spd = (255, 8, 15)
    steer_angle_upper_byte = (upper_byte, 24, 31)
    steer_angle_lower_byte= (lower_byte, 32, 39)
    steer_ange = (500, 24, 39)
    checksum_102 = (0, 56, 63)

    result = generate_byte_array(8, steer_en_ctrl, steer_angle_spd, steer_angle_upper_byte, steer_angle_lower_byte, checksum_102)
    hex_representation = format_candump(result)
    print(f'byte array ::{hex_representation}')
