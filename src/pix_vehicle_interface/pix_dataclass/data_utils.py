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


def extract_bits(byte_array: bytearray, start_bit: int, end_bit: int) -> int:
    """Extracts bits from a bytearray within the specified bit range.

    Args:
        byte_array (bytearray): The bytearray from which bits will be extracted.
        start_bit (int): The starting bit index (inclusive) of the range to extract.
        end_bit (int): The ending bit index (inclusive) of the range to extract.

    Returns:
        int: The extracted bits within the specified bit range.

    This function extracts bits from the given bytearray within the specified bit range.
    It iterates through each bit in the range and extracts it from the corresponding byte
    in the bytearray. The extracted bits are combined and returned as an integer.
    """
    # Calculate the number of bits in the range
    num_bits = end_bit - start_bit + 1

    # Variable to store the extracted bits
    extracted_bits = 0

    # Extract bits from the bytearray and add them to extracted_bits
    for i in range(num_bits):
        # Calculate the index of the current bit
        bit_index = start_bit + i

        # Calculate the index and offset of the byte containing the current bit
        byte_index, bit_offset = divmod(bit_index, 8)

        # Extract the bit from the byte and add it to extracted_bits
        extracted_bits <<= 1
        extracted_bits |= (byte_array[byte_index] >> (7 - bit_offset)) & 1

    return extracted_bits


def bytearray_to_bits(byte_array: bytearray) -> str:
    """Converts a bytearray to a binary string representation.

    Args:
        byte_array (bytearray): The bytearray to convert.

    Returns:
        str: A binary string representing the contents of the bytearray.

    This function takes a bytearray as input and converts it to a binary string representation.
    It iterates through each byte in the bytearray, converts it to binary, and adds it to a bit string.
    The resulting bit string represents the contents of the input bytearray.
    """
    bits = ""
    for byte in byte_array:
        bits += bin(byte)[2:].zfill(8)  # Convert each byte to binary and add it to a bit string
    return bits


def int_to_bits(n: int) -> str:
    """Converts an integer to its binary representation as a string.

    Args:
        n (int): The integer to convert.

    Returns:
        str: A binary string representing the integer, padded with zeros to ensure a length of 8.
    """
    return bin(n)[2:].zfill(8)




