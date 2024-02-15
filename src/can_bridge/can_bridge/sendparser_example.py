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
    mask = ((1 << (end_bit - start_bit + 1)) - 1) << (data.bit_length() - 1 - end_bit)
    # Extract data using the bitmask
    extracted_data = (data & mask) >> (data.bit_length() - 1 - end_bit)
    return extracted_data


def int_to_bits(n):
    """Converts an integer to its binary representation as a string.

    Args:
        n (int): The integer to convert.

    Returns:
        str: A binary string representing the integer, padded with zeros to ensure a length of 8.
    """
    return bin(n)[2:].zfill(8)


def generate_byte_array(data: int, start_bit: int, end_bit: int) -> bytearray:
    """Generates a byte array with specified data bits.

    Args:
        data (int): The integer data to be packed into the byte array.
        start_bit (int): The starting bit index of the data within the byte array.
        end_bit (int): The ending bit index of the data within the byte array.

    Returns:
        bytearray: A bytearray containing the specified data bits packed into bytes.

    The function generates a bytearray and fills it with the specified data bits
    within the given range of bit indices. The data is extracted from the input
    integer 'data' and packed into bytes, with each byte representing 8 bits.
    """
    # Calculate start_byte, start_bit_offset, end_byte, end_bit_offset
    start_byte, start_bit_offset = divmod(start_bit, 8)
    end_byte, end_bit_offset = divmod(end_bit, 8)

    # Create an empty byte array
    byte_array = bytearray(end_byte + 1)

    # Fill data bits into the byte array
    for byte_index in range(start_byte, end_byte + 1):
        byte = 0
        # Iterate through each bit in the byte
        for bit_offset in range(8):
            # Calculate the bit index in the byte array
            current_bit = byte_index * 8 + (7 - bit_offset)
            # Check if the current bit falls within the specified range
            if start_bit <= current_bit <= end_bit:
                # Extract the bit value from the data and set it in the byte
                bit_value = (data >> (end_bit - current_bit)) & 1
                byte |= bit_value << bit_offset
        # Store the byte in the byte array
        byte_array[byte_index] = byte

    return byte_array


def bytearray_to_bits(byte_array):
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


def extract_bits(byte_array, start_bit, end_bit):
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


def main():
    data = 1
    start_bit = 0
    end_bit = 19

    byte_array = generate_byte_array(data, start_bit, end_bit)
    result = extract_bits(byte_array, start_bit, end_bit)
    print(bytearray_to_bits(byte_array))
    print("byte_array:", byte_array)

    print("Result:", result)


if __name__ == "__main__":
    main()
