import struct

def convert_data_to_bytearray(data):
    # 0번 비트에만 데이터를 넣고 나머지 비트는 0으로 초기화
    byte_value = data & 0b00000001

    # 바이트로 패킹
    packed_byte = struct.pack('B', byte_value)

    # 바이트 어레이로 변환
    byte_array = bytearray(packed_byte)

    return byte_array

# 예제 데이터
data = 5

# 데이터를 바이트 어레이로 변환
result = convert_data_to_bytearray(data)

# 결과 출력
print("Converted Bytearray:", result)
