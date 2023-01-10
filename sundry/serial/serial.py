import binascii
import time
import traceback

import serial


# 串口，端口
PORT_X = 'COM15'
# 波特率
BAUD_RATE = 9600
# 字节大小
BYTE_SIZE = serial.EIGHTBITS
#
PARITY = serial.PARITY_NONE
# STOP_BITS
STOP_BITS = serial.STOPBITS_ONE
# 超时
TIMEOUT = 0.5

client = serial.Serial(PORT_X, BAUD_RATE, BYTE_SIZE, PARITY, STOP_BITS, TIMEOUT)

if client.is_open:
    print('打开')
print(client.is_open)

string = "FE 04 03 E8 00 14 64 7A"
byte_str = bytes.fromhex(string)
result = client.write(byte_str)
print(result)

rt = b''
for i in range(20):

    if result:
        # 这里需要一定的长度和缓存截取
        origin_data = client.read(24)

        rt += origin_data

        if origin_data == b'':
            hex_data = binascii.b2a_hex(rt)
            bytearray_data = bytearray(rt)
            print(rt, bytearray_data, [i for i in bytearray_data])

            rt = b''
            client.flushInput()
            client.flushOutput()
            result = client.write(byte_str)