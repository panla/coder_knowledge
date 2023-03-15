"""
计算 crc

crc16 modbus
"""

import struct


class CalcOp:

    @classmethod
    def calc_crc16(cls, original_data: str, flag: bool = True):
        """计算 crc16 modbus

        crc >> 8
        (crc & 0xff) << 8

        :param original_data hex string
        :param flag bool 翻转
        """

        data = bytearray.fromhex(original_data)

        crc = 0xFFFF

        for index in data:
            crc ^= index
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1

        if flag:
            return struct.pack('B', crc >> 8).hex(), struct.pack('B', crc & 0xff).hex()
        else:
            return struct.pack('B', crc & 0xff).hex(), struct.pack('B', crc >> 8).hex()
