"""
BitMap

https://blog.csdn.net/xc_zhou/article/details/110672513
"""


class BitMap:
    def __init__(self, max_value: int) -> None:
        self.size = int((max_value + 31 - 1) / 31)
        self.array = [0 for i in range(self.size)]

    def get_element_index(self, num: int):
        value = num // 31
        # print(f'num = {num}, element_index = {value}')
        return value

    def get_bit_index(self, num: int):
        value = num % 31
        # print(f'num = {num}, bit_index = {value}')
        return value

    def set_value(self, num: int):
        element_index = self.get_element_index(num)
        bit_index = self.get_bit_index(num)
        value = self.array[element_index] | (1 << bit_index)
        self.array[element_index] = value

    def clean(self, num: int):
        element_index = self.get_element_index(num)
        bit_index = self.get_bit_index(num)
        value = self.array[element_index] & (1 << bit_index)
        self.array[element_index] = value

    def find(self, num: int):
        element_index = self.get_element_index(num)
        bit_index = self.get_bit_index(num)
        return bool(self.array[element_index] & (1 << bit_index))


if __name__ == '__main__':
    lis = [45, 2, 78, 35, 67, 90, 879, 0, 340, 123, 46]
    max_value = max(lis)

    results = []

    bit_map = BitMap(max_value)

    for n in lis:
        bit_map.set_value(n)

    for i in range(max_value + 1):
        if bit_map.find(i):
            results.append(i)

    print(lis)
    print(results)
