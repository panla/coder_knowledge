"""
BitMap

https://blog.csdn.net/xc_zhou/article/details/110672513
"""


class BitMap:
    def __init__(self, max_value: int) -> None:
        self.size = int((max_value + 31 - 1) / 31)
        self.array = [0 for _ in range(self.size)]

    def get_index(self, num: int):

        element_index, bit_index = divmod(num, 31)
        return element_index, bit_index

    def set_value(self, num: int):
        element_index, bit_index = self.get_index(num)
        value = self.array[element_index] | (1 << bit_index)
        self.array[element_index] = value

    def clean(self, num: int):
        element_index, bit_index = self.get_index(num)
        value = self.array[element_index] & (1 << bit_index)
        self.array[element_index] = value

    def find(self, num: int):
        element_index, bit_index = self.get_index(num)
        return bool(self.array[element_index] & (1 << bit_index))


if __name__ == '__main__':
    lis = [45, 2, 78, 35, 67, 90, 879, 0, 340, 123, 46]
    max_value = max(lis)

    results = list()

    bit_map = BitMap(max_value)

    for n in lis:
        bit_map.set_value(n)

    for i in range(max_value + 1):
        if bit_map.find(i):
            results.append(i)

    print(lis)
    print(results)
