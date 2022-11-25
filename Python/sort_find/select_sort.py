"""
直接选择排序, 选择排序的一种

首先在未排序序列中找到最小（大）元素，存放到排序序列的起始位置

再从剩余未排序元素中继续寻找最小（大）元素，然后放到已排序序列的末尾
"""

import random
import time


def cal_time(func):
    def inner(*args, **kwargs):
        s_time = time.time()
        ret = func(*args, **kwargs)
        e_time = time.time()
        print(f'{func.__name__} costs {e_time - s_time} seconds')
        return ret
    return inner


@cal_time
def func(lis: list):
    length = len(lis)
    if length <= 1:
        return lis

    for index in range(length - 1):
        min_index = index
        for j in range(index + 1, length):
            if lis[j] < lis[min_index]:
                min_index = j
        if index != min_index:
            lis[index], lis[min_index] = lis[min_index], lis[index]
    return lis


if __name__ == '__main__':
    print('选择排序，10000')
    lis = [i for i in range(10000)]
    random.shuffle(lis)
    func(lis)

    print('选择排序，10')
    lis_1 = [i for i in range(10)]
    random.shuffle(lis_1)
    print(func(lis_1))
