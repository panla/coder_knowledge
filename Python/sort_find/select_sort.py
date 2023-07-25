"""
直接选择排序, 选择排序的一种

首先在未排序序列中找到最小（大）元素，存放到排序序列的起始位置

再从剩余未排序元素中继续寻找最小（大）元素，然后放到已排序序列的末尾
"""

import random

from tools import cal_time


@cal_time
def select_sort(lis: list):
    length = len(lis)
    if length <= 1:
        return lis

    for i in range(length):
        min_index = i
        for j in range(i + 1, length):
            if lis[j] < lis[min_index]:
                min_index = j

        if i != min_index:
            lis[i], lis[min_index] = lis[min_index], lis[i]

    return lis


if __name__ == '__main__':
    print('选择排序，10000')
    lis = [i for i in range(10000)]
    random.shuffle(lis)
    select_sort(lis)

    print('选择排序，10')
    lis_1 = [i for i in range(10)]
    random.shuffle(lis_1)
    print(select_sort(lis_1))
