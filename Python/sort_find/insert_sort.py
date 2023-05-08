"""
直接插入，插入排序的一种

通过构建有序序列，对于未排序数据，在已排序序列中从后向前扫描，找到相应位置并插入

相当于将L[i]与 "已排序序列" 中的元素(从后向前)依次比较，如果L[i]比较小，则交换位置
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

    for i in range(1, length):
        j = i

        while j >= 1 and lis[j] < lis[j - 1]:
            lis[j], lis[j - 1] = lis[j - 1], lis[j]
            j -= 1
    return lis


@cal_time
def func_2(lis: list):
    length = len(lis)
    if length <= 1:
        return lis

    for i in range(1, length):
        for j in range(i, 0, -1):
            if lis[j] < lis[j - 1]:
                lis[j], lis[j - 1] = lis[j - 1], lis[j]
            else:
                break
    return lis


@cal_time
def func_3(lis: list):
    """
    插入排序，相当于将L[i]与 "已排序序列" 中的元素(从后向前)依次比较
    如果 L[i] 比较小，则将 "已排序序列" 中的元素依次向后挪动一个位置
    """

    length = len(lis)
    if length <= 1:
        return lis

    # 假定 [lis[0]] 是已排序序列，i 表示 "未排序序列" 的第一个元素的下标，从 1 开始
    for i in range(1, length):
        temp = lis[i]
        j = i
        while j >= 1 and temp < lis[j-1]:
            lis[j] = lis[j-1]
            j -= 1
        lis[j] = temp
    return lis


if __name__ == '__main__':
    print('插入排序， 10000')
    lis = [i for i in range(10000)]
    random.shuffle(lis)
    func(lis)

    print('插入排序， 10')
    lis_1_1 = [i for i in range(10)]
    random.shuffle(lis_1_1)
    print(func(lis_1_1))

    print('插入排序， 10000')
    lis_2 = [i for i in range(10000)]
    random.shuffle(lis_2)
    func_2(lis_2)

    print('插入排序， 10')
    lis_2_2 = [i for i in range(10)]
    random.shuffle(lis_2_2)
    print(func_2(lis_2_2))

    print('插入排序， 10000')
    lis_3 = [i for i in range(10000)]
    random.shuffle(lis_3)
    func_3(lis_3)

    print('插入排序， 10')
    lis_3_3 = [i for i in range(10)]
    random.shuffle(lis_3_3)
    print(func_3(lis_3_3))
