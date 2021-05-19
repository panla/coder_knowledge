"""
希尔排序，插入排序的一种。

也称为缩小增量排序，是直接插入排序算法的一种更高效的改进版本
非稳定排序
将待排序列划分为若干组，在每一组内进行插入排序，以使整个序列基本有序，然后再对整个序列进行插入排序
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

    step = length >> 1
    while step > 0:
        for i in range(step, length):
            temp = lis[i]
            j = i
            while j >= step and temp < lis[j - step]:
                lis[j] = lis[j - step]
                j -= step
            lis[j] = temp
        step = step >> 1
    return lis


if __name__ == '__main__':
    print('希尔排序，100000')
    lis = [i for i in range(100000)]
    random.shuffle(lis)
    func(lis)

    print('希尔排序，30')
    lis_1 = [i for i in range(30)]
    random.shuffle(lis_1)
    print(func(lis_1))
