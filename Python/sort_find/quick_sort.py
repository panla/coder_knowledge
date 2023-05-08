"""
快排，交换排序

通过一趟排序将要排序的数据分割成独立的两部分，其中一部分的所有数据都比另外一部分的所有数据都要小，
然后再按此方法对这两部分数据分别进行快速排序，整个排序过程可以递归进行，以此达到整个数据变成有序序列。
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


def quick_sort(lis: int):
    length = len(lis)
    if length <= 1:
        return lis

    key = lis[(length >> 1)]
    left = [i for i in lis if i < key]
    right = [i for i in lis if i > key]
    return quick_sort(left) + [key] + quick_sort(right)


@cal_time
def main(lis):
    ret = quick_sort(lis)
    return ret


if __name__ == '__main__':

    print('快速排序，100000')
    lis_1 = [i for i in range(100000)]
    random.shuffle(lis_1)
    main(lis_1)

    print('快速排序，10000')
    lis_2 = [i for i in range(10000)]
    random.shuffle(lis_2)
    main(lis_2)

    print('快速排序，30')
    lis_3 = [i for i in range(30)]
    random.shuffle(lis_3)
    print(main(lis_3))
