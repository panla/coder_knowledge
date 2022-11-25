"""
归并排序
将数组中的元素分成两个子数组, 迭代，对子数组排序，合并数组

改进，
    长度较短时，不再递归，选择其他方式
    归并过程，以记录数组下标方式代替申请新的内存空间，避免频繁移动
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


def merge_sort_1(lis: list):
    length = len(lis)
    if length <= 1:
        return lis

    middle_index = length >> 1
    left = merge_sort_1(lis[:middle_index])
    right = merge_sort_1(lis[middle_index:])
    return merge_1(left, right)


def merge_1(left: list, right: list):
    l, r = 0, 0
    result = []
    while l < len(left) and r < len(right):
        if left[l] <= right[r]:
            result.append(left[l])
            l += 1
        else:
            result.append(right[r])
            r += 1

    result += left[l:]
    result += right[r:]
    return result


temp = [0] * 110000


def merge_sort_2(lis: list, low: int, high: int):
    if low >= high:
        return lis

    middle_index = (low + high) >> 1
    merge_sort_2(lis, low, middle_index)
    merge_sort_2(lis, middle_index + 1, high)
    merge_2(lis, low, middle_index, high)
    return lis


def merge_2(lis: list, low: int, middle: int, high: int):
    i = low
    j = middle + 1
    size = 0

    while i <= middle and j <= high:
        if lis[i] < lis[j]:
            temp[size] = lis[i]
            i += 1
        else:
            temp[size] = lis[j]
            j += 1
        size += 1
    while i <= middle:
        temp[size] = lis[i]
        size += 1
        i += 1
    while j <= high:
        temp[size] = lis[j]
        size += 1
        j += 1
    for i in range(size):
        lis[low + i] = temp[i]


@cal_time
def m_main_1(lis, function):
    return function(lis)

@cal_time
def m_main_2(lis, function, low, high):
    return function(lis, low, high)


if __name__ == '__main__':
    print('归并排序，100000')
    lis_1 = [i for i in range(100000)]
    random.shuffle(lis_1)
    m_main_1(lis_1, merge_sort_1)

    print('归并排序，30')
    lis_1_1 = [i for i in range(30)]
    random.shuffle(lis_1_1)
    print(m_main_1(lis_1_1, merge_sort_1))

    print('归并排序，100000')
    lis_2_2 = [i for i in range(100000)]
    random.shuffle(lis_2_2)
    m_main_2(lis_2_2, merge_sort_2, 0, len(lis_2_2) - 1)
