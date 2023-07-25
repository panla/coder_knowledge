"""
归并排序

分治法，将原问题划分成n个规模较小的结构与原问题相似的字问题
递归解决这些问题，再合并其结果

将数组中的元素分成两个子数组, 迭代，对子数组排序，合并数组

改进，
    长度较短时，不再递归，选择其他方式
    归并过程，以记录数组下标方式代替申请新的内存空间，避免频繁移动
"""

import random

from tools import cal_time


def merge(left: list, right: list):

    result = []
    i, j = 0, 0
    l_len = len(left)
    r_len = len(right)

    while i < l_len and j < r_len:
        if left[i] < right[j]:
            result.append(left[i])
            i += 1
        else:
            result.append(right[j])
            j += 1

    result += left[i:]
    result += right[j:]

    return result


def merge_sort(lis: list):
    length = len(lis)
    if length <= 1:
        return lis

    mid_index = length >> 1
    left = merge_sort(lis[:mid_index])
    right = merge_sort(lis[mid_index:])

    return merge(left, right)


@cal_time
def main(lis, function):
    return function(lis)


if __name__ == '__main__':

    print('归并排序，100000')
    lis_2_2 = [i for i in range(10)]
    random.shuffle(lis_2_2)
    main(lis_2_2, merge_sort)

    print(main([1, 3, 2, 5], merge_sort))
