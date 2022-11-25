"""
冒泡，升序
交换排序
"""

import random


def func(lis: list):
    length = len(lis)
    if length <= 1:
        return lis
    for i_index in range(1, length):
        for j_index in range(0, length - i_index):
            if lis[j_index] > lis[j_index + 1]:
                lis[j_index], lis[j_index + 1] = lis[j_index + 1], lis[j_index]
    return lis


def func_2(lis: list):
    """
    和 func 相比 改变了循环的 index 值
    """

    length = len(lis)
    if length <= 1:
        return lis

    for i_index in range(length):
        for j_index in range(length - i_index - 1):
            if lis[j_index] > lis[j_index + 1]:
                lis[j_index], lis[j_index + 1] = lis[j_index + 1], lis[j_index]
    return lis


def func_flag(lis: list):
    """
    增加 一个 flag 标识
    """

    length = len(lis)
    if length <= 1:
        return lis

    for i_index in range(1, length):
        flag = False
        for j_index in range(0, length - i_index):
            if lis[j_index] > lis[j_index + 1]:
                lis[j_index], lis[j_index + 1] = lis[j_index + 1], lis[j_index]
                flag = True
        if not flag:
            break
    return lis


if __name__ == '__main__':
    
    lis_1 = [i for i in range(20)]
    random.shuffle(lis_1)
    print(lis_1)
    print(func(lis_1))

    lis_2 = [i for i in range(20)]
    random.shuffle(lis_2)
    print(lis_2)

    print(func_2(lis_2))
    
    print(func_flag([1, 2, 5, 3, 4]))
    print(func_flag([3, 2, 5, 4, 1]))
    print(func_flag([5, 1, 2, 3, 4]))
