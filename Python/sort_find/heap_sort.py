"""
堆排, 选择排序的一种
堆积是一个近似完全二叉树的结构，并同时满足堆积的性质：即子结点的键值或索引总是小于（或者大于）它的父节点。
堆排序可以说是一种利用堆的概念来排序的选择排序。

Ki <= K2i and Ki <= K(2i+1)
or
Ki >= K2i and Ki >= K(2i+1)
and
1 <= i <= (n / 2)
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


def adjust_heap(lis: int, i: int, size: int):
    l_child = 2 * i + 1
    r_child = 2 * i + 2
    max_index = i
    if i < (size >> 1):
        if l_child < size and lis[l_child] > lis[max_index]:
            max_index = l_child
        if r_child < size and lis[r_child] > lis[max_index]:
            max_index = r_child
        if max_index != i:
            lis[max_index], lis[i] = lis[i], lis[max_index]
            adjust_heap(lis, max_index, size)


def build_heap(lis: list, size: int):
    """
    构造堆
    """

    for i in range(0, (size >> 1))[::-1]:
        adjust_heap(lis, i, size)


@cal_time
def heap_sort(lis: list, size: int):

    for i in range(0, size)[::-1]:
        lis[0], lis[i] = lis[i], lis[0]
        adjust_heap(lis, 0, i)
    return lis


def main(lis: list):
    size = len(lis)
    for i in range(0, (size >> 1))[::-1]:
        adjust_heap(lis, i, size)

    return heap_sort(lis, size)

    
if __name__ == '__main__':
    print('堆排，100000')
    lis_1 = [i for i in range(100000)]
    random.shuffle(lis_1)
    main(lis_1)

    print('堆排，10000')
    lis_2 = [i for i in range(10000)]
    random.shuffle(lis_2)
    main(lis_2)

    print('堆排，30')
    lis_3 = [i for i in range(30)]
    random.shuffle(lis_3)
    print(main(lis_3))
