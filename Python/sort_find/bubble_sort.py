"""
冒泡，升序
属于交换排序
"""
from tools import cal_time


@cal_time
def func_flag(lis: list):
    """
    增加 一个 flag 标识
    TODO 更优
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

    print(func_flag([1, 2, 5, 3, 4]))
    print(func_flag([3, 2, 5, 4, 1]))
    print(func_flag([5, 1, 2, 3, 4]))
