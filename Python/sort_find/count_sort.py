"""
计数排序
将输入的数据值转化为键存储在额外开辟的数组空间中。
作为一种线性时间复杂度的排序，计数排序要求输入的数据必须是有确定范围的整数。

找到最大元素
统计数组中每个元素出现的次数
把数组的次数组合成结果
"""

from tools import cal_time, random_lis


@cal_time
def count_sort(lis: list):

    length = max(lis) + 1
    bucket = [0] * length

    result = []

    for i in lis:
        bucket[i] += 1

    for i in range(length):
        if bucket[i] != 0:
            result.extend(bucket[i] * [i])

    return result


@cal_time
def count_sort_2(lis: list):

    length = max(lis) + 1
    bucket = [0] * length

    for v in lis:
        bucket[v] += 1

    # 缓存
    temp = list()

    for i in range(length):
        for _ in range(bucket[i]):
            temp.append(i)

    for i in range(len(lis)):
        lis[i] = temp[i]

    return lis


if __name__ == '__main__':

    print('计数排序，30')
    lis = random_lis(30, 3)
    print(count_sort(lis))

    print('计数排序，30')
    lis = random_lis(30, 3)
    print(count_sort_2(lis))

    print('计数排序，10000')
    lis = random_lis(10000)
    count_sort(lis)
