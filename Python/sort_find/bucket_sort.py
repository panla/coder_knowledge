"""
桶排序
非比较排序
"""

from tools import cal_time, random_lis


@cal_time
def bucket_sort(lis: list):
    max_value = max(lis)
    min_value = min(lis)
    bucket = [0] * (max_value + 1)

    # 最终结果
    ret = list()

    for v in lis:
        bucket[v] += 1

    for v in range(max_value + 1):
        for i in range(bucket[v]):
            ret.append(v)

    for i in range(len(lis)):
        lis[i] = ret[i]
    return lis


if __name__ == '__main__':
    print('桶排，100000')
    lis = random_lis(100000)
    bucket_sort(lis)

    print('桶排，30')
    lis = random_lis(30, 3)
    print(bucket_sort(lis))
