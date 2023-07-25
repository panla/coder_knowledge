"""
桶排序
非比较排序
是计数排序的扩展，
计数排序每个桶只存储相同元素，桶排序每个桶存储一定范围的元素
尽量保证元素分散均匀

根据待排序集合中最大最小元素差值和映射规则，确定申请的桶的个数
遍历排序序列，把每个元素放入对应的桶
对不是空的桶进行排序
按顺序访问桶，把桶中元素放入结果
"""

from tools import cal_time, random_lis


@cal_time
def bucket_sort(lis: list):
    max_v, min_v = max(lis), min(lis)

    # 桶的大小
    bucket_size = (max_v - min_v) / len(lis)
    # 桶数组
    bucket = [[] for i in range(len(lis) + 1)]
    # 向桶中添加
    for i in lis:
        index = int((i - min_v) // bucket_size)
        bucket[index].append(i)
    # 清除原桶
    lis.clear()
    for i in bucket:
        for j in sorted(i):
            lis.append(j)

    return lis


if __name__ == '__main__':
    print('桶排，10000')
    lis = random_lis(10000)
    bucket_sort(lis)

    print('桶排，30')
    lis = random_lis(30, 3)
    print(bucket_sort(lis))
