from typing import List, Union


def bin_search(lis: List[int], target: int):

    length = len(lis)
    start, end = 0, length - 1

    while start <= end:
        mid = (start + end) // 2

        if target == lis[mid]:
            return mid
        elif target > lis[mid]:
            start = mid + 1
        else:
            end = mid - 1

    return None

print(bin_search([1, 2, 3], 1))
print(bin_search([1, 2, 3], 2))
print(bin_search([1, 2, 3], 3))
print(bin_search([1, 2, 3, 4], 2))
