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


def random_lis(max_value: int, min_value: int = 0):
    lis = [i for i in range(min_value, max_value)]
    random.shuffle(lis)
    return lis
