import ctypes
import time

hello_so = ctypes.cdll.LoadLibrary('../lib/libhello.so')
hello_so.func()

person_so = ctypes.cdll.LoadLibrary('../lib/libperson.so')
person_so.person_all()

dog_so = ctypes.cdll.LoadLibrary('../lib/libdog.so')
rt = dog_so.func(12)
print(rt)
print('*' * 60)

fib_len = 40
fib_so = ctypes.cdll.LoadLibrary('../lib/libfib.so')
s_time = time.time()
rt = fib_so.fib(fib_len)
print(rt)
time_const_1 = time.time() - s_time
print(time_const_1)


def fib(n):
    if n in [1, 2]:
        return 1
    return fib(n - 1) + fib(n - 2)

s_time = time.time()
rt = fib(fib_len)
print(rt)
time_const_2 = time.time() - s_time
print(time_const_2)
print(f'计算斐波那契数列，Python耗时 {time_const_2}，C 耗时 {time_const_1}，倍数为 {time_const_2 / time_const_1}')
