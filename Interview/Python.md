# Python

[TOC]

## 1 语法，数据类型

### 1.1 列表list与元组tuple区别

- 区别一：列表属于可变类型，元组属于不可变类型
- 区别二：列表元素可以修改，元组不可以修改
- 区别三：空列表的占用空间为40字节，空元组占用空间为24字节

### 1.2 字典的key的要求

要求key属于不可变类型，可哈希，

可以是 string, number, tuple,

### 1.3 可哈希

```text
生命周期内，不可变

可哈希对象 __hash__方法

用户所定义的类的实例对象默认为可哈希（唯一），其hash值也是id
```

### 1.4 双下划线方法

```text
__init__ 对实例初始化参数
__new__ 生成实例
__call__ 可调用对象，让类的实例的行为表现的像函数一样

__hash__ 可哈希
__iter__ 可迭代
__next__ 可以找到下一个元素
__dict__ 可字典化结果
__copy__ 复制

__name__
__str__ 字符串名称
__del__ 销毁
__len__ 长度，字符串长度，容器长度

__exit__ 上下文管理退出
__enter__ 上下文管理进入

__setattr__ 设置属性
__getattr__ 获取属性
__delattr__ 删除属性

__setitem__
__getitem__
__delitem__

__dir__ 对象的属性，方法
__doc__ 文档
```

### 1.5 装饰器

#### 1.5.1 单例

```python
import threading

def singleton(cls):

    _instances = {}
    _lock = threading.Lock()

    def inner(*args, **kwargs):
        with _lock:

            if cls not in _instances:
                _instances[cls] = cls(*args, **kwargs)
            return _instances[cls]
    return inner

class Singleton:

    _instances = {}
    _lock = threading.Lock()

    def __init__(self, cls):
        self._cls = cls

    def __call__(self, *args, **kwargs):

        with self._lock:
            if self._cls not in self._instances:
                self._instances[self._cls] = self._cls(*args, **kwargs)
            return self._instances[self._cls]

class Person:

    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        # 缺点，__init__ 不能做其他事，否则相当于又生了一个新的实例，而非第一个
        pass

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if not cls._instance:
                cls._instance = super().__new__(cls, *args, **kwargs)
            return cls._instance

class Meta(type):
    _instances = {}
    _lock = threading.Lock()

    def __call__(cls, *args, **kwargs):

        with cls._lock:
            if cls not in cls._instances:
                cls._instances[cls] = super().__call__(*args, **kwargs)
            return cls._instances[cls]

class Student(metaclass=Meta):
    def __init__(self, name) -> None:
        self.name = name
```

#### 1.5.2 求执行时间

```python
import time

def cal_time(func):

    def inner(*args, **kwargs):
        start = time.time()
        rt = func(*args, **kwargs)
        end = time.time()
        print(f'{func.__name__} costs {end - start} seconds')
        return rt
    return inner
```

### 1.6 dict 实现

### 1.7 异步，协程

## 2 代码

### 2.1 排序

#### 2.1.1 冒泡排序

```python
def bubble_sort(lis):

    length = len(lis)
    if length <= 1:
        return lis

    for i in range(1, length):
        flag = False
        for j in range(0, length - i):
            if lis[j] > lis[j + 1]:
                lis[j], lis[j + 1] = lis[j + 1], lis[j]
                flag = True
        if not flag:
            break
    return lis
```

#### 2.1.2 插入排序

```python
def insert_sort(lis):

    length = len(lis)
    if length <= 1:
        return lis

    for i in range(1, length):
        temp = lis[i]
        j = i
        while j >= 1 and temp < lis[j-1]:
            lis[j] = lis[j-1]
            j -= 1
        lis[j] = temp
    return lis
```

#### 2.1.3 选择排序

#### 2.1.4 快排

```python
def quick(lis):
    length = len(lis)
    if length < 2:
        return lis

    mid_index = length // 2
    left = [i for i in lis if i < lis[mid_index]]
    right = [i for i in lis if i > lis[mid_index]]

    return quick(left) + [lis[mid_index]] + quick(right)

```

#### 2.1.5 希尔排序

#### 2.1.6 归并排序

```python
def merge(left, right):

    l, r = 0, 0
    results = []

    l_length, r_length = len(left), len(right)

    while l < l_length and r < r_length:
        if left[l] <= right[r]:
            results.append(left[l])
            l += 1
        else:
            results.append(right[r])
            r += 1

    results += left[l:]
    results += right[r:]

    return results

def merge_sort(lis):
    length = len(lis)
    if length <= 1:
        return lis

    mid = length >> 1
    left = merge_sort(lis[:mid])
    right = merge_sort(lis[mid:])

    return merge(left, right)
```

#### 2.1.7 堆排

#### 2.1.8 桶排序

#### 2.1.9 基数排序

#### 2.1.10 计数排序

### 2.2 二分查找

```python
def bin_search(lis, target):

    left, right = 0, len(lis)

    while left <= right:

        mid = (left + right) // 2

        if lis[mid] == target:
            return mid
        elif lis[mid] < target:
            left = mid + 1
        else:
            right = mid - 1
    return -1
```

### 2.3 求素数

求一定范围内的素数

```python
def function(start, end):

    for i in range(start, end + 1):
        flag = False
        for j in range(2, int(i ** 0.5) + 1):
            if i % j == 0:
                flag = True
                break

        if not flag:
            print(i)
```

## 3 Python概念

### 3.1 引用计数

#### 3.1.1 引用计数 +1

```text
1，对象被创建时
2，对象被copy引用时，赋值到其他对象时，
3，对象作为参数传入到函数中
4，对象作为子元素存储到容器中，list tuple
```

#### 3.1.2 引用计数 -1

```text
对象被显示销毁，del name
对象被赋予新的对象时，
    name_a = "name"
    name_b = name_a
    name_b = "athena"
    此时 name_a 引用次数-1
离开作用域
对象所在容器被销毁

```

### 3.2 标记清除

```text
解决引用计数循环引用的缺点

list dict tuple set class 底层再维护一个新的链表，存储可能产生循环引用的对象

扫描检查内部元素，检查是否有循环引用情况，当子/孙级元素是父级本身的时候，证明出现了循环引用
此时让双方的引用计数同时-1，如果-1后是0，则触发垃圾回收

```

### 3.3 分代回收

```text
以空间换时间

如果对象存活时间长就认为不太可能是垃圾对象

把对象分成若干代(0,1,2)，每代由一个链表维护

0代超过700，1，2代超过10，触发回收机制
0代触发后清理0，1，2代
1代触发后清理1，2代
2代触发后清理3代
```

## 4 设计模式

### 4.1 单例

```python
import threading

def singleton(cls):

    _instances = {}
    _lock = threading.Lock()

    def inner(*args, **kwargs):
        with _lock:

            if cls not in _instances:
                _instances[cls] = cls(*args, **kwargs)
            return _instances[cls]
    return inner

class Singleton:

    _instances = {}
    _lock = threading.Lock()

    def __init__(self, cls):
        self._cls = cls

    def __call__(self, *args, **kwargs):

        with self._lock:
            if self._cls not in self._instances:
                self._instances[self._cls] = self._cls(*args, **kwargs)
            return self._instances[self._cls]
```

### 4.2 工厂方法

### 4.3 代理
