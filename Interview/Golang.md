# Golang

[TOC]

## 1 GC

## 2 GMP模型

## 3 类型

### 3.1 map

#### 3.1.1 map 实现

```go
type hmap struct {
    count       int
    flags       uint8
    B           uint8
    noverflow   uint8
    hash0       uint32

    buckets     unsafe.Pointer
    oldbuckets  unsafe.Pointer
    nevacuate   uintptr
    extra       *mapextra
}

```

#### 3.1.2 map遍历是有序的吗

#### 3.1.3 map作为函数是什么传递

值，一切皆值传递，

指针的复制 值传递

值的复制 值传递

#### 3.1.4 函数内修改map会影响原对象

map引用类型，引用的值复制传递

array 不会

slice 引用类型，会

### 3.2 channel

#### 3.2.1 读写关闭的channel

读：剩余元素或者零值

写：panic

### 3.3 反射和interface转换

## 4 语法

### 4.1 init

先全局变量或常量

## 5 锁

### 5.1 死锁

必要条件

- 互斥条件
- 请求和保持条件
- 不剥夺条件
- 环路等待条件

### 5.2 分布式锁

## 6 部分算法

### 6.1 反转链表

### 6.2 连续子序列最大和
