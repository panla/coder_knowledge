# MySQL 函数

[TOC]

## 一般函数

```text
abs()               求绝对值
celling(9.4)        向上取整
floor(9.4)          向下取整
rand()              0--1的随机数


char_length('')         字符串长度
concat('', '')          拼接字符串
insert('', 1, 2, '')    替换
lower('')               小写
upper('')               大写
instr('', '')           第一次出现字串的索引
replace('', '', '')     替换出现的指定的字符串
substr('', 4, 6)        返回指定字串

current_date()          当前日期
current_timestamp()     当前时间戳，2038
now()                   当前时间
localtime()             本地时间
```

## 聚合函数

```text
count()             统计个数
max()               最大值
min()               最小值
avg()               平均值
sun()               总和
distinct()          去重
```
