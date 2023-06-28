# MySQL

[TOC]

## 1 索引类型

- 普通索引
- 主键索引
- 外键索引
- 唯一索引
- 联合索引
- 全文索引
- Hash索引
- 聚集索引（主键），各行的索引顺序与物理顺序一致
- 非聚集索引

## 2 约束类型

- 普通约束
- 主键约束
- 外键约束
- 唯一约束
- 非空约束

## 3 引擎区别

InnoDB

MyISAM

| 项目 | InnoDB | MyISAM |
| :-: | :-: | :-: |
| 事务支持 | 支持 | 不支持 |
| 锁定 |  | 表级锁定 |
| 读写阻塞 |  | 读写阻塞 |
| 外键 | 支持 | 不支持 |
| 全文索引 | 支持 | 支持 |
| 缓存 | 会缓存数据 | 不缓存数据 |
|  |  |  |

## 4 索引失效

- 联合索引没有使用左列字段
- like 以 % 开头
- 需要类型转换
- where 中索引列需要运算
- where 中索引列使用函数
- 数据量较少，全表扫描更快
- or 时需要全都有索引

## 5 索引使用注意事项

避免

- 频繁更新的字段
- 唯一性差的字段
- 不涉及到查询的字段
- 索引使用不等于符号 <>

## 6 SQL语句优化

## 7 慢查询

配置中开启慢查询

分析慢查询日志

统计SQL语句

对SQL语句进行分析优化

## 8 explain

查看SQL执行计划

explain 内容

```text
id select_id 该语句唯一标识
select_type 查询类型
    simple 简单查询，未使用union或子查询
    union
    subquery 子查询
    derived

table table 查询的表名
partitions 匹配的分区
type access_type 联接类型
    system 只有一行
    const 主键或唯一索引
possible_keys 可能的索引选择
key 实际选择的索引
key_len 索引长度
ref 索引的哪一列被引用了
rows 估计要扫描的行
filtered 符合查询条件的数据百分比
```

## 9 group by, order by, having, limit, offset

### 9.1 分组

### 9.2 排序

### 9.3 having

### 9.4 分页

## 10 char varchar 区别

## 11 事务

### 11.1 事务四大特性
