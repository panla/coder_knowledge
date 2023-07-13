# MySQL

[TOC]

## 1 索引

### 1.0 索引基本原理

把无序的数据变成有序的查询

### 1.1 MySQL 索引类型

- 普通索引
- 主键索引
- 外键索引
- 唯一索引
- 联合索引
- 全文索引
- Hash索引
- 聚集索引（主键），各行的索引顺序与物理顺序一致
- 非聚集索引

聚簇索引，非聚簇索引区别

```text

```

### 1.2 约束类型

- 普通约束
- 主键约束
- 外键约束
- 唯一约束
- 非空约束

### 1.3 索引失效

- 联合索引没有使用左列字段
- like 以 % 开头
- 避免字段开头模糊查询
- or 时需要全都有索引
- in, not in
- WHERE 中索引列需要运算(等号左侧)
- WHERE 中索引列使用函数(等号左侧)
- WHERE 中索引列需要类型转换
- ORDER BY 条件与WHERE不一致
- 数据量较少，全表扫描更快
- 尽量非空，避免null值判断

### 1.4 索引使用注意事项

避免

- 频繁更新的字段
- 唯一性差的字段
- 不涉及到查询的字段
- 索引使用不等于符号 <>
- 避免索引失效

需要

- 主键
- 用于查询条件的字段
- 区分度高的字段
- 有唯一性要求的字段
- 覆盖索引，查询的列被所建的索引覆盖，查询时从索引中就可以获得

### 1.5 索引覆盖

```text
要查询的字段已经在索引中了，避免了回表
```

### 1.6 最左前缀匹配

```text
最左优先，在检索数据时从联合索引的最左边开始匹配
```

## 2 SQL语句优化

- 减少IO次数，IO数据量

### 2.1 SQL执行顺序

```sql
SELECT DISTINCT <list>
FROM <table>
<JOIN TYPE> JOIN <right table>
ON <left.id=right.id>
WHERE <condition>
GROUP BY <list>
HAVING <condition>
ORDER BY <condition>
LIMIT <num>
OFFSET <num>

FROM <table>
ON <left.id=right.id>
<JOIN TYPE> JOIN <right table>
WHERE <condition>
GROUP BY <list>
HAVING <condition>
SELECT DISTINCT <list>
ORDER BY <condition>
LIMIT <num>
OFFSET <num>
```

### 2.2 避免索引失效

### 2.3 避免使用 SELECT *

控制查询的字段，避免浪费资源(内存，cpu，网络)

SELECT * 容易产生回表

### 2.4 子查询小表驱动大表

```sql
SELECT id FROM orders
WHERE orders.user_id in (SELECT id FROM users WHERE users.status=1)

SELECT id FROM orders
WHERE exists (SELECT 1 FROM users WHERE orders.user_id = users.id AND users.status=1)
```

in 适合左大表，右小表，优先执行in里的子查询，如果in里的数据量少，速度会较快

exists 适合左小表，右大表，先查询左边语句，作为条件去和右边语句匹配

### 2.5 LIMIT

限制返回条数

按照 `user_id` 查找最早一条

```sql
SELECT id FROM orders
WHERE orders.user_id = {user_id}
ORDER BY orders.created_at ASC
LIMIT 1;
```

统计有无

```sql
SELECT 1 FROM orders
WHERE xxx
LIMIT 1;
```

### 2.6 优化分页

需要id连续

```sql
SELECT id FROM users
LIMIT 100000 OFFSET 20；

-- 换成
SELECT id FROM users
WHERE id BETWEEN 100000 AND 100020;
-- BETWEEN 需要在唯一索引上分页

SELECT id FROM users
WHERE id > 100000 LIMIT 20;
```

### 2.7 控制索引数量

索引过多，占用存储空间。影响写操作性能

### 2.8 选择合适的字段类型

- char varchar
- 能用数字不用字符串
- 尽可能小的类型 bit, tiny int
- 金额 decimal

### 2.9 分组查询

```sql
SELECT orders.user_id, COUNT(orders.id) FROM orders
GROUP BY orders.user_id
HAVING orders.user_id  <= 200;
-- 把所有订单分组后，再找到user_id<=200

-- 改为

-- 先找到user_id<=200，再分组
SELECT orders.user_id, COUNT(orders.id) FROM orders
WHERE orders.user_id  <= 200
GROUP BY orders.user_id;
```

### 2.10 explain 索引优化

### 2.11 连接查询子查询

- 子查询：简单，结构化，但需要创建删除临时表

```sql
SELECT o.id FROM orders o
INNER JOIN users u ON o.user_id = u.id
WHERE
```

left join 时选择小表驱动大表

### 2.12 使用 union all 代替 union

union可以获取 排重后的数据，union all可以获取所有数据包含重复数据

排重过程需要遍历，排序，比较，更为耗费时间，资源

### 2.13 批量操作

批量插入

## 3 慢查询

配置中开启慢查询 `slow_query_log` `long_query_time=2` 2秒

分析慢查询日志

统计SQL语句

对SQL语句进行分析优化

### 3.1 慢查询优化

## 4 explain

查看SQL执行计划

explain 内容

- id 该语句select唯一标识
- select_type 查询类型
  - simple 简单查询，未使用union或子查询
  - union
  - subquery 子查询
  - derived
- table 查询的表名
- partitions 匹配的分区
- type access_type 连接类型
  - system 只有一行
  - const 主键或唯一索引
- possible_keys 可能的索引选择
- key 实际选择的索引
- key_len 实际索引长度
- ref 与索引比较的列，索引的哪一列被引用了
- rows 估计要扫描的行
- filtered 符合查询条件的数据百分比
- extra 附加信息

## 5 GROUP BY, ORDER BY, HAVING, LIMIT, OFFSET, BETWEEN AND, LIKE

### 5.1 分组

### 5.2 排序

### 5.3 having

### 5.4 分页

## 6 char varchar 区别

- 1 char固定长度，varchar可变长度
- 2 填充
  - char右侧填充空格
  - varchar不填充
- 3 长度限制
  - char 255字符以内
  - varchar 65535字节以内，数据行的限制

## 7 事务

### 7.1 事务四大特性

- 原子性：
  - 不可分割，全部成功或全部失败
  - 一起成功，一起失败
  - 事务提交，事务回滚
- 一致性：
  - 事务必须使数据库从一个一致性状态变换到另一个一致性状态
  - 事务按照预期生效，数据的状态是预期的状态
  - 事务前后，数据的状态确保一致
- 隔离性：
  - 多个事务并发彼此不干扰
  - 一个事务未提交，另一个事务不能读取未提交的数据
- 持久性：
  - 事务提交后数据的改变持久化保存

### 7.2 事务隔离级别

级别

- 读未提交：可能会，脏读，不可重复读，幻读
- 读提交：不会脏读，可能会，不可重复读，幻读
- 可重复读：默认，不会 脏读和不可重复读，可能幻读
- 串行化

要解决的问题

- 脏读：读到了其他事物未提交的数据
- 可重复读：事务内，开始读到数据与结束前任意时刻读到的数据都一致，针对更新
- 不可重复读：事务内，不同时刻读到的数据可能不一致，受其他事务干扰，针对更新
  - 事务A查询时，事务B删除更新了数据，导致事务A再次查询时数据不一致
- 幻读：第一个事务更改后未提交，第二个事务插入了与第一个事务更改前相同的数据行
  - 事务A查询时，事务B插入了数据，导致事务A再次查询时查到了这条数据

## 8 MySQL 性能优化

### 8.1 参数配置优化

缓存区，连接数，线程数

- 缓冲区
  - 索引 `key_buffer_size` 索引缓冲
  - innodb `innodb_buffer_pool_size` 引擎缓冲
  - 查询缓冲 `query_cache_size`
  - 读缓冲区 `read_buffer_size`
  - 排序缓冲区 `sort_buffer_size`

### 8.2 单表大数据量

#### 8.2.1 分库

按照业务分库，一部分数据表放在一个库中

#### 8.2.2 分表

按照时间分表，按照`project_id`分表，按照字段水平分表

分区表

hash分表，取模分表，区间分表

#### 8.2.3 冷热分离

冷数据存储在其他库中/表

#### 8.2.4 NoSQL

MongoDB切片集群

ES

ClickHouse
