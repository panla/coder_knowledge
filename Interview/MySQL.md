# MySQL

[TOC]

## 1 MySQL

### 1.1 锁

- 表级锁：开销小，加锁快，不死锁，粒度大，冲突概率高，并发低
- 行级锁：开销大，加锁慢，会死锁，粒度小，冲突概率低，并发高
- 页面锁：开销，加锁时间，粒度在行锁表锁之间，会死锁，并发一般

### 1.2 引擎

- MyISAM
  - 不支持事务，但每次查询都是原子操作
  - 表级锁
  - 存储总行数
  - 表包括
    - 索引文件，表结构文件，数据文件
- INnoDB B+树
  - 支持ACID事务，支持四种事务隔离级别
  - 支持行级锁，外键
  - 不存储总行数

### 1.3 字段类型，数据类型

#### 1.3.1 char varchar 区别

- 1 char固定长度，varchar可变长度
- 2 填充
  - char右侧填充空格
  - varchar不填充
- 3 长度限制
  - char 255字符以内
  - varchar 65535字节以内，数据行的限制
- 4 字符
  - char 对英文ASCII占用1个字节，对汉字占用2个字节
  - varchar 对每个字符占用2个字节

### 1.4 函数

- CONCAT(A, B) 连接两个字符串，把多个字段合并为一个字段
- FORMAT(X, D) 格式化数字X到D有效数字
- CURRDATE() CURRTIME()当前日期，时间
- NOW() 当前日期和时间
- MONTH() DAY() YEAR() WEEK() WEEKDAY()
- HOUR() MINUTE() SECOND()
- DATEDIFF(A, B) 日期差异
- SUBTIMES(A, B) 两次之间的差异
- FROMDAYS(INT) 把整数天转化为日期

## 2 事务

### 2.1 事务四大特性

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

### 2.2 事务隔离级别

级别

- read uncommitted 读未提交：可能会，脏读，不可重复读，幻读
- read committed 读提交：不会脏读，可能会，不可重复读，幻读
- repeatable read 可重复读：默认，不会 脏读和不可重复读，可能幻读
- serializable 串行化

要解决的问题

- 脏读：读到了其他事物未提交的数据
- 可重复读：事务内，开始读到数据与结束前任意时刻读到的数据都一致，针对更新
- 不可重复读：事务内，不同时刻读到的数据可能不一致，受其他事务干扰，针对更新
  - 事务A查询时，事务B删除更新了数据，导致事务A再次查询时数据不一致
- 幻读：第一个事务更改后未提交，第二个事务插入了与第一个事务更改前相同的数据行
  - 事务A查询时，事务B插入了数据，导致事务A再次查询时查到了这条数据

## 3 MySQL 性能优化

### 3.0 优化

```text
SQL语句以及索引优化
数据库表结构优化
系统配置优化
硬件优化
```

```text
选取合适的字段属性，减少字段宽度，尽量非空
合适的索引
用连接查询代替子查询
用联合UNION代替临时表
优化查询语句
事务保证数据
```

### 3.1 参数配置优化

缓存区，连接数，线程数

- 缓冲区
  - 索引 `key_buffer_size` 索引缓冲
  - INnodb `INnodb_buffer_pool_size` 引擎缓冲
  - 查询缓冲 `query_cache_size`
  - 读缓冲区 `read_buffer_size`
  - 排序缓冲区 `sort_buffer_size`

### 3.2 慢查询

配置中开启慢查询 `slow_query_log` `long_query_time=2` 2秒

分析慢查询日志

统计SQL语句

对SQL语句进行分析优化

### 3.3 单表大数据量

#### 3.3.1 分库

按照业务分库，一部分数据表放在一个库中

#### 3.3.2 分表

按照时间分表，按照`project_id`分表，按照字段水平分表

分区表

hash分表，取模分表，区间分表

#### 3.3.3 冷热分离

冷数据存储在其他库中/表

#### 3.3.4 NoSQL

MongoDB切片集群

ES

ClickHouse

### 3.4 每天5万数据，维护三年，处理方案

```text
1，良好的数据库结构，允许冗余，尽量避免JOIN查询
2，适当索引
3，主从读写分离
4，每条数据减少数据量，优化空间占用和查询性能
5，利用缓存，Redis
6，优化SQL
```

## 4 索引与约束

### 4.0 索引基本原理

把无序的数据变成有序的查询

快速访问数据表中的特定信息，提高检索速度，用唯一约束保证唯一性，加速表和表之间的连接

写和维护索引耗费时间，占用物理空间

```text
在所有的叶子结点中增加了指向下一个叶子结点的指针
```

### 4.1 MySQL 索引约束类型

> 索引

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

> 约束

- 普通约束
- 主键约束
- 外键约束
- 唯一约束
- 非空约束

### 4.2 索引失效

- 联合索引没有使用左列字段
- LIKE 以 % 开头，避免字段开头模糊查询
- OR 前后没有都使用索引
- IN, NOT IN
- WHERE 中索引列需要运算(等号左侧)
- WHERE 中索引列使用函数(等号左侧)
- WHERE 中索引列需要类型转换
- ORDER BY 条件与WHERE不一致
- 数据量较少，全表扫描更快
- 尽量非空，避免null值判断

### 4.3 索引使用注意事项

避免

- 频繁更新的字段
- 唯一性差的字段
- 不涉及到查询的字段
- 索引使用不等于符号 <>
- 避免索引失效

需要

- 主键
- 用于查询，排序条件的字段
- 区分度高的字段
- 有唯一性要求的字段
- 覆盖索引，查询的列被所建的索引覆盖，查询时从索引中就可以获得

### 4.4 索引覆盖

```text
要查询的字段已经在索引中了，避免了回表
```

### 4.5 最左前缀匹配

```text
最左优先，在检索数据时从联合索引的最左边开始匹配

联合索引为(a, b, c)时，(a), (a, b), (a, b, c)会走索引
(a, c)会走a字段索引，不走c字段

如果索引是(a, b)，使用 where b=1 and a = "aaa"，MySQL查询优化器也会自动调整顺序

如果索引是(a, b, c)，使用 where a=1 and b>10 and c="aa"
    这样mysql向右匹配到范围查询后停止匹配
    a b 走完索引后，到达 c 时已是无序，c不再走索引

联合索引的索引树是以左边第一个字段作为非叶子结点，按照顺序进行放置

最左前缀，即MySQL只能高效地使用索引的最左前缀列
```

## 5 SQL语句优化

- 减少IO次数，IO数据量

DDL数据定义，DML数据操纵，DCL数据控制，DQL数据查询

### 5.1 SQL执行顺序

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

### 5.2 避免索引失效

### 5.3 避免使用 SELECT *

控制查询的字段，避免浪费资源(内存，cpu，网络)

SELECT * 容易产生回表

### 5.4 子查询小表驱动大表

```sql
SELECT id FROM orders
WHERE orders.user_id IN (SELECT id FROM users WHERE users.status=1)

SELECT id FROM orders
WHERE EXISTS (SELECT 1 FROM users WHERE orders.user_id = users.id AND users.status=1)
```

IN 适合左大表，右小表，优先执行IN里的子查询，如果IN里的数据量少，速度会较快

EXISTS 适合左小表，右大表，先查询左边语句，作为条件去和右边语句匹配

### 5.5 LIMIT

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

### 5.6 优化分页

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

### 5.7 控制索引数量

索引过多，占用存储空间。影响写操作性能

### 5.8 选择合适的字段类型

- char varchar
- 能用数字不用字符串
- 尽可能小的类型 bit, tINy INt
- 金额 decimal

### 5.9 分组查询

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

### 5.10 explaIN 索引优化

### 5.11 连接查询子查询

- 子查询：简单，结构化，但需要创建删除临时表

```sql
SELECT o.id FROM orders o
INNER JOIN users u ON o.user_id = u.id
WHERE
```

left joIN 时选择小表驱动大表

### 5.12 使用 union all 代替 union

union可以获取 排重后的数据，union all可以获取所有数据包含重复数据

排重过程需要遍历，排序，比较，更为耗费时间，资源

### 5.13 批量操作

批量插入

### 5.14 explaIN

查看SQL执行计划

explaIN 内容

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

## 6 SQL语句

### 6.1 分组 GROUP BY

### 6.2 排序 ORDER BY

### 6.3 HAVING

### 6.4 分页 LIMIT OFFSET

### 6.5 DISTINCT

结合 GROUP BY 是用来优化 DISTINCT

### 6.6 联合 UNION

### 6.7 IN NOT IN

### 6.8 EXISTS NOT EXISTS
