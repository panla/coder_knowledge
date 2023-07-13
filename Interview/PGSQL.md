# PostgreSQL

[TOC]

## 1 与其他数据库的一些区别

```text
PG多进程模式，MySQL多线程模式

PG没有回滚段，MySQL有

PG BSD协议，MySQL双协议

PG支持物理备库

PG只支持堆表不支持索引组织表，MySQL InnoDB只支持索引组织表

PG事务默认 Read Committed，MySQL默认 Repeatable Read

PG块默认8KB，MySQL默认数据页16KB

PG较为完整的支持SQL2016
```

### 1.1 与MySQL区别

- 许可证
  - PG 采用 PostgreSQL许可证，类似于BSD/MIT
  - MySQL社区版采用GPL许可证
- 性能
- 索引
  - PG B树
  - MySQL InnoDB引擎采用B+树
- 对象层次结构
  - PG 实例，数据库，模式(schema)，表，列
  - MySQL 实例，数据库，表，列
- ACID事务
  - PG 支持 DML DDL 事务
  - MySQL 支持 支持 DML，DDL不能在另一个事务中
- 安全性
  - 支持 RBAC
- 复制
  - PG采用WAL物理复制，发布订阅逻辑复制
  - MySQL采用binlog逻辑复制
- JSON
  - PG JSON更丰富
- 易用性
  - MySQL，更宽松，允许GROUP BY语句的SELECT语句中包含非聚合列
  - PG，更严格，不允许GROUP BY语句的SELECT语句中包含非聚合列
  - MySQL 默认大小写不敏感
  - PG 默认大小写敏感
- 进程，线程
  - PG采用进程模型
  - MySQL采用线程模型
