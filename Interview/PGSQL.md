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
```
