# NySQL Log

[toc]

## 1 Bin Log

### 1.1 介绍

Server 层日志，记录了对MySQL数据库执行更改的所有的写操作

主要用于 **主从复制** **数据恢复**

- 主从复制：Master 开启 binlog，然后将 binlog 发送到各个 Slave，Slave 重放 binlog 操作
- 数据恢复：mysqlbinlog 工具来恢复数据

### 1.2 日志数据格式

5.7.7 以后，ROW，通过 binlog-format 指定，STAMENT, MIXED

ROW：基于行的复制，仅记录哪条数据被修改了

### 1.2 配置

```conf
# bin-log，二进制日志，默认开启
log-bin = /opt/mysql/logs/bin-logs/binlog
# bin log 每个文件的最大大小
max_binlog_size = 500M
```

### 1.3 bin log 恢复

## 2 redo log

### 2.1 介绍

引擎层日志，宕机也可以把已经提交的事务持久化到硬盘中

## 3 undo log

引擎层日志，事务可以回滚从而保证事务操作原子性
