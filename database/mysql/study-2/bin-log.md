# NySQL Log

[toc]

## 1 Bin Log

### 1.1 介绍

主要用于 **主从复制** **数据恢复**

5.7.7 之后默认 ROW 格式，可以通过 `binlog-format` 指定

### 1.2 配置

```conf
# bin-log，二进制日志，默认开启
log-bin = /opt/mysql/logs/bin-logs/binlog
# bin log 每个文件的最大大小
max_binlog_size = 500M
```

### 1.3 bin log 恢复
