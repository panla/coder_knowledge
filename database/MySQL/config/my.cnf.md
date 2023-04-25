# MySQL config

## 服务器系统变量

Version: MySQL8.0

<https://dev.mysql.com/doc/refman/8.0/en/server-system-variables.html>

| System Variable | Default | Desc | Other |
| :-: | :-: | :-: | :-: |
| `bind_address` | * | 监听地址 `bind_address=0.0.0.0`  | 8.0.13以后可以多个，逗号分割 |
| `port` | 3306 | 服务器监听 TCP/IP 连接的端口 |  |

| `system_time_zone` |  | 时区 |  |
| `time_zone` | SYSTEM |  |  |
| `default_time_zone` | +08:00 |  |  |

| `basedir` | mysql parent dir | MySQL安装基本目录 |  |
| `datadir` |  | 数据目录, `datadir=/var/lib/mysql` |  |
| `log_error` |  | 错误日志文件，文件名 |  |
| `plugin_dir` | `BASEDIR/lib/plugin` | 插件目录 |  |
| `pid_file` |  | 写入进程ID的文件 |  |
| `socket` |  |  |  |
| `tmpdir` |  | 临时文件的目录 |  |
| `read_only` | OFF 0 | 只读 |  |

| `autocommit` | ON | 自动提交 | `autocommit=1` |
| `character_set_server` | utf8mb4 | 服务器默认字符集 |  |
| `collation_server` | `utf8mb4_0900_ai_ci` | 服务器默认排序规则 |  |
| `default_authentication_plugin` | `caching_sha2_password` | 密码, `mysql_native_password` |  |
| `default_storage_engine` | InnoDB | 默认存储引擎 |  |

| `interactive_timeout` | 28800 | 服务器在交互式连接关闭之前等待活动的时间 |  |
| `wait_timeout` | 28800 | 服务器在非交互式连接关闭之前等待活动的时间, 单位: 秒, MAX=31536000, MIN=1 |  |
| `connect_timeout` | 10 | mysqld服务器响应超时时间, 单位: 秒, Max=31536000, Min=2 |  |
| `net_read_timeout` | 30 | 读取超时, 单位: 秒, MAX=31536000, MIN=1 |  |
| `net_write_timeout` | 60 | 写入超时, 单位: 秒, MAX=31536000, MIN=1 |  |

| `join_buffer_size` | 262144 256K | 单位: 字节, Max=(2**64)-1, Min=128 |  |
| `key_buffer_size` | 8388608 8M | 索引缓冲, 单位: 字节, Max=OS_PER_PROCESS_LIMIT, Min=0 |  |
| `read_buffer_size` | 131072 128K | 读取缓冲区, 单位: 字节, MAX=2147479552 2G, MIN=8192 | 1M |
| `sort_buffer_size` | 262144 256K | 排序缓冲区, 单位: 字节, MAX=(2**64)-1, MIN=32768 32K | 2M |
| `tmp_table_size` | 16777216 16M | 内存中临时表的最大大小, 单位: 字节, MAX=(2**64)-1, MIN=1024 | 256M |
| `max_heap_table_size` | 16777216 16M | 用户创建的表增长到的最大大小, 单位: 字节, MAX=(2**64)-1, MIN=16384 16K |  |

| `long_query_time` | 10 | 超过此值认为是慢查询, 单位: 秒, Max=31536000, Min=0 |  |
| `slow_query_log` | OFF 0 | 启用慢查询日志 |  |
| `slow_query_log_file` | host_name-slow.log | 慢查询日志文件 |  |
| `log_slow_admin_statements` | OFF | 记录写入慢查询日志 | 更改表、分析表、检查表、创建索引、删除索引、优化表和修复表 |

| `max_connections` | 151 | 允许的最大并发客户端数, MAX=100000, MIN=1 | 3000 |
| `open_files_limit` | 5000 | 操作系统可用于 mysqld 的文件描述符数量, MAX=平台, MIN=0 | 10 + `max_connections` + `table_open_cache` * 2 |
| `table_open_cache` | 4000 | 所有线程的打开表数, MAX=524288, MIN=1 | max(open_files_limit - 10 - max_connections) / 2, 400) |
| `thread_cache_size` | -1 | 缓存多少线程以供重用, MAX=16384, MIN=0, -1=自动调整 | 8 + (max_connections / 100) |
| `thread_pool_size` | 16 | 线程池线程数, MAX=512(>=8.0.19)(64 <=8.0.19>), MIN=1 | 需要启用线程池插件 |

| `max_allowed_package` | 67108864 64M | 数据包最大大小, 单位: 字节, MAX=1073741824 1G, MIN=1024 |  |
| `sql_buffer_result` | OFF 0 | 把SELECT结果放入临时表 |  |

| `ssl_ca` |  |  |  |
| `ssl_cert` |  |  |  |
| `ssl_key` |  |  |  |
| `tls_ciphersuites` |  |  | TLSv1.2 TLSv1.3 |
| `tls_version` |  |  |  |
| `ssl_session_cache_timeout` | 300 | SSL Session 缓存时间, 单位: 秒, MAX=86400, MIN=0 |  |

```conf
[mysqld]
# ==========================================================================
server_id = 1
bind_address = 0.0.0.0
port = 3306

authentication_policy = mysql_native_password

default_time_zone=+08:00

character_set_server = utf8mb4
# character
init_connect = 'SET NAMES utf8mb4'
# order
collation_server = utf8mb4_0900_ai_ci


# ==========================================================================
# wait_timeout = 3600
# interactive_timeout = 3600
connect_timeout = 10
net_read_timeout = 30
net_write_timeout = 60


# ==========================================================================
# log
# ==========================================================================
# error log 开启后不再向 stdout 打印
# log_error = /var/log/mysql/error.log

# binlog
log_bin = /var/log/mysql/master-bin.log
# binlog index
# log_bin_index = /var/log/mysql/master-bin.log.index
# binlog 模式, DEFAULT=row
binlog_format = mixed
# 事务能够使用的最大 binlog 缓存空间 DEFAULT=32K MAX= MIN=4K
binlog_cache_size = 1M
# binlog 文件最大空间，达到该大小时切分文件 DEFAULT=1073741824 1G, MAX=1G, MIN=4K
max_binlog_size = 256M
# BINLOG 保存时间，秒数
binlog_expire_logs_seconds = 864000

# 启用慢查询日志
slow_query_log = ON
# 慢查询检测时间
long_query_time = 20
# 慢查询文件
slow_query_log_file = /var/log/mysql/slow.log
# 记录 更改表、分析表、检查表、创建索引、删除索引、优化表和修复表 慢查询
log_slow_admin_statements = ON


# ==========================================================================
# relay log
# ==========================================================================
relay_log = /var/log/mysql/relay.log
relay_log = /var/log/mysql/relay.log.index
# 从库从主库复制的数据写入从库 binlog 日志 DEFAULT=OFF
log_slave_updates = ON
# 最大 relay log size DEFAULT=0 MAX=1073741824 1G MIN=0
max_relay_log_size = 256M
# 自动清空不再需要的中继日志 DEFAULT=ON
relay_log_purge = ON
# 重启 slave 时删除所有 relay log, 通过 SQL 重放的位置点去重新拉取日志 DEFAULT=OFF
relay_log_recovery = ON

# ==========================================================================
# replica
# ==========================================================================
# 副本集类型
replica_parallel_type = LOGICAL_CLOCK
# 副本worker num
replica_parallel_workers = 4
# slave 上 commit 顺序保持一致
replica_preserve_commit_order = ON


# ==========================================================================
# 连接
# ==========================================================================
# 允许的最大并发客户端数, MAX=100000, MIN=1, DEFAULT=151
max_connections = 2000
# 操作系统可用于 mysqld 的文件描述符数量, MAX=平台, MIN=0, DEFAULT=5000
open_files_limit = 5000
# 所有线程的打开表数, MAX=524288, MIN=1, max(open_files_limit - 10 - max_connections) / 2, 400), DEFAULT=4000
table_open_cache = 4000


# ==========================================================================
# 缓冲区, 连接查询, 索引, 读取缓冲, 排序缓冲, 临时表, 用户表
# ==========================================================================
# DEFAULT 256K
join_buffer_size = 2M
# DEFAULT 8M
key_buffer_size = 64M
# DEFAULT 128K
read_buffer_size = 1M
# DEFAULT 256K
sort_buffer_size = 2M
# DEFAULT 16M
tmp_table_size = 256M
# DEFAULT 16M
max_heap_table_size = 256M


# ==========================================================================
read_only = OFF


# ==========================================================================
# 数据包
# ==========================================================================

# 数据包最大大小, 单位: 字节, MAX=1073741824 1G, MIN=1024
max_allowed_packet = 64M


# ==========================================================================
# InnoDB
# ==========================================================================
# 缓冲区 default=128M
innodb_buffer_pool_size = 256M
# 异步 I/O 子系统
# innodb_use_native_aio = NO
# 读线程数
innodb_read_io_threads = 16
# 写线程数
innodb_write_io_threads = 16
# 并行查询
innodb_parallel_read_threads = 16

```

slave

```conf
[mysqld]
# 从库只读
read_only = ON
```

GTID

```conf
[mysqld]
# 开启 GTID 同步 DEFAULT=OFF
gtid_mode = ON
# 强制事务一致 DEFAULT=OFF
enforce_gtid_consistency = ON
# 在MySQL启动或重启时搜索GTID时, binlog迭代方式 DEFAULT=ON
binlog_gtid_simple_recovery = ON
```

```sql
create user 'repuser'@'%' identified by 'repuser123';
grant replication slave on *.* to 'repuser'@'%';
```
