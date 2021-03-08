# PGSQL Config

## 1 配置项 postgresql.conf

### 1.1 配置文件配置

```conf
# initdb 和 pg_ctl start 时的 -D 参数
data_directory = 'ConfigDir'

# 指定基于主机认证配置文件
hba_file = 'ConfigDir/pg_hba.conf'

# 指定用于用户名称映射的配置文件
ident_file = 'ConfigDir/pg_ident.conf'

# 指定可被服务器创建的用于管理程序的额外进程 ID（PID）文件
external_pid_file = ''
```

### 1.2 连接配置

```conf
# 指定服务器在哪些TCP/IP地址上监听客户端连接，值的形式是一个逗号分隔的主机名和/或数字 IP 地址列表
# * 指 所有可用IP接口，0.0.0.0 允许监听所有IPv4地址
listen_addresses = 'locahost'

# TCP 端口号
port = 5432

# 数据库最大并发连接，默认100
max_connections = 1000

# 使用默认值
superuser_reserved_connections = 3

# 直接编译在了源码中，默认是/tmp，如果修改了需要
# a export PGHOST=/opt/pgsql/tmp
# b ln -s opt/pgsql/tmp/.s.PGSQL.nnnn /tmp/.s.PGSQL.5432
unix_socket_directories = '/opt/pgsql/tmp'

# unix socket 所属组，默认空字符串=服务器用户的默认组，windows 无
unix_socket_group = 'postgres'

# unix socket 权限
unix_socket_permissions = 0770
```

### 1.3 TCP 安全和认证配置

```conf
tcp_keepalives_idle = 120

tcp_keepalives_interval = 120

# 允许完成客户端认证的最长时间，默认1分钟，1s--600s
authentication_timeout = 1min

# 加密方式 md5 or scram-sha-256
password_encryption = scram-sha-256
```

### 1.4 内存配置

```conf
# 数据库服务器将使用的共享内存缓冲区量，默认128MB
# 可以是系统内存的25%
shared_buffers = 256MB

# 为每个数据库会话设置临时缓冲区的最大内存，默认8MB
temp_buffers = 8MB
```

### 1.5 磁盘

```conf
# 指定一个进程能用于临时文件的最大磁盘空间量，默认-1：没有限制
temp_file_limit = -1
```

### 1.6 内核资源使用

```conf
# 每个服务器子进程允许同时打开的最大文件数目，默认是 1000 个文件，最小64
max_files_per_process = 1000
```

### 1.7 Background Writer

后台写入器

```conf
# 设置PostgreSQL可以同时被执行的并发磁盘 I/O 操作的数量
# 1--1000
effective_io_concurrency = 1

# 与 effective_io_concurrency 类似，但支持多客户端会话完成的维护工作
maintenance_io_concurrency = 10

# 该系统能够支持的后台进程的最大数量，默认8
max_worker_processes = 8

# 设置系统为并行操作所支持的工作者的最大数量。默认值为8
max_parallel_workers = 8
```

### 1.8 预写日志 WRITE-AHEAD LOG

### 1.9 检查点

```conf
# 自动WAL检查点之间的最长时间，默认5分钟，30-1d
checkpoint_timeout = 5min
```

### 1.10 在哪记录日志

```conf
# 记录服务器消息的方法，stderr,csvlog,syslog
log_destination = stderr

# 启用日志收集器，stderr 时用到，默认 off
logging_collector = on

# 当logging_collector 被启用时，决定log文件目录，默认log
log_directory = 'log'

# 当logging_collector 被启用时，设置被创建的日志文件的文件名
; log_filename = 'postgresql-%Y-%m-%d_%H%M%S.log'
log_filename = 'postgresql-%Y-%m-%d.log'

# log文件mod，默认 0600
log_file_mode = 0600

# 当logging_collector 被启用时，决定使用单个日志文件的最大时间量
# 0 代表禁用基于时间创建
log_rotation_age = 1d

# 当logging_collector 被启用时，决定一个个体日志文件的最大尺寸
# 0 代表禁用基于大小创建
log_rotation_size = 10MB
```

### 1.11  Statement Behavior

```conf
statement_timeout = 200000
idle_in_transaction_session_timeout = 200000
```

## 客户端认证 2 pg_gba.conf

### pg_hba.conf文件

[文档链接](http://www.postgres.cn/docs/13/auth-pg-hba-conf.html)

```conf
# "local" is for Unix domain socket connections only
local   all             all                                     trust
# IPv4 local connections:
host    all             all             127.0.0.1/32            trust
# IPv6 local connections:
host    all             all             ::1/128                 trust
# Allow replication connections from localhost, by a user with the
# replication privilege.
local   replication     all                                     trust
host    replication     all             127.0.0.1/32            trust
host    replication     all             ::1/128                 trust
```

## 进程

```text
postgres                                主进程和服务进程
postgres: checkpointer                  检查点进程
postgres: stats collector               统计数据收集进程
postgres: background writer             后台写进程
postgres: autovacuum launcher           自动清理进程
postgres: logical replication launcher
```

## 修改

postgresql.conf

```conf
listen_addresses = 'locahost'

# TCP 端口号
port = 5432

# 数据库最大并发连接，默认100
max_connections = 2000

# 直接编译在了源码中，默认是/tmp，如果修改了需要
# a export PGHOST=/opt/pgsql/tmp
# b ln -s opt/pgsql/tmp/.s.PGSQL.nnnn /tmp/.s.PGSQL.5432
unix_socket_directories = '/opt/pgsql/tmp'

# unix socket 所属组，默认空字符串=服务器用户的默认组，windows 无
unix_socket_group = 'postgres'

# unix socket 权限
unix_socket_permissions = 0770

# TCP
tcp_keepalives_idle = 120

tcp_keepalives_interval = 120

# 允许完成客户端认证的最长时间，默认1分钟，1s--600s
authentication_timeout = 1min

# 加密方式 md5 or scram-sha-256
password_encryption = scram-sha-256

# 内存
# 数据库服务器将使用的共享内存缓冲区量，默认128MB
# 可以是系统内存的25%
shared_buffers = 256MB

# 为每个数据库会话设置临时缓冲区的最大内存，默认8MB
temp_buffers = 8MB

# 日志
# 记录服务器消息的方法，stderr,csvlog,syslog
log_destination = stderr

# 启用日志收集器，stderr 时用到，默认 off
logging_collector = on

# 当logging_collector 被启用时，决定log文件目录，默认log
log_directory = 'log'

# 当logging_collector 被启用时，设置被创建的日志文件的文件名
; log_filename = 'postgresql-%Y-%m-%d_%H%M%S.log'
log_filename = 'postgresql-%Y-%m-%d.log'

# log文件mod，默认 0600
log_file_mode = 0600

# 事务连接收回
statement_timeout = 200000
idle_in_transaction_session_timeout = 200000
```

```sql
select rolname,rolpassword from pg_authid;

-- 修改密码
alter role postgres with password '';
```
