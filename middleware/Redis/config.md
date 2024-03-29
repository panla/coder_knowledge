# Redis config

[TOC]

## 配置项

- 网络配置
  - bind 127.0.0.1 绑定的ip
  - `protected-mod` yes 保护模式
  - port 6379 端口
- 通用
  - daemonize yes 守护进程运行
  - pidfile `redis_6379.pid` 如果以后台方式运行需要指定 pid 文件
  - loglevel debug/verbose/notice/warning 日志级别 notice
  - logfile "" 日志文件名
  - databases 16 默认16个数据库
- SNAPSHOTTING 快照
  - save 900 1      900 秒内至少1个key进行过修改，就进行持久化操作
  - save 300 10     300 秒内至少10个key进行过修改，就进行持久化操作
  - save 60 100000  60  秒内至少10000个key进行过修改，就进行持久化操作
  - `stop-writes-on-bgsave-error` yes 持久化出错继续工作
  - rdbcompression yes 是否压缩rdb文件
  - rdbnchecksum yes 保存rdb文件出错时进行错误检查校验
  - dir ./ rdb 文件保存目录
- REPLICATION 主从复制
- SECURITY 安全
  - requirepass 密码
- CLIENTS 客户端
  - maxclients 10000 设置能连上redis的最大客户端数量
  - maxmemory `<bytes>` redis 配置的内存容量
  - maxmemory-policy 内存到达上限后的处理策略
    - `volatile-lru` 只对设置了过期时间的key进行LRU
    - `allkeys-lru` 删除lru算法的key
    - `volatile-random` 随机删除即将过期ke
    - `allkeys-random` 随机删除
    - noeviction 永不过期
- APPEND ONLY MO 持久化策略 AOF
  - appendonly no 默认不开启，默认使用rdb，大部分情况rdb足够
  - appendfilename "appendonly.aof"
  - appendfsync everysec 每秒同步
    - always 每次修改都
    - no 不同步 操作系统自己同步，速度最快
    - everysec 每秒同步
