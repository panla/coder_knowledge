# Redis 持久化 AOF

[toc]

## 逻辑

以日志形式记录每个操作，将redis执行过的所有指令记录下来（不记录读），只许追加但不可以改写文件，redis重启就会根据日志内容将写指令再执行一次

## 保存

aof -> 'appendonly.aof'

## 配置

- appendonly no 默认不开启，默认使用rdb，大部分情况rdb足够
- appendfilename "appendonly.aof"
  - appendfsync everysec 每秒同步
  - always 每次修改都
  - no 不同步 操作系统自己同步，速度最快
  - everysec 每秒同步
- no-appendfsync-on-rewrite no

## 恢复

- `redis-check-aof`
- `redis-check-rdb`

## 优缺点

优点：
1 可以每次修改都同步，完整性更好
缺点：
1 慢
2 效率低
