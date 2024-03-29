# Redis

[TOC]

## 1 数据结构，用途场景

string list ziplist set zset hash

geo bitmap stream

```text
string
    简单动态字符串
    分布式锁，
    简单缓存
    计数器

list
    字符串列表，双向链表
    栈，左进左出
    队列，左进右出
    阻塞队列，左进，右阻塞出 brpop
    列表展示

ziplist

hash
    存储对象
    key:{key:value}

set
    交集并集差集
    共同关注
    抽奖
    点赞数

zset
    排行榜
```

```text
静态资源

user token

消息队列

分布式锁

数据库数据缓存

自增，浏览数
```

## 2 Redis速度为什么快

- 内存存储
- 非阻塞式I/O，epoll多路复用，处理并发连接
- 单线程执行指令避免线程切换和竟态产生的消耗

## 3 缓存击穿，雪崩，穿透

### 3.1 击穿

key 过期或者其他因素不存在，大量请求到达数据库，影响数据库服务

- 1，延长过期时间，永不过期
- 2，互斥锁，

### 3.2 雪崩

Redis服务挂掉，集群失效，缓存失败，多个key因为某种因素不存在，大量请求到达数据库，影响数据库服务

- 1，缓存高可用
- 2，二级缓存，本地缓存
- 3，随机过期
- 3，Redis备份，预热

### 3.3 穿透

查询在缓存和数据库中注定不存在的数据，影响数据库服务

- 1，验证拦截，校验参数
- 2，缓存空数据
- 3，过滤器，布隆过滤器，不存在时一定不存在

## 4 key处理

big key，hot key 服务性能下降，用户体验变差，引发大面积故障

### 4.1 big key

value 较大数据或含有较多成员元素的 key

#### 4.1.1 特征

- Key本身的数据量过大：一个String类型的Key，它的值为5 MB
- Key中的成员数过多：一个ZSET类型的Key，它的成员数量为10,000个
- Key中成员的数据量过大：一个Hash类型的Key，它的成员数量虽然只有1,000个但这些成员的Value（值）总大小为100 MB

#### 4.1.2 危害，Redis核心工作是单线程

- 客户端执行命令的时间变长
- 读写请求导致带宽占用高，服务变慢
- 内存占用高，引发操作阻塞，或其他key被淘汰，或内存溢出
- 删除操作可能造成主库较长时间阻塞，影响从库，或其他节点
- 集群模式下，某个数据分片的内存使用率远超其他数据分片，造成集群不平衡

#### 4.1.3 产生

过大，未拆分，只增不减

- 在不适合的场景下使用Redis或其中的数据类型，造成value过大，如使用string存储大体积二进制文件，或大的JSON数据
- 设计不佳，没有对key中的成员进行拆分，造成个别key中的成员数量过多
- 没有及时清理无效数据，造成hash类型中成员不断增加
- 使用 list 时逻辑错误只增不减

#### 4.1.4 找到

- 增加 内存/网络/超时等监控指标
- bigkey指令，redis-cli --bigkeys
- redis-rdb-tools 对 rdb 文件离线扫描

#### 4.1.5 处理

- 对big key拆分
- 删除 unlink 非阻塞删除
  - 缓存至其他存储中
  - 清理过期数据
- 压缩

### 4.2 hot key

一定时间内访问频次比较高的key，

#### 4.2.1 特征

- QPS集中在少部分key，某个key的访问频率明显高于其他key
- 带宽使用率集中在少部分key，hash 大量的 hgetall 请求
- CPU使用时间集中在少部分key，zset set zrange 请求

#### 4.2.2 危害

- 占用CPU资源，影响其他请求导致服务性能降低
- 集群模式下，hot key 大量访问，其他节点空闲，节点不平衡
- 抢购秒杀场景下，hot key请求过大，超出Redis处理能力，超卖
- hot key超出Redis承受能力，造成缓存击穿，大量请求到达数据库，造成数据库宕机，影响其他业务

#### 4.2.3 产生

- 预期外的数据访问量陡增，爆款商品，热点新闻，刷屏点赞
- 游戏人物聚集

#### 4.2.4 找到

- 增加 内存/网络/超时等监控指标
- hotkeys指令，redis-cli --hotkeys

#### 4.2.5 处理

- 复制key到其他节点，缓解分片压力
- 1，收集上报，客户端或代理层去收集，统计key的访问次数
- 2，定时扫描，redis-cli --hotkeys

## 5 过期策略，内存淘汰策略

### 5.1 过期策略，定期删除，惰性删除

定期扫描key的过期时间，随机抽取key

访问时去判断需不需要删除

### 5.2 内存淘汰策略

- 1 noeviction 当内存使用超出配置的时候返回错误
- 2 allkeys-lru 加入key的时候，LRU算法淘汰最久未使用的key
- 3 volatile-lru 加入key的时候，从有过期时间的keys中淘汰最久未使用key
- 4 allkeys-random 随机
- 5 voliatile-ttl 从有过期时间的keys中淘汰即将过期的keys
- 6 allkeys-lfu 从所有的keys中淘汰使用频率低的key
- 7 voliatile-lfu 从有过期时间的keys中淘汰使用频率的key

LRU 最久未使用

哈希链表

```text
add key 时在链表尾部 append node，如果超出阈值，就淘汰队头node

update key 时修改对应节点的值，在移至队尾

访问 key 时，把访问的节点移至队尾
```

LFU 最近最少使用

```text
LRU的最近最少使用实际上并不精确

get 时返回 value

get/put 时，频率+1

容量到达阈值时，把频率最低的key删除，如果此频率对应多个key，删除最旧的key

```

## 6 持久化 RDB AOF

### 6.1 RDB

把内存数据以快照形式保存在硬盘上

持久化机制

- save，会阻塞当前服务，影响其他指令，直到RDB完成
- bgsave，后台异步进行快照操作，fork一个子进程去处理
- 自动，一定时间内数据写操作一定次数触发持久化机制

优缺点

- 文件紧凑，全量备份，较适合备份和恢复
- 恢复大数据集时速度比AOF更快
- 实时性较差
- 父进程修改数据时，子进程不能响应，可能存在丢失数据

### 6.2 AOF

追加写入写命令到文件，日志记录

触发机制

- always 每次修改都记录
- everysec 每秒同步，每秒记录，如果一秒内宕机，可能会有数据丢失
- no 不同步

优缺点

- 实时性较好，数据较为完整
- 比 RDB更为可靠
- 持久化文件越来越大，bgrewriteaof 把内存中的数据以命令的方式写入临时文件，再fork子进程去重写AOF文件

### 6.3 混合模式

`aof-use-rdb-preamble`

- 1 fork出子进程把当前全量数据RDB写入AOF中，
- 2 再把AOF重写缓冲区的增量命令以AOF方式写入文件，
- 3 写入完成后通知主进程将新的含有RDB和AOF格式的AOF文件替换旧的AOF文件

优缺点

- 大部分属RDB格式数据，加载速度快
- 结合AOF，减少数据丢失
- 兼容性，可读性不好

## 7 高可用

### 7.1 Redis cluster

## 8 锁

## 9 一致性

### 9.0

- 强一致性
- 弱一致性
- 最终一致性，一定时间内达到数据一致性

### 9.1 与数据库双写一致性

#### 9.1.1 缓存延时双删

- 1，删除缓存
- 2，更新数据库
- 3，休眠，缓存如果存在就删除缓存

休眠时间 = 读业务逻辑耗时 +几百毫秒

缺点：降低了写入性能

#### 9.1.2 删除缓存重试

第二删除失败可能导致脏数据

- 1，更新数据库
- 2，删除缓存失败
- 3，把失败的key放入消息队列
- 4，消费消息队列消息，获取需要删除的key
- 5，重试删除缓存

缺点：复杂度提高，代码入侵

#### 9.1.3 binlog 删除缓存

canal 中间件，把binlog日志采集发送到消息队列中，消费消息队列消息，获取需要删除的key，删除缓存

#### 9.1.4

缓存和数据库同时有数据时，有写操作，先操作数据库，在操作缓存

更新数据库，删除缓存

## 10 事务

概念

Redis事务是一组命令的集合，一次执行多个命令，事务内的命令被序列化，串行执行，
其他客户端提交的命令请求不会插入到事务中

- 没有隔离级别
- 不保证原子性，单条命令原子性执行，事务不保证原子性，且不回滚，其中命令失败，其余命令依然执行

事务阶段

- 开始事务
- 命令入队
- 执行事务

相关命令

```text
watch key, key
    监视key，在事务执行之前，被监视的key发生更新，则事务被打断
multi
    标记事务开始
exec
    执行，一旦执行，之前加的watch失效
discard
    取消事务
unwatch
    取消watch

```

案例

```text
正常执行

放弃事务

命令错误，所有都不执行

语法错误，其他命令被执行，错误命令抛出异常

watch
```
