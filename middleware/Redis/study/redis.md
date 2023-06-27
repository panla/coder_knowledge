# use Redis

[TOC]

## 参考文档

<http://redisdoc.com/index.html>

## usage

### 服务

| 命令 | 说明 |
| :-: | :-: |
| ./src/redis-server ./redis.conf | 启动服务 |
| ./src/redis-cli -h host -p port | 启动进入客户端 |
| auth password | 验证 |

### key and db

| 命令 | 说明 |
| :-: | :-: |
| select db | 选择切换数据库 |
| move key db | 移动key到另一个数据库 |
| del key | 删除键 |
| randomkey | 随机返回一个key |
| rename key newkey | 如newkey 存在,会覆盖掉 |
| renamenx key newkey | newkey 不存在时修改 |
| exists key | key 是否存在 |
| type key | 返回key的value的类型 |

过期时间

| 命令 | 说明 |
| :-: | :-: |
| expire key seconds | 按秒设置 |
| expireat key timestamp | 按时间戳设置 |
| pexpire key milliseconds | 按毫秒设置 |
| persist key | 移除key的过期时间 |
| pttl key | 按毫秒返回剩余过期时间 |
| ttl key | 按秒返回剩余过期时间 |

不知所云

| 命令 | 说明 |
| :-: | :-: |
| keys pattern |  |
| scan sursor [macth pattern] [coun count] |  |
| dump key |  |

## 数据类型

### string

字符串,最大512MB

### hash

键值对,每个hash 可存储 2 ^ 32 - 1 键值对???

```text
HMSET key field value [field value]
HGET  key field
```

### list

列表,每个list 可存储 2 ^ 32 - 1 元素???

```text
lpush key element [element]
lrange key 0 10
```

### set

集合,唯一且无序 通过哈希表实现

```text
sadd key member
  成功返回1,已存在返回0
smemmers key
```

### zset

有序集合,唯一,会关联一个double类型的分数,根据分数进行排序

```text
zadd key score member
  增加或更新元素
zrangebyscore key score1 score2
```
