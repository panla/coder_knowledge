# use Redis

## usage

[Redis命令参考](http://redisdoc.com/index.html)

### String

#### String设置key-value

| command | 作用 |
| :-: | :-: |
| set key value | 设置key:value,会覆盖 |
| set key value nx | key不存在时才设置 |
| setnx key value | key不存在时才设置 |
| set key value xx | key存在时才设置 |
| setrange key offset value | 从offset开始用value覆盖 |
| mset key value [key value] | 同时设置多个值 |
| msetnx key value [key value] | 同时设置多个值,不存在时才设置 |

#### String设置过期时间

| command | 作用 |
| :-: | :-: |
| set key value ex seconds | 按秒设置过期时间 |
| setex key seconds value | 按秒设置过期时间 |
| set key value && expire key seconds | 按秒设置过期时间 |
| set key value px mseconds | 按毫秒设置过期时间 |
| psetex key mseconds value | 按毫秒设置过期时间 |
| set key value && pexpire key seconds | 按毫秒设置过期时间 |

#### String增长,返回现值,返回字符串类型,num值可正可负

| command | 作用 |
| :-: | :-: |
| incr key | value+=1 |
| incrby key num | value+=num |
| incrbyfloat key num | value+=num |
| decr key | value-=1 |
| decr key num | value-=num |
| mget key [ key ] | 同时获得多个 |

#### String其他

| command | 作用 |
| :-: | :-: |
| getset key value | 替换并返回旧值 |
| strlen key value | 返回字符串长度 |
| append key value | 在后面添加value |
| getrange key start end | [start: end] |
