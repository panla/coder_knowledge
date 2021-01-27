# use Redis

## usage

[Redis命令参考](http://redisdoc.com/index.html)

### Set

#### Set存入取出member,或其他数据

| command | 作用 |
| :-: | :-: |
| sadd key member [ member ] | 把member添加入key,不重复 |
| smembers key | 返回key的所有members |
| spop key | 随机抛出一个member |
| srandmember key | 随机抛出一个member 不删除 |

#### 交并差集

| command | 作用 |
| :-: | :-: |
| sinter key [ key ] | 返回指定key的交集 |
| sinterstore target key [ key ] | 把指定key的交集放在target |
| sunion key [ key ] | 返回指定key的并集 |
| sunionstore target key [ key ] | 把指定key的并集放在target |
| sdiff key [ key ] | 返回指定key的差集 |
| sdiffstore target key [ key ] | 把指定key的差集放在target |

#### 其他

| command | 作用 |
| :-: | :-: |
| srem key member [ member ] | remove member |
| smove source target member | 把member从source移动到target |
| scard key | return count (members) |
| sismember key member | member is in key ? |
