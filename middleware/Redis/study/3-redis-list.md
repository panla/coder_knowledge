# use Redis List

[TOC]

## usage

[Redis命令参考](http://redisdoc.com/index.html)

消息队列，lpush rpop, 栈 lpush lpop

### List存入value,或其他数据

| command | 作用 |
| :-: | :-: |
| lpush key [ element ] | 从左侧插入数据,最后一个数据在最左侧 |
| lpushx key [ element ] | 在key存在的前提下,同lpsuh |
| rpush key [ element ] | 从右侧插入数据,最后一个数据在最右侧 |
| rpushx key [ element ] | 在key存在的前提下,同rpsuh |
| lset key index element | 把index位置的元素替换成element |
| ltrim key start end | 只保留[start: end]的数据 |

### List取出value,或其他数据

| command | 作用 |
| :-: | :-: |
| lrange key start end | key[start: end] |
| lpop key | 从key中抛出最左侧数据 |
| rpop key | 从key中抛出最右侧数据 |
| rpoplpush source destination | rpop source && lpush destination element |
| lrem key count value | coun>0从左侧,count<0从右侧,count=0,移除全部 |
| llen key | key的长度,元素个数 |
| lindex key index | 根据index从左取值 |
| linsert key before \| after pivot value | 把value插入到pivot前\|后 |

rpoplpush source destination 原子性地返回并移除存储在 source 的列表的最后一个元素 并把该元素放入存储在 destination 的列表的第一个元素位置

### 其他

| command | 作用 |
| :-: | :-: |
| blpop | 阻塞lpop |
| brpop | 阻塞rpop |
| brpoplpush | 阻塞rpoplpush |
