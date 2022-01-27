# use Redis

## usage

[Redis命令参考](http://redisdoc.com/index.html)

### Zset有序集合

有序集合依靠score来排序

#### Zset存入取出member,或其他数据

| command | 作用 |
| :-: | :-: |
| zadd key score member [[score member]] | 存入数据,会覆盖 |
| zscore key member | 返回member的score |
| zincrby key num member | member.score+=num |
| zrange key start stop [ withscores ] | 返回index in [start: stop]的members,小->大 |
| zrangebyscore key min max [ withscores ] [ limit offset count ] | 返回score in [min: max]的members,小->大 |
| zrangebyscore key -inf +inf | 取出所有值 |
| zrangebylex key min max [ limit offset count ] | 返回member min max 中 [表示闭区间,(表示开区间,+表示正无限,-表示负无限 |
| zrevrange key start stop [ withscores ] | 同zrange,大->小 |
| zrevrangebyscore key min max [ withscores ] [ limit offset count ] | 同zrangebyscore,大->小 |

withscores : 返回时带上score
要用 limit 时要带上 limit
需警惕能否反转

#### Zset返回其他数据

| command | 作用 |
| :-: | :-: |
| zcard key | 返回key的member数目 |
| zcount key min max | 返回score在[min: max]的member数量 |
| zrank key member | 返回member的index,index是按照score从小到大 |
| zrevrank key member | 同zrank,index是按照score从大到小 |
| zlexcount key min max | 返回数量, min max 中 [表示闭区间,(表示开区间,+表示正无限,-表示负无限 |

#### 其他

| command | 作用 |
| :-: | :-: |
| zrem key member [ member ]| 删除member |
| zremrangebyrank key start stop | 移除 index [start: stop] 的member |
| zremrangebyscore key min max | 移除 score [min: max] 的member |
| zremrangebylex key min max | 移除member min max 中 [表示闭区间,(表示开区间,+表示正无限,-表示负无限 |
