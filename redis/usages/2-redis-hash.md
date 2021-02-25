# use Redis Hash

## usage

[Redis命令参考](http://redisdoc.com/index.html)

相比 String 一般多个H/h
返回值,一般成功返回1,失败返回0

### 设置key

| command | 作用 |
| :-: | :-: |
| hset key filed value [field value] | 设置key:{field:value},会覆盖 |
| hsetnx key field value | field 不存在时才设置,key不存在则新建 |
| hmset key filed value [field value] | 设置多个k-v |

hmset和hset有什么区别？好像都可以设置多个值

### 获得

| command | 作用 |
| :-: | :-: |
| hget key field | 获得field.value |
| hmget key field [ field ] | 可以获得多个field.value |
| hkeys key | 所有field |
| hvals key | 获取所有的field.value |
| hgetall key | 获取所有的field, field.value |

### 增长

| command | 作用 |
| :-: | :-: |
| hincrby key field num | field.value+=num |
| hincrbyfloat key field num | field.value+=num |

### 判断

| command | 作用 |
| :-: | :-: |
| hexists key field | 检查field 是否存在于key中 |

### 其他

| command | 作用 |
| :-: | :-: |
| hdel key field [ field ] | 删除field,全删的话,key也会消失 |
| hlen key | 返回有多少个field |
| hstrlen key field | 返回field.value.length |
