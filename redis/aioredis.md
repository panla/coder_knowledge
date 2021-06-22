# Python AioRedis

## list

```text
blpop(key, *keys, timeout=0, encoding=_NOTSET)
    从队列左侧移出一个元素，会阻塞
brpop(key, *keys, timeout=0, encoding=_NOTSET)
    从队列右侧移出一个元素，会阻塞

lpop(key, *, encoding=_NOTSET)
    从队列左侧移出一个元素

rpop(key, *, encoding=_NOTSET)
    移除队列中最后一个元素，最右侧

lpush(key, value, *values)
    从队列左侧推进一个元素
lpushx(key, value)
    仅当 key 存在时，把 value 放进队列左侧头部

rpush(key, value, *values)
rpushx(key, value)

rpoplpush(sourcekey, destkey, timeout=0, encoding=_NOTSET)
    从 sourcekey 右侧尾部抛出一个元素，推进 destkey 左侧头部
brpoplpush(sourcekey, destkey, timeout=0, encoding=_NOTSET)
    从 sourcekey 右侧尾部抛出一个元素，推进 destkey 左侧头部，会阻塞

lindex(key, index, *, encoding=_NOTSET)
    返回一个元素在队列中的左侧 index

linsert(key, pivot, value, before=False)
    在 pivot 前/后 插入 value

llen(key)
    返回队列的长度

lrange(key, start, stop, *, encoding=_NOTSET)
    返回指定 index 范围的队列

lrem(key, count, value)
    从队列中移除 count 个 value

bzpopmax(keys, )
```

## string

```text
append(key, value)

decr(key) i += 1
decrby(key, decrement) i += decrement
incr(key) i -= 1
incrby(key, increment) i -= increment

get(key)
set(key, value, *, expire=0, pexpire=0, exist=None)

getbit(key, offset)
setbit(key, offset, value)

getrange(key, start, end)
getset(key, value)
    Set the string value of a key and return its old value

mget(key, *keys)
mset(*args) {'': ''}, {'': ''}    {'':'', '':''}

setex(key, seconds, value)
psetex(key, milliseconds, value)

setnx(key, value)
msetnx(key, value, **pairs)
    仅当 key 不存在时，才执行

strlen
```

## stream

```text
xadd(stream, fields, message_id=b'*', max_len=None, exact_len=False)

xrange()

xread()
xread_group()

xgroup_create()
xgroup_destroy()

xdel()

xack()
xtrim()

xlen()

```
