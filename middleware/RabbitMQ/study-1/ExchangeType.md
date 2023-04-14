# ExchangeType

[TOC]

## types

fan fan in fan out

direct

topic

headers

## fan

把消息投递到所有和它绑定的队列

## direct

把消息投递到 binding key 完全匹配 routing key 的队列中

```text
生产者 -> Routing Key: "green" -> Direct Exchange "green, red, orange" -> Queues -> 消费者
只会走 "green"
```

## topic

把消息路由到 binding key 和 routing key 模式匹配的队列

```text
将路由键和某模式进行匹配

a.b.c

*.b.*
*.b.c
*.*.c
```

## headers
