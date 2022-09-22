# 系统主题

<https://www.emqx.io/docs/zh/v4.4/advanced/system-topic.html>

`$SYS/brokers/{node}/` 开头

## tip

EMQX 默认只允许本机的 MQTT 客户端订阅 $SYS 主题，请参照 内置 ACL 修改发布订阅 ACL 规则。

设置系统消息周期
broker.sys_interval = 1m

## 客户端上下线

`$SYS/brokers/${node}/clients/{clientid}/connected`

```json
{
    "username": "foo",
    "ts": 1625572213873,
    "sockport": 1883,
    "proto_ver": 4,
    "proto_name": "MQTT",
    "keepalive": 60,
    "ipaddress": "127.0.0.1",
    "expiry_interval": 0,
    "connected_at": 1625572213873,
    "connack": 0,
    "clientid": "emqtt-8348fe27a87976ad4db3",
    "clean_start": true
}
```

`$SYS/brokers/${node}/clients/{clientid}/disconnected`

```json
{
    "username": "foo",
    "ts": 1625572213873,
    "sockport": 1883,
    "reason": "tcp_closed",
    "proto_ver": 4,
    "proto_name": "MQTT",
    "ipaddress": "127.0.0.1",
    "disconnected_at": 1625572213873,
    "clientid": "emqtt-8348fe27a87976ad4db3"
}
```

订阅 `SYS/brokers/+/clients/+/connected` add_callback
订阅 `SYS/brokers/+/clients/+/disconnected` add_callback

并设置 acl.conf

允许所有客户端订阅客户端状态
{allow, all, subscribe, ["$SYS/brokers/+/clients/#"]}.

## 系统告警
