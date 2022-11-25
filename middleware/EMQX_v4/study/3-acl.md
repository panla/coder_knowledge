# ACL

## 1 发布订阅 ACL

"允许(Allow)/拒绝(Deny)"  "谁(Who)"  "订阅(Subscribe)/发布(Publish)" "主题列表(Topics)"

- 内置 ACL
- Mnesia ACL
- 外部数据库 ACL
- HTTP ACL

### 1.1 内置 ACL

`etc/acl.conf`

规则从下至上加载

### 1.2 Mnesia ACL

`etc/plugins/emqx_auth_mnesia.conf`

允许 client or clientid 发布 `test/client` 主题

```json
{
    "username": "client",
    "clientid": "clientid",
    "topic": "test/client",
    "action": "pub",
    "access": "allow"
}
```

### 1.3 HTTP ACL

`etc/plugins/emqx_auth_http.conf`

```conf
auth.http.acl_req = http://127.0.0.1:8000//api/v1/users/acl
auth.http.acl_req.method = get
# POST 时为表单

# %A 操作类型，"1" 订阅，"2" 发布
# %u username, %c clientid, %a client ip, %r client 协议
# %m Mountpoint
# %t topic
auth.http.acl_req.params = access=%A,username=%u,clientid=%c,ipaddr=%a,topic=%t,mountpoint=%m
```

### 1.4 JWT ACL

`etc/plugins/emqx_auth_jwt.conf`

```conf
# 需要在 token 中指定 ACL 字段
auth.jwt.acl_claim_name = acl

{
    "acl": {
        "sub": [
            "some/topic/for/sub/1",
            "some/topic/for/sub/2"
        ],
        "pub": [
            "some/topics/for/pub/1",
            "some/topics/for/pub/2"
        ],
        "all": [
            "some/topics/for/pubsub/1",
            "some/topics/for/pubsub/2"
        ]
    }
}
```
