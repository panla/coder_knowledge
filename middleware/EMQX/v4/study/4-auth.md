# 4 认证

## 4.0 校验

同时开启了 username-passwd 校验和 JWT 校验，而且 JWT 中校验 client_id==username
那么此时需要用 client_id 生成 token 并且 username 不能符合 username-passwd 校验，否则 passwd 不通过

## 4.1 mnesia 预设账户配置

配置文件：`etc/plugins/emqx_auth_mnesia.conf`

并在 `data/loaded_plugins` 中开启 `emqx_auth_mnesia`

```conf
auth.user.2.username = user_1
auth.user.2.password = user_1
```

## 4.2 jwt

配置文件：`etc/plugins/emqx_auth_jwt.conf`

并在 `data/loaded_plugins` 中开启 `emqx_auth_jwt`

```conf
# 秘钥
auth.jwt.secret = SZjj5S8NgOcuq8LH+bAtDw

# 从 password 处读取 token
auth.jwt.from = password

# 校验字段
auth.jwt.verify_claims = on

# 比对 username = token 中的 username
auth.jwt.verify_claims.username = %u
# auth.jwt.verify_claims.client_id = %u
```

## 4.3 http

配置文件：`etc/plugins/emqx_auth_http.conf`

并在 `data/loaded_plugins` 中开启 `emqx_auth_http`

```conf
# EMQX 在设备连接事件中使用当前客户端相关信息作为参数，向用户自定义的认证服务发起请求查询权限，通过返回的 HTTP 响应状态码 (HTTP statusCode) 来处理认证请求
# 认证成功：API 返回200

# 请求地址
auth.http.auth_req = http://127.0.0.1:8000/api/v1/users/auth
# 请求方法：POST
auth.http.auth_req.method = post
# 请求头
auth.http.auth_req.headers.content-type = application/json
# 请求参数
## %u username, %c clientid, %a client ip, %P password, %r client 协议
auth.http.auth_req.params = clientid=%c,username=%u,password=%P

```
