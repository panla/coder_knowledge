# README

[toc]

## 1 容器配置

[docker-compose](./docker/docker-compose.yml)

## 2 emqx.conf

### 2.1 禁止匿名用户

```text
allow_anonymous = false
```

### 2.2 SSL/TLS

```text
## https
listener.ssl.external = 8883

listener.ssl.external.key_password = yourpass
listener.ssl.external.keyfile = etc/certs/server-key.pem
listener.ssl.external.certfile = etc/certs/server-cert.pem
listener.ssl.external.cacertfile = etc/certs/ca-cert.pem
listener.ssl.external.dhfile = etc/certs/dh-params.pem


## wss
listener.wss.external = 8084
listener.wss.external.mqtt_path = /mqtt

listener.wss.external.keyfile = etc/certs/key.pem
listener.wss.external.certfile = etc/certs/cert.pem
listener.wss.external.cacertfile = etc/certs/cacert.pem
listener.wss.external.key_password = yourpass

##
listener.ssl.external.dhfile = etc/certs/dh-params.pem
```

## 3 loaded plugins 加载插件设置

```text
# loaded_plugins

{emqx_management, true}.
{emqx_dashboard, true}.
{emqx_modules, false}.
{emqx_recon, true}.
{emqx_retainer, true}.
{emqx_telemetry, true}.
{emqx_rule_engine, true}.
{emqx_bridge_mqtt, false}.
{emqx_auth_mnesia, true}.
{emqx_auth_jwt, true}.

```

## 4 认证

### 4.1 mnesia 预设账户配置

```text
# emqx_auth_mnesia.conf

auth.user.2.username = user_1
auth.user.2.password = user_1
```

### 4.2 jwt

```text
# emqx_auth_jwt.conf

auth.jwt.secret = SZjj5S8NgOcuq8LH+bAtDw

auth.jwt.from = password

auth.jwt.verify_claims = on

auth.jwt.verify_claims.username = %u
# auth.jwt.verify_claims.client_id = %u
```

## 5 命令

### 5.1 plugins

```bash
# 列出所有插件
./bin/emqx_ctl plugins list

# 加载插件
./bin/emqx_ctl plugins load emqx_auth_jwt

## 卸载插件
./bin/emqx_ctl plugins unload emqx_auth_jwt

# 重载插件
./bin/emqx_ctl plugins reload emqx_auth_jwt
```

### 5.2 cluster

```bash
# 查询状态
./bin/emqx_ctl cluster status

# 加入集群
./bin/emqx_ctl cluster join emqx@other

# 离开集群
./bin/emqx_ctl cluster leave

# 删除节点
./bin/emqx_ctl cluster force-leave emqx@this
```
