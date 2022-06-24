# README

[toc]

## 容器配置

[docker-compose](./docker/docker-compose.yml)

## mnesia 预设账户配置

```text
# emqx_auth_mnesia.conf

auth.user.2.username = user_1
auth.user.2.password = user_1
```

## loaded plugins 加载插件设置

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

## emqx.conf

```text
# emqx.conf

# 禁止匿名用户
allow_anonymous = false
```
