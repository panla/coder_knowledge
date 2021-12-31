# README

## 容器配置

```yaml
version: "3.9"
services:

  emqx:
    image: emqx/emqx:4.3.1
    container_name: dev_study_emqx
    volumes:
      - ./conf/emqx/emqx.conf:/opt/emqx/etc/emqx.conf
      - ./conf/emqx/vm.args:/opt/emqx/etc/vm.args
      - ./conf/emqx/emqx_auth_mnesia.conf:/opt/emqx/etc/plugins/emqx_auth_mnesia.conf
      - ./conf/emqx/loaded_plugins:/opt/emqx/data/loaded_plugins
    restart: always
    ports:
      - 8753:1883
    networks:
      custom_net:
        ipv4_address: "172.25.0.4"
    environment:
      - TZ=Asia/Shanghai

networks:
  custom_net:
    external: true
```

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

```

## emqx.conf

```text
# emqx.conf

# 禁止匿名用户
allow_anonymous = false
```
