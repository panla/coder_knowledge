# 5 插件

## 5.1 loaded plugins 加载插件设置

`/opt/emqx/data/loaded_plugins`

```conf
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

## 5.2 plugins 命令

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
