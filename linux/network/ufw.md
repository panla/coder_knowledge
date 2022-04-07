# ufw

## command

```bash
# 查看状态
ufw status verbose
# To Action From
# 22 ALLOW IN Anywhere

# 更新 应用
ufw reload
```

## allow app

```bash
# 增加允许
ufw allow ssh

# 删除允许
ufw delete allow ssh
```

## allow port

```bash
# 允许 TCP/UDP 外部访问
ufw allow 8700

# 允许一段 TCP 端口
ufw allow 8700:8800/tcp

# 允许一段 UDP 端口
ufw allow 8700:8800/udp
```
