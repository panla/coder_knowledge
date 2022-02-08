# ssh

## ssh 公私钥连接

```bash
ssh user@地址
ssh 别名
```

客户端 `~/.ssh/config`

```text
Host 别名
    HostName ip地址
    Port 端口
    User 远程用户名
    IdentityFile ~/.ssh/id_rsa 本机私钥
```

服务器 `/etc/ssh/sshd_config`

```text
Port 22
ListenAddress 0.0.0.0
ListenAddress ::

HostKey /etc/ssh/ssh_host_rsa_key
HostKey /etc/ssh/ssh_host_ecdsa_key
HostKey /etc/ssh/ssh_host_ed25519_key

# 是否允许root登录
PermitRootLogin no

# 是否允许公钥登录
PubkeyAuthentication yes
AuthorizedKeysFile %h/.ssh/authorized_keys

# 是否允许密码登录
PasswordAuthentication no
```

服务器 `~/.ssh/authorized_keys` 上加入公钥
