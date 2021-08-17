# Git

## 部分配置

- 生成公钥私钥

```bash
ssh-keygen -t rsa -C "email"
```

- 配置用户名,邮箱

```bash
git config --global user.email "email"
git config --global user.name "name"
git config --global core.quotepath false
git config --global pull.rebase false
```

- vim

```bash
git config --global core.editor "vim"
```

- 保存账户密码

```bash
git config --global credential.helper store
```

## 别名

```bash
git config --global alias.co checkout
git config --global alias.ci commit
git config --global alias.st status
```

## ssh config

```text
Host test
    HostName ip or server_name
    User user
    Port 22
    IdentityFile ~/.ssh/id_rsa
```

## 加速？

```text
https://www.ipaddress.com/

github.com

github.global.ssl.fastly.net

assets-cdn.github.com

avatars1.githubusercontent.com

====================================================
199.232.69.194 github.global.ssl.fastly.net

140.82.112.4 github.com

185.199.108.153 assert-cdn.github.com

185.199.108.133 avatars1.githubusercontent.com

====================================================

修改 /etc/resolv.conf 新增
nameserver 8.8.8.8
nameserver 8.8.4.4
```
