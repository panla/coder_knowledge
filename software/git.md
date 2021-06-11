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

## 别名

```bash
git config --global alias.co checkout
git config --global alias.ci commit
git config --global alias.st status
```

## 加速？

```text
https://www.ipaddress.com/

github.com

github.global.ssl.fastly.net

assets-cdn.github.com

avatars1.githubusercontent.com

查找后修改 /etc/hosts -> ip 域名

修改 /etc/resolv.conf 新增
nameserver 8.8.8.8
nameserver 8.8.4.4
```
