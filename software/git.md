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
