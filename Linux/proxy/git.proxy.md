# Git Proxy

```bash
# 查看
git config --global --list

# 设置
git config --global http.proxy http://ip:port
git config --global https.proxy https://ip:port

git config --global http.proxy "socks5://ip:port"

# 取消
git config --global --unset http.proxy
```
