# firewalld

## cmd

```bash
# 重启firewalld
sudo systemctl restart firewalld.service

# 查看开放的端口
sudo firewall-cmd --list-ports

# 开放端口
sudo firewall-cmd --add-port=10002/tcp --permanent --zone=public

# 关闭端口
sudo firewall-cmd --remove-port=10002/tcp --zone=public --permanent
```
