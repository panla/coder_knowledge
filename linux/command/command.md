# 命令

## firewalld

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

## ps 命令

找到僵尸进程父进程，并杀死

```bash
ps -ef | grep defunct | more | grep -v grep | cut -c 15-20 | xargs kill -9
```

批量杀死进程

```bash
ps -ef | grep /opt/interface_c/venv/bin/python3.5 | grep -v grep | cut -c 9-15 | xargs kill -9
```
