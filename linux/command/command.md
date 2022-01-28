# 命令

[toc]

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

## 查找文件

### locate 通过名字来查找文件

```bash
locate bin/zip
# locate 命令将会搜索它的路径名数据库，输出任一个包含字符串“bin/zip”的路径名

locate zip | grep bin
```

### find

```bash

```
