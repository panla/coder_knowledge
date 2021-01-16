# ps 命令

## 使用

查找僵尸进程

```bash
ps -ef | grep defunct | more
```

批量杀死进程

```bash
ps -ef | grep /opt/interface_c/venv/bin/python3.5 | grep -v grep | cut -c 9-15 | xargs kill -9
```
