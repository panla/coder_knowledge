# ps 命令

## 使用

找到僵尸进程父进程，并杀死

```bash
ps -ef | grep defunct | more | grep -v grep | cut -c 15-20 | xargs kill -9
```

批量杀死进程

```bash
ps -ef | grep /opt/interface_c/venv/bin/python3.5 | grep -v grep | cut -c 9-15 | xargs kill -9
```
