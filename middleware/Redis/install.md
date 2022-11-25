# Redis

[toc]

## 下载安装

[下载地址](https://redis.io/download)

```bash
wget http://download.redis.io/releases/redis-6.0.6.tar.gz
tar xzf redis-6.0.6.tar.gz
mv redis-6.0.6 redis
cd redis
make PREFIX=/home/opt/redis
make install PREFIX=/home/opt/redis
```

## 部分配置项

```text
tcp-keepalive 60
225 daemonize yes
247 pidfile /home/opt/redis/tmp/redis_6379.pid
261 logfile /home/opt/redis/logs/redis_6379.log
789 requirepass password
# save RDB 文件同步频率
# maxclients 最大连接数
# maxmemory 占用内存大小
# appendpnly 开启AOF备份
# appendfsync AOF同步频率 no|everysec|always
```

## 权限

dump.rdb 文件的权限

## redis.service

```text
[Unit]
Description=Redis
After=network.target

[Service]
Type=forking
ExecStart=/home/opt/redis/bin/redis-server /home/opt/redis/redis.conf
ExecReload=/home/opt/redis/bin/redis-server -s reload
ExecStop=/home/opt/redis/bin/redis-server -s stop
PrivateTmp=true

[Install]
WantedBy=multi-user.target
```
