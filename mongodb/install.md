# MongoDB

## 下载安装

[下载地址](https://www.mongodb.com/download-center/community)

## 解压配置

mongo.conf `/opt/mongodb/bin/mongo.conf`

```text
dbpath=/opt/mongodb/data/db
logpath=/opt/mongodb/logs/mongodb.log
port=27017
logappend=true
journal=true
quiet=true
pidfilepath=/opt/mongodb/logs/mongodb.pid
```

systemctl管理 `/usr/lib/systemd/system/mongodb.service`

```text
[Unit]
Description=The MongoDB Server
After=network.target remote-fs.target nss-lookup.target

[Service]
User=mongo
Group=mongo

Type=forking
ExecStart=/opt/mongodb/bin/mongod -f /opt/mongodb/bin/mongo.conf -fork
ExecReload=/bin/kill -s HUP $MAINPID
ExecStop=/opt/mongodb/bin/mongod -f /opt/mongodb/bin/mongo.conf --shutdown
TimeoutStopSec=5
PrivateTmp=true

[Install]
WantedBy=multi-user.target
```
