# MongoDB

## 下载安装

[下载地址](https://www.mongodb.com/download-center/community)

## 解压配置

mongo.conf

```text
dbpath=/opt/mongodb/data/db
logpath=/opt/mongodb/logs/mongodb.log
pidfilepath=/opt/mongodb/logs/mongodb.pid

port=27017

auth=true

logappend=true
journal=true
quiet=true
```

systemctl管理 `/usr/lib/systemd/system/mongodb.service`

```text
[Unit]
Description=The MongoDB Server
After=network.target remote-fs.target nss-lookup.target

[Service]
# User=mongo
# Group=mongo

Type=forking
ExecStart=/opt/mongodb/bin/mongod -f /opt/mongodb/bin/mongo.conf -fork
ExecReload=/bin/kill -s HUP $MAINPID
ExecStop=/opt/mongodb/bin/mongod -f /opt/mongodb/bin/mongo.conf --shutdown
TimeoutStopSec=5
PrivateTmp=true

[Install]
WantedBy=multi-user.target
```

## docker-compose

```yaml
version: '3.9'
services:
  mongo01:
    image: mongo:4.4.5
    container_name: mongo01
    restart: always
    environment:
      - TZ="Asia/Shanghai"
      - LC_ALL=C.UTF-8
      - LANG=C.UTF-8
      - MONGO_INITDB_ROOT_USERNAME=admin
      - MONGO_INITDB_ROOT_PASSWORD=admin
    networks:
      mongo:
        ipv4_address: 172.19.0.3
    ports:
      - 127.0.0.1:27018:27017
    volumes:
      - ./db:/data/db
      - ./log:/var/log/mongodb

networks:
  mongo:
    external: true
```

```bash
docker-compose up -d
```
