# Docker

## 下载安装

[清华大学镜像](https://mirrors.tuna.tsinghua.edu.cn/docker-ce/linux/static/stable/x86_64/)

下载后解压至 `/opt/docker`

```bash
cd /opt/docker
mkdir run
mkdir lib
mkdir tmp
mkdir bin
chmod 777 bin/*
```

## daemon.json

`/opt/docker/daemon.json`

```json
{
  "registry-mirrors": [
    "http://registry.docker-cn.com",
    "http://hub-mirror.c.163.com"
  ]
}
```

## docker.service

`/usr/lib/systemd/system/docker.service`

```text
[Unit]
Description=Docker Application Container Engine
Documentation=https://docs.docker.com
After=network-online.target firewalld.service
Wants=network-online.target

[Service]
Type=notify
#EnvironmentFile=/opt/docker/env
ExecStart=/opt/docker/bin/dockerd -H tcp://0.0.0.0:2375 -H unix:///opt/docker/run/docker.sock --config-file=/opt/docker/daemon.json --data-root=/opt/docker/lib --exec-root=/opt/docker/run --pidfile=/opt/docker/tmp/docker.pid
ExecReload=/bin/kill -s HUP $MAINPID
LimitNOFILE=infinity
LimitNPROC=infinity
TimeoutStartSec=0
Delegate=yes
KillMode=process
Restart=on-failure
StartLimitBurst=3
StartLimitInterval=60s

[Install]
WantedBy=multi-user.target
```

## 可执行文件

```bash
cd /usr/bin
sudo ln -s /opt/docker/bin/* ./
```

## path

```text
export PATH="/opt/docker/bin:$PATH"
```
