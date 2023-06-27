# Docker

[TOC]

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
    "http://hub-mirror.c.163.com",
    "http://registry.docker-cn.com"
  ],
  "hosts": [
    "tcp://0.0.0.0:2375",
    "unix:///var/run/docker.sock"
  ],
  "pidfile": "/opt/docker/tmp/docker.pid",
  "exec-root": "/opt/docker/run",
  "data-root": "/opt/docker/lib",
  "experimental": true
}
```

## docker.service

`/usr/lib/systemd/system/docker.service`

```conf
[Unit]
Description=Docker Application Container Engine
Documentation=https://docs.docker.com
After=network-online.target firewalld.service containerd.service
Wants=network-online.target
# Requires=docker.socket

[Service]
Type=notify
ExecStart=/opt/docker/bin/dockerd --config-file=/opt/docker/daemon.json
ExecReload=/bin/kill -s HUP $MAINPID
ExecStartPost=-chmod 666 /var/run/docker.sock
TimeoutSec=0
RestartSec=2
Restart=always

# Note that StartLimit* options were moved from "Service" to "Unit" in systemd 229.
# Both the old, and new location are accepted by systemd 229 and up, so using the old location
# to make them work for either version of systemd.
StartLimitBurst=3

# Note that StartLimitInterval was renamed to StartLimitIntervalSec in systemd 230.
# Both the old, and new name are accepted by systemd 230 and up, so using the old name to make
# this option work for either version of systemd.
StartLimitInterval=60s

# Having non-zero Limit*s causes performance problems due to accounting overhead
# in the kernel. We recommend using cgroups to do container-local accounting.
LimitNOFILE=infinity
LimitNPROC=infinity
LimitCORE=infinity

# Comment TasksMax if your systemd version does not support it.
# Only systemd 226 and above support this option.
TasksMax=infinity

# set delegate yes so that systemd does not reset the cgroups of docker containers
Delegate=yes

# kill only the docker process, not all processes in the cgroup
KillMode=process

[Install]
WantedBy=multi-user.target
```

containerd.service

```conf
[Unit]
Description=containerd container runtime
Documentation=https://containerd.io
After=network.target local-fs.target

[Service]
ExecStartPre=-/sbin/modprobe overlay
ExecStart=/usr/bin/containerd

Type=notify
Delegate=yes
KillMode=process
Restart=always
RestartSec=5
# Having non-zero Limit*s causes performance problems due to accounting overhead
# in the kernel. We recommend using cgroups to do container-local accounting.
LimitNPROC=infinity
LimitCORE=infinity
LimitNOFILE=infinity
# Comment TasksMax if your systemd version does not supports it.
# Only systemd 226 and above support this version.
TasksMax=infinity
OOMScoreAdjust=-999

[Install]
WantedBy=multi-user.target
```

## 可执行文件

```bash
cd /usr/bin
sudo ln -s /opt/docker/bin/* ./
```

## path

```bash
export PATH="/opt/docker/bin:$PATH"
```

```bash
cd /usr/local/bin

sudo ln -s /opt/docker/bin/* ./
```

## 安装 portainer

```bash
sudo apt install apparmor
```

```bash
docker run -d -p 127.0.0.1:19000:9000 --name=portainer --restart=always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer-ce
```
