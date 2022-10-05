# docker

## 目录文件占用

```text
/usr/local/bin/*docker*

/Applications/Docker.app
/System/Volumes/Data/Applications/Docker.app

/Users/user/Library/Containers/com.docker.docker

/Users/user/.docker
/Users/user/.docker/run/docker.sock
/Users/user/Library/Application Support/com.bugsnag.Bugsnag/com.docker.docker
/Users/user/Library/Preferences/com.electron.dockerdesktop.plist
/Users/user/Library/Preferences/com.docker.docker.plist
/Users/user/Library/Application Scripts/group.com.docker
/Users/user/Library/HTTPStorages/com.docker.docker
/Users/user/Library/Group Containers/group.com.docker
/Users/user/Library/Group Containers/group.com.docker/Library/Application Scripts/group.com.docker

/private/var/run/docker-cli.sock
/private/var/run/docker.sock
/private/var/run/com.docker.vmnetd.sock

```

```text
/private/var/run/docker.sock -> /Users/user/.docker/run/docker.sock

/private/var/dirs_cleaner/jP/docker.sock -> /Users/user/.docker/run/docker.sock

/var/run/docker.sock -> /Users/user/.docker/run/docker.sock
```

## 查看容器占用

<https://blog.csdn.net/m0_38112165/article/details/120116336>

```bash
# 使用一个已有的镜像
docker run -it --privileged --pid=host golang:1.19.1-bullseye nsenter -t 1 -m -u -n -i sh

1 启动一个容器
2 进入容器
3 cd /var/lib/docker/containers
4 查看各个容器数据，可以删除过多的日志
```

## 3.3 镜像不同指令集

arm64 aarch64
