# Docker 命令

## 镜像 image

### 下载新镜像

```bash
docker pull 镜像名称:标签
docker pull centos:8.3
```

### 全部镜像

```bash
docker images
docker image ls
docker image ls -a
```

### 删除镜像

```bash
docker image rm image_id
docker rmi image_id
docker image rm 镜像名称:标签
```

离线，将打包的tar镜像文件倒入自己的docker仓库

```bash
docker load -i nginx.tar
```

## 容器 container

宿主机:容器

### 操作容器，查/日志/进程

```bash
# 查看容器
docker ps
docker ps -a
docker ps -q

# 查看容器细节信息
docker inspect container

# 日志
docker logs container
docker logs -f container

# 容器内进程
docker top container
```

### 重启/启动/停止/删除

```bash
# 重启
docker restart container
# 启动
docker start container
# 停止
docker stop container
# 删除
docker rm container
docker rm -f container
docker rm $(docker ps -qa)
```

### 运行容器

```bash
docker run 镜像名:tag
docker run 镜像id
docker run -d -p 8080:8080 --name tomcat tomcat

--name 容器名称
-d 后台运行 daemon
-p 映射端口 宿主机:容器
-v 数据卷 宿主机绝对路径:容器
```

### 宿主机与容器交互

```bash
docke exec -it container /bin/bash

# 复制
docker cp  container:/data /data
docker cp /data container:/data
```

### 数据卷

```bash
docker run -d -p 8080:8080 --name tomcat -v /data/apps:/data/apps tomcat
```

### commit

```bash
docker commit -m "commit message" -a "author" container container:tag
docker save container:tag -o container.tar
```

## 镜像分层原理
