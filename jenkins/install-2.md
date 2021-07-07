# Docker 下的使用

## 安装

- [docker-compose.yml 参考](./docker-compose.yml)

## 在容器中的 jenkins 中使用宿主机的 docker 命令(单机)

为了能够使 docker 容器中的 jenkins 执行 docker 命令，
需要挂载数据卷。docker,docker.sock 如此，就可以

```text
-v /var/run/docker.sock:/var/run/docker.sock
-v /usr/bin/docker:/usr/bin/docker
-v /usr/bin/docker-compose:/usr/bin/docker-compose
```

## jenkins 与 项目所在不同服务器

通过 ssh 等来远程操作服务器
