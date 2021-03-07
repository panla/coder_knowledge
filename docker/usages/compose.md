# Docker Compose

负责对 Docker 容器，集群的快速编排，定义和运行多个容器的应用

通过 docker-compose.yml 模板文件来定义一组相关联的应用容器为一个项目

## 下载

```bash
url = "https://github.com/docker/compose/releases/download/1.28.2/docker-compose-$(uname -s)-$(uname -m)"
sudo curl -L $url -o /opt/docker/bin/docker-compose
# 需要直接用 url，url 可以用更新版本

cd /opt/docker/bin
chmod 777 docker-compose
cd /usr/bin
sudo ln -s /opt/docker/bin/docker-compose ./
```

## YAML 模板

### image

```yaml
image: centos:8.3.2011
```

### build，指定 Dockerfile 文件夹的路径

```yaml
build: /path/to/build/dir
```

### ports 暴露的端口信息

没有指定宿主机端口时，会随机指定宿主机端口

```yaml
ports:
  - "3000"
  - "8000:8000"
  - "127.0.0.1:8001:8001"
```

### expose 只暴露端口，但不映射到主机，只被连接的服务访问

```yaml
expose:
  - "3000"
  - "8080"
```

### net 与 docker --net 一致

```yaml
net: "bridge"
net: "net_name"
```

### volumes 数据卷挂载设置

```yaml
volumes:
  - /srv/logs
  - /srv/logs:/srv/logs
  - /srv/data:/srv/data:ro
```

### volumes_from 从另一个服务或容器挂载它的所有数据卷

```yaml
volumes_from:
  - service_name
  - container_name
```

### command，覆盖容器启动后执行的命令

```yaml
command: python -V
```

### enviroment 设置环境变量

```yaml
enviroment:
  password: abcdefg
```

### env_file 从文件中获取环境变量

```yaml
env_file: .env

env_file:
  - /opt/apps/conf.env
```

### links 链接到其他服务器

使用的别名会自动在容器中的 /etc/hosts 创建

```text
172.17.2.186 db
172.17.2.186 database
172.17.2.187 redis
```

```yaml
links:
  - db
  - db:database
  - redis
```

### external_links

```yaml
external_links:
 - redis_1
 - project_db_1:mysql
 - project_db_1:postgresql
```

### extends 基于已有的服务进行扩展

common.yaml

```yaml
webapp:
  build: ./webapp
  environment:
    - DEBUG=false
    - SEND_EMAILS=false
```

development.yml

```yaml
web:
  extends:
    file: common.yml
    service: webapp
  ports: - "8000:8000"
```

### 其他项

和 docker run 支持的选项类似

```yaml
workding_dir: /code
entrypoint: /code/go.sh
user: postgrs
restart: always

hostname: foo
domainname: foo.com
mem_limit: 1000000000
privileged: true
stdin_open: true
tty: true
```

## 使用

服务：一个应用容器，实际上可以运行多个相同镜像的实例
项目：由一组关联的应用容器组成一个完整业务单元
一个项目可以由多个服务（容器）关联而成，Compose 面向项目进行管理。

```bash
docker-compose [option] [COMMAND] [ARGS...]
```

command

```text
# build         构建或重新构建服务
# up            构建，重新创建，启动，链接一个服务器相关的容器

# pull          拉取服务镜像
# logs          回去服务的输出，日志
# port          打印绑定的公共端口
# ps            列出所有容器
# images        列出所有镜像

# start         启动一个已经存在的服务容器
# stop          停止一个已经存在的服务容器
# kill          停止容器
# rm            删除停止的容器
# exec          在一个运行的容器里运行命令
# restart       重启服务

# run           在一个服务上运行命令
#               run ubuntu ping docker.com
#               如果不希望自动启动关联的容器，可以使用 --no-deps 选项
# scale         设置同一个服务运行的容器的个数
#               scale web=2 worker=3
```
