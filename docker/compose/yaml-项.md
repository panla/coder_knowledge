# Docker-compose YAML

## YAML 模板

### version

[对应关系](https://docs.docker.com/compose/compose-file/compose-file-v3/)

### servicves

### image

```yaml
image: "centos:8.3.2011"
```

### build，指定 Dockerfile 文件夹的路径

```yaml
build: "/path/to/build/dir"
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
  password: "abcdefg"
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
