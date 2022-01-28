# Docker-compose YAML

[toc]

## YAML 模板

```yaml
version: "3.9"
services:

```

### version

[对应关系](https://docs.docker.com/compose/compose-file/compose-file-v3/)

### servicves

### image

```yaml
version: "3.9"
services:
  web1:
    image: centos:8.3.2011
```

### build

指定 Dockerfile 文件夹的路径，利用它自动构建这个镜像，然后使用这个镜像

```yaml
version: "3.9"
services:
  web1:
    build: ./dir
```

### command

覆盖容器启动后默认执行的命令

```yaml
version: "3.9"
services:
  web1:
    command: python -V
```

### `container_name`

指定容器名称

```yaml
version: "3.9"
services:
  web1:
    container_name: web1
```

### `depends_on`

容器之间的依赖，a 依赖 b

但 a 也不用等 a 完全启动后再启动

```yaml
version: "3.9"
services:
  web1:
    depends_on: web2
  web2:
    ...
```

### `env_file`

从文件中获取环境变量

```yaml
version: "3.9"
services:
  web1:
    env_file: ./common.env
```

### enviroment

设置环境变量

```yaml
version: "3.9"
services:
  web1:
    environment:
      # 格式还挺多？!
      - password: "abcdefg"
      - "ES_JAVA_OPTS=-Xms256m -Xmx256m"
      - "TZ=Asia/Shanghai"
      - TZ=Asia/Shanghai
      TZ: "Asia/Shanghai"
```

### expose

只暴露端口，但不映射到主机，只被连接的服务访问

```yaml
version: "3.9"
services:
  web1:
    expose:
      - "3000"
      - "8080"
```

### ports

暴露的端口信息

没有指定宿主机端口时，会随机指定宿主机端口

```yaml
version: "3.9"
services:
  web1:
    ports:
      - "3000"
      - "8000:8000"
      - "127.0.0.1:8001:8001"
```

### networks 内

```yaml
version: "3.9"
services:
  web1:
    networks:
      es:
        ipv4_address: 172.18.0.2
```

### networks 外

```yaml
networks:
  es:
    external: true
```

### sysctls

```yaml
ulimits:
  nproc: 65535
  # 最大进程数
  nofile:
    # 文件句柄数
    soft: 20000
    hard: 40000
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

### `volumes_from` 从另一个服务或容器挂载它的所有数据卷

```yaml
volumes_from:
  - service_name
  - container_name
```

### swarm mode

configs deploy

### 其他项

```yaml
devices:
  - "/dev/ttyUSB1:/dev/ttyUSB0"
  # 指定设备映射关系
```

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

### example

```yaml
version: '3.9'
services:
  kibana:
    container_name: kibana1
    image: kibana:7.12.0
    restart: always
    ports:
      - 5601:5601
    networks:
      es:
        ipv4_address: 172.18.0.2
    environment:
      - TZ=Asia/Shanghai
      - LC_ALL=C.UTF-8
      - LANG=C.UTF-8
      - ELASTICSEARCH_HOSTS: "http://172.18.0.3:9200"
    depends_on:
      es01
  es01:
    container_name: es01
    image: elasticsearch:7.12.0
    restart: always
    ports:
      - 9200:9200
      - 9300:9300
    volumes:
      - ./data/es01:/user/share/elasticsearch/data
      - ./logs/es01:/usr/share/elasticsearch/logs
      - ./plugins/es01:/usr/share/elasticsearch/plugins
      - ./config/es01:/user/share/elasticsearch/config
    networks:
      es:
        ipv4_address: 172.18.0.3
    environment:
      - TZ=Asia/Shanghai
      - LC_ALL=C.UTF-8
      - LANG=C.UTF-8
      - node.name=es01
      - cluster.name=es-docker-cluster
      - discovery.seed_hosts=172.18.0.4, 172.18.0.5
      - cluster.initial_master_nodes=172.18.0.3, 172.18.0.4, 172.18.0.5
      - "ES_JAVA_OPTS=-Xms256m -Xmx256m"
networks:
  es:
    external: true
```
