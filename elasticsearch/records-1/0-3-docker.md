# 在 docker 中使用 es

## docker es

### 获取镜像

```bash
docker pull elasticsearch:7.12.0
```

### 创建网络

```bash
docker network create --driver bridge --subnet 172.18.0.0/16 --gateway 172.18.0.1 es
```

### 创建启动容器

```bash
docker run -d -p 9200:9200 -p 9300:9300 --net es --ip 172.18.0.2 --name es -e ES_JAVA_POTS="-Xms256m -Xmx256m" -e "discovery.type=single-node" elasticsearch:7.12.0
```

## docker kibana

[版本对应](https://www.elastic.co/cn/support/matrix#matrix_compatibility)

```bash
docker run -d --net es --ip 172.18.0.3 -p 5601:5601 --name kibana kibana:7.12.0
```
