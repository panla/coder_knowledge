# 在 docker 中使用 es

[参考](https://www.elastic.co/guide/en/elasticsearch/reference/current/docker.html)

## docker es

### 获取镜像

```bash
docker pull elasticsearch:7.12.0
```

### 创建网络

```bash
docker network create --driver bridge --subnet 172.19.0.0/16 --gateway 172.19.0.1 es
```

### 创建启动容器

```bash
docker run -d -p 9200:9200 -p 9300:9300 --net es --ip 172.19.0.2 --name es -e ES_JAVA_OPTS="-Xms256m -Xmx256m" -e "discovery.type=single-node" -e "TZ=Asia/Shanghai" elasticsearch:7.12.0
```

## docker kibana

[版本对应](https://www.elastic.co/cn/support/matrix#matrix_compatibility)

`config/kibana.yml`

```text
elasticsearch.hosts: [ "http://172.19.0.2:9200" ]
```

```bash
docker run -d --net es --ip 172.19.0.3 -p 5601:5601 -e "TZ=Asia/Shanghai" -e "ELASTICSEARCH_HOSTS=http://172.19.0.2:9200" --name kibana kibana:7.12.0
```
