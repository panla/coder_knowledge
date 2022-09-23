# cluster

[toc]

## 0 原有数据问题

原有集群不动，新的节点不留数据，加入集群后会自动同步数据

账号数据 **emqx_auth_mnesia.conf 账号 似乎没有同步**

### 0.1 端口问题

- 4370: cluster.mcast.ports，没有它，发现不了其他节点
- 5369: rpc.tcp_server_port，没有它，各个节点无法同步消息

## 1 不同服务器

### 1.1 static

- 192.168.9.96
- 192.168.9.114

```yaml
version: "3"

networks:
  dev_net:
    external: true

services:

  emqx-1:
    image: emqx/emqx:4.4.9
    container_name: emqx-cluster-node-1
    restart: always
    networks:
      dev_net:
        ipv4_address: "172.18.7.3"
    ports:
      - 4370:4370
      - 5369:5369
      - 9200:18083
      - 9201:1883
      - 9202:8883
      - 9203:8083
      - 9204:8084
    volumes:
      - /etc/localtime:/etc/localtime:ro
      - ./conf/emqx-1/etc/emqx.conf:/opt/emqx/etc/emqx.conf
      - ./conf/emqx-1/etc/acl.conf:/opt/emqx/etc/acl.conf
      - ./conf/emqx-1/etc/certs:/opt/emqx/etc/certs
      - ./conf/emqx-1/data/loaded_plugins:/opt/emqx/data/loaded_plugins
      - ./conf/emqx-1/etc/plugins:/opt/emqx/etc/plugins
    environment:
      - EMQX_NAME=emqx
      - EMQX_HOST=192.168.9.96
      - EMQX_CLUSTER__DISCOVERY=static
      - EMQX_CLUSTER__STATIC__SEEDS=emqx@192.168.9.114,emqx@192.168.9.96

networks:
  dev_net:
    external: true
```

### 1.2 manual

- 192.168.9.96
- 192.168.9.114

```text
修改 cluster.name
修改 node.name
修改 node.cookie
```

```yaml
version: "3"

networks:
  dev_net:
    external: true

services:

  emqx-1:
    image: emqx/emqx:4.4.9
    container_name: emqx-cluster-node-1
    restart: always
    networks:
      dev_net:
        ipv4_address: "172.18.2.3"
    ports:
      - 4370:4370
      - 5369:5369
      - 9200:18083
      - 9201:1883
      - 9202:8883
      - 9203:8083
      - 9204:8084
    volumes:
      - /etc/localtime:/etc/localtime:ro
      - ./conf/emqx-1/etc/emqx.conf:/opt/emqx/etc/emqx.conf
      - ./conf/emqx-1/etc/acl.conf:/opt/emqx/etc/acl.conf
      - ./conf/emqx-1/etc/certs:/opt/emqx/etc/certs
      - ./conf/emqx-1/data/loaded_plugins:/opt/emqx/data/loaded_plugins
      - ./conf/emqx-1/etc/plugins:/opt/emqx/etc/plugins
    environment:
      - EMQX_NODE_NAME=emqx@192.168.9.99
      # - EMQX_NODE_NAME=emqx@192.168.9.114

```

加入

```bash
# 192.168.9.96

emqx_ctl cluster status

emqx_ctl cluster join emqx@192.168.9.114

emqx_ctl cluster status
```

## 2 同一服务器

### 2.1 static

- 192.168.9.96
- 192.168.9.114

### 2.2 manual

如果在容器中，则不太需要考虑端口问题

- 192.168.9.96
- 192.168.9.114

```yaml
version: "3"
services:

  emqx-1:
    image: emqx/emqx:4.4.6
    container_name: dev_study_emqx-1
    restart: always
    networks:
      dev_net:
        ipv4_address: "172.20.7.3"
    ports:
      - 1883:1883
      - 8883:8883
      - 18083:18083
    volumes:
      - /etc/localtime:/etc/localtime:ro
      - ./conf/emqx-1/etc/emqx.conf:/opt/emqx/etc/emqx.conf
      - ./conf/emqx-1/etc/acl.conf:/opt/emqx/etc/acl.conf
      - ./conf/emqx-1/etc/certs:/opt/emqx/etc/certs
      - ./conf/emqx-1/data/loaded_plugins:/opt/emqx/data/loaded_plugins
      - ./conf/emqx-1/etc/plugins:/opt/emqx/etc/plugins

    environment:
      - EMQX_NODE_NAME=emqx@192.168.9.114

  emqx-2:
    image: emqx/emqx:4.4.6
    container_name: dev_study_emqx-2
    restart: always
    networks:
      dev_net:
        ipv4_address: "172.20.7.4"
    ports:
      - 1883:1883
      - 8883:8883
      - 18083:18083
    volumes:
      - /etc/localtime:/etc/localtime:ro
      - ./conf/emqx-1/etc/emqx.conf:/opt/emqx/etc/emqx.conf
      - ./conf/emqx-1/etc/acl.conf:/opt/emqx/etc/acl.conf
      - ./conf/emqx-1/etc/certs:/opt/emqx/etc/certs
      - ./conf/emqx-1/data/loaded_plugins:/opt/emqx/data/loaded_plugins
      - ./conf/emqx-1/etc/plugins:/opt/emqx/etc/plugins

    environment:
      - EMQX_NODE_NAME=emqx@192.168.9.96

networks:
  dev_net:
    external: true
```

加入

```bash
# 172.20.7.4

emqx_ctl cluster status

emqx_ctl cluster join emqx@172.20.7.3

emqx_ctl cluster status
```
