# 集群

[toc]

## 1 命令

### 1.1 cluster

```bash
# 查询状态
./bin/emqx_ctl cluster status

# 加入集群
./bin/emqx_ctl cluster join emqx@other

# 离开集群
./bin/emqx_ctl cluster leave

# 删除节点
./bin/emqx_ctl cluster force-leave emqx@this
```

## 2 配置

### 2.1 集群节点发现

集群发现节点与自动机群

```conf
# 发现方案
cluster.discovery = manual
# manual static
```

集群节点发现端口

```conf
# ekka 4.0 之后默认模式
# ListeningPort = BasePort + Offset
# BasePort = 4370 不可配置

# 如果 emqx.conf 里配置了节点名：node.name = emqx@192.168.0.12，那么监听端口为 4370，
# 但对于 emqx1 (或者 emqx-1) 端口就是 4371，以此类推

# Cluster RPC Port
# BasePort = 5370 不可配置
```

### 2.2 集群节点其他配置

```conf
# 支持集群脑裂自动恢复
cluster.autoheal = on

# 从集群自动删除宕机节点
cluster.autoclean = 5m

# 本应多台服务器，组集群
# 单机时，需要处理端口问题，将原先所有 emqx 节点监听的端口 port 加上一个偏移 offset 作为新的 emqx2 节点的监听端口
```

### 2.3 节点配置

```conf
# 节点名称
node.name = emqx-1@192.168.1.10
node.name = emqx-2@192.168.1.11

# 如果是 docker 在配置 emqx.conf 后还要，
# 还需要配置  EMQX_NODE_NAME=emqx-1@192.168.1.10
# TODO 这一点比较奇怪，单纯配置 emqx.conf 没有发挥作用
```

### 2.4 dir

```text
.
├── conf
│   ├── certs
│   │   ├── cacert.pem
│   │   ├── cert.pem
│   │   └── key.pem
│   ├── emqx-1
│   │   └── emqx.conf
│   ├── emqx-2
│   │   └── emqx.conf
│   ├── loaded
│   │   └── loaded_plugins
│   └── plugins
│       ├── emqx_auth_http.conf
│       ├── emqx_auth_jwt.conf
│       └── emqx_auth_mnesia.conf
└── docker-compose.yaml
```

## 3 负载均衡

### 3.1 nginx

```conf
stream {
  upstream stream_backend {
      zone tcp_servers 64k;
      hash $remote_addr;
      server 192.168.0.2:1883 max_fails=2 fail_timeout=30s;
      server 192.168.0.3:1883 max_fails=2 fail_timeout=30s;
  }

  server {
      listen 8883 ssl;
      status_zone tcp_server;

      proxy_buffer_size 4k;
      ssl_handshake_timeout 15s;

      ssl_certificate     /etc/emqx/certs/cert.pem;
      ssl_certificate_key /etc/emqx/certs/key.pem;

      proxy_pass stream_backend;

  }
}

```
