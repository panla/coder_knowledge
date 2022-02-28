# Redis SSL

[toc]

## 0 pre

Version 6.2.x

还不能很好地支持 SSL

## 1 生成证书

`../scripts/gen-tls.sh`

## 2 config

```text
port                 0 # 禁用 非 TLS 端口
tls-port             6379

tls-cert-file        redis.crt
tls-key-file         redis.key
tls-ca-cert-file     ca.crt
tls-dh-params-file   redis.dh
tls-auth-clients     yes
tls-replication      yes

# 集群时
tls-cluster          yes
```

## 3 使用 Redis-Cluster + SSL
