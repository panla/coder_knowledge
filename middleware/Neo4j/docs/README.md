# README

## 目录

## 网络

- Bolt: 7687
- HTTP: 7474
- HTTPS:7473

## 部分配置项

```text
# 开放访问
dbms.default_listen_address=0.0.0.0



# Bolt SSL configuration
#dbms.ssl.policy.bolt.enabled=true
#dbms.ssl.policy.bolt.base_directory=certificates/bolt
#dbms.ssl.policy.bolt.private_key=private.key
#dbms.ssl.policy.bolt.public_certificate=public.crt
#dbms.ssl.policy.bolt.client_auth=NONE

# Https SSL configuration
#dbms.ssl.policy.https.enabled=true
#dbms.ssl.policy.https.base_directory=certificates/https
#dbms.ssl.policy.https.private_key=private.key
#dbms.ssl.policy.https.public_certificate=public.crt
#dbms.ssl.policy.https.client_auth=NONE

# Cluster SSL configuration
#dbms.ssl.policy.cluster.enabled=true
#dbms.ssl.policy.cluster.base_directory=certificates/cluster
#dbms.ssl.policy.cluster.private_key=private.key
#dbms.ssl.policy.cluster.public_certificate=public.crt

# Backup SSL configuration
#dbms.ssl.policy.backup.enabled=true
#dbms.ssl.policy.backup.base_directory=certificates/backup
#dbms.ssl.policy.backup.private_key=private.key
#dbms.ssl.policy.backup.public_certificate=public.crt
```
