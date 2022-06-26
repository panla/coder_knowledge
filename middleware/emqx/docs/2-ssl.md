# 2 emqx ssl

## 2.1 https

```conf
## https
listener.ssl.external = 8883

listener.ssl.external.key_password = yourpass
listener.ssl.external.keyfile = etc/certs/server-key.pem
listener.ssl.external.certfile = etc/certs/server-cert.pem
listener.ssl.external.cacertfile = etc/certs/ca-cert.pem
```

### 2.2 wss

```conf
## wss
listener.wss.external = 8084
listener.wss.external.mqtt_path = /mqtt

listener.wss.external.keyfile = etc/certs/key.pem
listener.wss.external.certfile = etc/certs/cert.pem
listener.wss.external.cacertfile = etc/certs/cacert.pem
listener.wss.external.key_password = yourpass

##
listener.ssl.external.dhfile = etc/certs/dh-params.pem
```
