# 证书

[toc]

## 1 参考

NGINX 配置 SSL 双向认证与自签名证书

<https://blog.csdn.net/zkt286468541/article/details/80864184>

IP 自签名证书

<https://www.cnblogs.com/dirigent/p/15246731.html>

## 2 参考一

### 2.1 CA 私钥

```bash
openssl genrsa -out ca-key.pem 2048
```

### 2.2 CA 根证书

```bash
openssl req -new -x509 -days 365 -key ca-key.pem -out ca-cert.pem

# Common Name: Root
```

### 2.3 Server 私钥

```bash
openssl genrsa -out server.pem 2048
openssl rsa -in server.pem -out server-key.pem
```

### 2.4 Server 签发请求

```bash
openssl req -new -key server.pem -out server.csr

# Common Name: Server
# A challenge password []:Server
```

### 2.5 签发 Server

```bash
openssl x509 -req -sha256 -in server.csr -CA ca-cert.pem -CAkey ca-key.pem -CAcreateserial -days 265 -out server-cert.pem
```

## 2.6 Client 私钥

```bash
openssl genrsa -out client.pem 2048
openssl rsa -in client.pem -out client-key.pem
```

### 2.7 Client 签发请求

```bash
openssl req -new -key client.pem -out client.csr

# Common Name: Client
# A challenge password []:Client
```

### 2.8 签发 Client

```bash
openssl x509 -req -sha256 -in client.csr -CA ca-cert.pem -CAkey ca-key.pem -CAcreateserial -days 265 -out client-cert.pem
```

## 4 证书转换

### 4.1 去除秘钥密码

```shell
openssl rsa -in client.key -out client.key.unsecure
```

### 4.2 .pem .cer 转 bks

```shell
# 生成 .p12
openssl pkcs12 -export -nodes -in client.cer -inkey client.key -out client.p12

# 生成 .bks
keytool -importkeystore -srckeystore client.p12 -srcstoretype pkcs12 -destkeystore client.bks -deststoretype bks -provider org.bouncycastle.jce.provider.BouncyCastleProvider -providerpath bcprov-ext-jdk15on-1.70.jar

# 查看证书列表
keytool -list -rfc -keystore client.bks -storetype BKS -provider org.bouncycastle.jce.provider.BouncyCastleProvider -storepass 'bks秘钥库密码' -providerpath bcprov-ext-jdk15on-1.70.jar
```

### 4.3 .cer 转 .pfx

```bash
# 生成 pfx
openssl pkcs12 -export -in client.cer -inkey client.key -out client.pfx
```

### 4.4 校验

```shell
openssl verify -CAfile ca.cer server.cer client.cer
```
