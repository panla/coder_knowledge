# 证书

[toc]

## 1 参考一

NGINX 配置 SSL 双向认证与自签名证书

<https://blog.csdn.net/zkt286468541/article/details/80864184>

### 1.1 CA

#### 1.1.1 CA 私钥

```bash
openssl genrsa -out ca-key.pem 2048
```

#### 1.1.2 CA 根证书

```bash
openssl req -new -x509 -days 365 -key ca-key.pem -out ca-cert.pem

# Common Name: Root
```

### 1.2 Server

#### 1.1.1 Server 私钥

```bash
openssl genrsa -out server.pem 2048
openssl rsa -in server.pem -out server-key.pem
# 将生成的私钥转换为 RSA 私钥文件格式
```

#### 1.2.2 Server 签发请求

```bash
openssl req -new -key server.pem -out server.csr

# Common Name: Server
# A challenge password []:Server
```

#### 1.2.3 签发 Server

```bash
openssl x509 -req -sha256 -in server.csr -days 365 -CA ca-cert.pem -CAkey ca-key.pem -CAcreateserial -out server-cert.pem
```

### 1.3

#### 1.3.1 Client 私钥

```bash
openssl genrsa -out client.pem 2048
openssl rsa -in client.pem -out client-key.pem
# 将生成的私钥转换为 RSA 私钥文件格式
```

#### 1.3.2 Client 签发请求

```bash
openssl req -new -key client.pem -out client.csr

# Common Name: Client
# A challenge password []:Client
```

#### 1.3.3 签发 Client

```bash
openssl x509 -req -sha256 -in client.csr -days 365 -CA ca-cert.pem -CAkey ca-key.pem -CAcreateserial -out client-cert.pem
```

## 2 参考二

2 与 1 相近，合并了生成私钥与签发请求步骤

<https://www.jianshu.com/p/63ba67e133ce>

### 2.1 CA

#### 2.1.1 CA 私钥

```bash
openssl genrsa 2048 > ca-key.pem
```

#### 2.1.2 CA 根证书

```bash
openssl req -new -x509 -days 365 -sha256 -nodes -key ca-key.pem > ca-cert.pem

# Common Name: Root
```

### 2.2 Server

#### 2.2.1 Server 私钥和请求

```bash
openssl req -sha256 -newkey rsa:2048 -days 365 -nodes -keyout server.pem > server-req.pem

# 将生成的私钥转换为 RSA 私钥文件格式
openssl rsa -in server.pem -out server-key.pem
```

#### 2.2.2 签发 Server

```bash
openssl x509 -req -sha256 -in server-req.pem -days 365 -CA ca-cert.pem -CAkey ca-key.pem -set_serial 01 > server-cert.pem
```

### 2.3 Client

#### 2.3.1 Client 私钥和请求

```bash
openssl req -sha256 -newkey rsa:2048 -days 365 -nodes -keyout client.pem > client-req.pem

# 将生成的私钥转换为 RSA 私钥文件格式
openssl rsa -in client.pem -out client-key.pem
```

#### 2.3.2 签发 Client

```bash
openssl x509 -req -sha256 -in client-req.pem -days 365 -CA ca-cert.pem -CAkey ca-key.pem -set_serial 01 > client-cert.pem
```

## 3 参考三

IP 自签名证书

<https://www.cnblogs.com/dirigent/p/15246731.html>

与 1，2相比 3 使用 des 来生成私钥，以及加上了 针对 IP 的证书

### 3.1 CA
