# 签名

非对称

私钥签名，公钥验证

## 1 产生私钥

```bash
openssl genrsa -out private-key.pem 2048
```

## 2 导出公钥

```bash
openssl rsa -pubout -in private-key.pem -out public-key.pem
```

## 3 利用私钥生成签名

```bash
openssl dgst -sha256 -sign private-key.pem -out original.sig original.txt
```

## 4 利用公钥校验签名

```bash
openssl dgst -sha256 -verify public-key.pem -signature original.sig original.txt
```
