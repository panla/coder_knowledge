# Nginx SSL 证书配置

## 配置内容

```text
listen 443 ssl;
ssl_certificate   cert/example.pem;
ssl_certificate_key  cert/example.key;
ssl_session_timeout 5m;
ssl_ciphers "openssl ciphers -v";
ssl_protocols TLSv1 TLSv1.1 TLSv1.2;

if ($scheme != "https") {
    return 301 https://$host$request_uri;
}
```