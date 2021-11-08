# Nginx SSL 证书配置

## 配置项

```text
ssl_ciphers      加密算法，多个算法用:分隔，ALL表示全部算法，!表示不启用该算法，+表示将该算法排到最后面去

    TLS_AES_128_GCM_SHA256:TLS_CHACHA20_POLY1305_SHA256:TLS_AES_128_CCM_SHA256:TLS_AES_128_CCM_8_SHA256:TLS_AES_256_GCM_SHA384:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-ECDSA-CHACHA20-POLY1305:ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-ECDSA-AES128-SHA:ECDHE-ECDSA-AES256-SHA:ECDHE-ECDSA-AES128-SHA256:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES128-GCM-SHA256:ECDHE-RSA-CHACHA20-POLY1305:ECDHE-RSA-AES256-GCM-SHA384:ECDHE-RSA-AES128-SHA:ECDHE-RSA-AES256-SHA:ECDHE-RSA-AES128-SHA256:ECDHE-RSA-AES256-SHA384:DHE-RSA-CHACHA20-POLY1305:DHE-RSA-AES128-GCM-SHA256:DHE-RSA-AES256-GCM-SHA384:DHE-RSA-AES128-SHA:DHE-RSA-AES256-SHA:DHE-RSA-AES128-SHA256:DHE-RSA-AES256-SHA256

ssl_protocols    指定SSL协议
```

```text
ssl_protocols TLSv1.2 TLSv1.3;
ssl_ciphers ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384:ECDHE-ECDSA-CHACHA20-POLY1305:ECDHE-RSA-CHACHA20-POLY1305:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA384:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA256;
```

## 优化

```text
开启 HTTP2
    listen 443 ssl;
    listen 443 ssl http2;

禁用 TLSv1
    ssl_protocols TLSv1 TLSv1.1 TLSv1.2 TLSv1.3;
    ssl_protocols TLSv1.2 TLSv1.3;

开启 SSL session 缓存
    ssl_session_cache shared:SSL:50m;     # 1m 4000个，
    ssl_session_timeout 1h;               # 1小时过期 1 hour during which sessions can be re-used.
```

## 1-rewrite

```conf
server {
    listen 80;
    server_name domain.com;

    rewrite ^(.*) https://$server_name$1 permanent;
}
server {
    listen 443 ssl;
    server_name domain.com;

    ssl on;
    ssl_certificate     /etc/nginx/ssl/domain.com.pem;
    ssl_certificate_key /etc/nginx/ssl/domain.com.key;
    ssl_session_timeout 5m;
    ssl_ciphers ALL:!DH:!EXPORT:!RC4:+HIGH:+MEDIUM:!eNULL;;
    ssl_protocols TLSv1 TLSv1.1 TLSv1.2 TLSv1.3;

    location / {
        proxy_pass  http://127.0.0.1:8000;
    }
}
```

## 2-return

```conf
server {
    listen 80;
    server_name domain.com;

    return 301 https://$server_name$request_uri;
}
server {
    listen 443 ssl;
    server_name domain.com;

    ssl on;
    ssl_certificate     /etc/nginx/ssl/domain.com.pem;
    ssl_certificate_key /etc/nginx/ssl/domain.com.key;
    ssl_session_timeout 5m;
    ssl_ciphers ALL:!DH:!EXPORT:!RC4:+HIGH:+MEDIUM:!eNULL;;
    ssl_protocols TLSv1 TLSv1.1 TLSv1.2 TLSv1.3;

    location / {
        proxy_pass  http://127.0.0.1:8000;
    }
}
```

## 3---?

```conf
server {
    listen 443 ssl;

    ssl_certificate   cert/example.pem;
    ssl_certificate_key  cert/example.key;
    ssl_session_timeout 5m;
    ssl_ciphers ALL:!DH:!EXPORT:!RC4:+HIGH:+MEDIUM:!eNULL;;
    ssl_protocols TLSv1 TLSv1.1 TLSv1.2 TLSv1.3;

    if ($scheme != "https") {
        return 301 https://$host$request_uri;
    }
}
```
