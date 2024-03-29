# 配置

[TOC]

## config files

### `conf/nginx.conf`

```text
user  root;
worker_processes  4;

#error_log  logs/error.log;
#error_log  logs/error.log  notice;
#error_log  logs/error.log  info;

pid        logs/nginx.pid;

events {
    worker_connections  4096;
}

http {
    include /opt/nginx/conf.d/*.conf;
}
```

### `conf.d/nginx.conf`

```text
server {
    listen       80;
    server_name  localhost;

    #charset koi8-r;

    #access_log  logs/host.access.log  main;

    location / {
        root   html;
        index  index.html index.htm;
    }

    #error_page  404              /404.html;

    # redirect server error pages to the static page /50x.html
    #
    error_page   500 502 503 504  /50x.html;
    location = /50x.html {
        root   html;
    }
}
```

## proxy_params

```text
proxy_set_header Host $http_host;
proxy_set_header X-Real-IP $remote_addr;
proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
proxy_set_header X-Forwarded-Proto $scheme;
```

## CORS

```text
add_header 'Access-Control-Allow-Origin' '*';

if ($request_method = 'OPTIONS') {
    add_header 'Access-Control-Allow-Origin' '*';
    add_header 'Access-Control-Max-Age' 86400;
    add_header 'Content-Type' 'text/plain charset=UTF-8';
    add_header 'Content-Length' 0;
    return 204;
}
```

## 静态资源

```text
location /media {
    autoindex on;
    root /docker_media;
    alias /docker_media;
}
```

## 长连接

```text
keepalive_requests 1000;
keepalive_timeout 60;
```

## 推荐配置

```text
http {
    upstream backend {
        zone upstream 64K;
        server 127.0.0.1:3000 max_fail_timeout=20s;
        keepalive 20s;
    }

    server {
        listen 80;
        server_name example.com;

        location /api/v1 {
            include proxy_params;

            proxy_pass http://backend;
            proxy_next_upstream error timeout http_500;
        }
    }
}

```
