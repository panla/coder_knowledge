# 配置

[toc]

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
autoindex on;
root
alias
```

## `conf/nginx.conf`

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

## `conf.d/nginx.conf`

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

## 前端

1 `try_files`

```text
location / {
    root   /web/front;
    try_files $uri $uri/ /index.html;
}
```

2 `index`

```text
location / {
    root   /web/front;
    index  index.html index.htm;
}
```

3 `try_files index`

```text
location / {
    root   /web/front;
    try_files $uri $uri/ /index.html;
    index  index.html index.htm;
}
```

当 vue + vite 时，1 3 可以，2 不行

当 react + build 时，1 2 3 都可以

```text
index 就是根目录，也就是只识别“/”结尾的，输入不存在或者刷新页面的路径就直接报nginx的404了，而不会重定向到index.html

try_files 更加可靠， 首先会查找"$uri"下的这个文件，如果不存在会查找$uri/,如果还不存在就会重定向到 /index.html页面
```

## upstream

### upstream base

```text
upstream backend {
    server 172.25.0.8:8000;
    server 172.25.0.10:8000;
    # server address [parameters];
}

server {
    location / {
        proxy_pass http://backend;
    }
}
```

### upstream parameters

- weight: 设置请求发到后端的权重，每台服务器能够响应的请求数量的比例，默认 1
- `max_fails`: 在指定时间内请求失败的最大次数，默认 1，设置为 0 则禁用此设置
- `fail_timeout`: `max_fails` 的时间
- down: 服务不可用
- backup: 当所有服务都不可用时，会对 backup 分流

TODO 请求失败，**这里的请求失败指的什么样的失败**?

### 负载均衡策略

- `ip_hash`: 根据 ip 地址将请求分流到后端
- `least_conn`: 请求量最小的会优先获得分流
- hash: 按照指定的 key 将请求分布到后端服务器，同一个 IP 地址的请求会被代理到同一个后端
- sticky: 根据 Cookie 分流

```text
upstream backend {
    hash $request_uri;
    server addr max_fails=5 fail_timeout=10s weight=10;
}
```

### 长连接

```text
keepalive_requests 1000;
keepalive_timeout 60;
```
