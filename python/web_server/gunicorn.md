# 配置

## cmd

```text

-k/--worker-class    worker 类型
-w/--workers         worker 数量
-b/--bind            绑定地址
-t/--timeout         超时 30
--reload             热重启
--keep-alive         在 Keep-Alive 连接上等待的秒数 2
--log-level          INFO DEBUG

--backlog             积压数 2048
--worker-connections  单个 worker 最大连接数
--max-requests        请求达到这个数值后重启 worker
--max-requests-jitter 请求达到 max-requests 后允许的扩容
```

## file -c/--config

```text
bind
workers
worker_class
timeout
keepalive
loglevel
reload

worker_connections
max_requests
max_requests_jitter
```
