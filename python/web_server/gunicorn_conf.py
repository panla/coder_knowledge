import multiprocessing


# 监听端口或UNIX socket
bind = 'unix:/tmp/project.sock'
# bind = '127.0.0.1:8000'

# 进程数
# workers = multiprocessing.cpu_count() * 2 + 1
workers = 2

# 进程方式 pip3 install eventlet
worker_class = 'eventlet'

# 并发客户端的最大数量, eventlet/gevent 下有效
worker_connections = 2000

# 重载
reload = True

# 日志级别
loglevel = 'DEBUG'

# 把标准输出重定向到 error_log 中
capture_output = True

# 守护进程
# daemon = True

# pid 文件
pid = './tmp/p.pid'

# 允许处理设置的安全标头的前端IP
forwarded_allow_ips = '*'

# 超时进程被杀死时间
timeout = 30

# 等待活动连接上的请求的秒数
keep_alive = 60

# 请求达到这个数值后重启 worker
max_requests = 500

# 请求达到 max-requests 后允许的扩容
max_requests_jitter = 20
