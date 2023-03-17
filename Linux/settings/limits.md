# limits

## shell

```bash
ulimit -n 100000
```

## user

`/etc/security/limits.conf`

```conf
* soft nofile 655350
* hard nofile 655350

* soft nproc 1310720
* hard nproc 1310720
```

## system

`/proc/sys/fs/file-max`

(2 ** 63) - 1

## sysctl.conf

`/etc/sysctl.conf`

```conf
; 进程可以同时打开的最大句柄数，直接限制最大并发连接数
fs.file-max = 9223372036854775807
fs.nr_open=2097152

; 是否允许将TIME-WAIT状态的socket重新用于新的TCP链接
net.ipv4.tcp_tw_reuse = 1

; TIME-WAIT Socket 最大数量、回收与重用设置
net.ipv4.tcp_max_tw_buckets = 1048576

; 当期佣keepalive时，TCP发送keepalive消息的频度，设置的小，可以更快清理无效连接
net.ipv4.tcp_keepalive_time = 600

; 当服务器主动关闭连接时，socket保持在FIN-WAIT-2状态的最大时间
net.ipv4.tcp_fin_timeout = 30

; TCP接收缓存(用于滑动窗口)的最小值，默认值，最大值
net.ipv4.tcp_rmem =4096 32768 16777216
; TCP发送缓存(用于滑动窗口)的最小值，默认值，最大值
net.ipv4.tcp_wmem =4096 32768 16777216

; 内核套接字接收缓存区默认的大小
net.core.rmem_default = 262144
; 内核套接字发送缓存区默认的大小
net.core.wmem_default = 262144
; 内核套接字接收缓存区默认的最大大小
net.core.rmem_max = 2097152
; 内核套接字发送缓存区默认的最大大小
net.core.rmem_max = 2097152

net.ipv4.tcp_syncookies = 1

; TCP三次握手建立阶段接受WYN请求队列的最大长度，默认1024,
; 将其设置大一些可以使出现Nginx繁忙来不及accept新连接的情况时，Linux不至于丢失客户端发起的连接请求
net.ipv4.tcp_max_syn_backlog = 1024

; 定义了 UDP TCP 连接中本地端口的取值范围
net.ipv4.ip_local_port_range =1024    61000

; 当网卡接收数据包的速度大于内核处理的速度时，会有一个队列保存数据包，该队列的最大值
net.core.netdev_max_backlog=16384
net.ipv4.tcp_max_syn_backlog=16384
```
