# Net

[TOC]

## 1 网络模型

| 七层网络模型 | 五层网络模型 | TCP/IP | 协议 | other |
| :-: | :-: | :-: | :-: | :-: |
| 应用层 | 应用层 | 应用层 | TELNET FTP SMTP HTTP |  |
| 表示层 | 应用层 | 应用层 |  |  |
| 会话层 | 应用层 | 应用层 |  |  |
| 运输层 | 运输层 | 运输层 | TCP UDP |  |
| 网络层 | 网络层 | 网际层 | ICMP IP ARP RARP |  |
| 数据链路层 | 数据链路层 | 网络接口层 |  |  |
| 物理层 | 物理层 | 网络接口层 |  |  |

## 2 TCP

### 2.1 建立连接，三次握手

- SYN：请求同步标志，1 为有效
- ACK：应答标志，表示接收到所发的数据，1 为有效
- FIN：结束请求标志，1 为有效

- ack：应答，值是告诉对方下一次所发数据地址, 也是接收到的 seq + 1
- seq：值为所发数据地址

```text
Client                  Server
    --> SYN=1, seq=x，(请求建立连接)
    <-- SYN=1, ACK=1, seq=y, ack=x+1(因为来的数据是 seq=x)，(针对客户端的 SYN 确认应答，并建立连接)
    --> ACK=1, seq=x+1(上次的x再加1), ack=y+1(因为来的数据是 seq=y)，(针对服务端的 SYN 的确认应答)

为了确认双方的发送和接收能力正常

1，客户端发送正常
2，服务端接收，发送正常
3，客户端接收正常
```

### 2.2 释放连接，四次挥手

```text
Client                  Server
    --> FIN=1, seq=x，结束请求同步
    <-- ACK=1, ack=x+1, (服务端接收到客户端结束的请求)
    <-- FIN=1, seq=y, (服务端回复已经可以断开了，seq=z 中间数据传输，地址发生了变化)
    --> ACK=1, ack=y+1，(客户端结束)

TCP连接是全双工的，因此，每个方向都必须要单独进行关闭

Server收到FIN时，告诉Client收到了FIN，然后Server把自己的数据发送完，才能去发送FIN给Client
```

### 2.3 数据传输

- 1 超时重传，发送端的数据带有seq，接收端返回ack，进行确认，超时未收到ack则认为丢失，重传数据
- 2 快速重传，接收端发现数据丢失后，返回ack，发送连续收到相同的ack，出发发送端快速重传
  - 接收端主动告诉发送端需要重传
- 3 流量控制，TCP滑动窗口流量控制
  - 只关注了发送端和接收端自身的状况，未考虑整个网络的通信情况
- 4 拥塞控制，

### 2.3 TCP与UDP区别

- 区别一：TCP面向连接，收发消息前需要先建立连接，UDP则是非连接，两端不必建立连接
- 区别二：TCP包头20字节，UDP包头8字节
  - 源端口16，目标端口16，序列号32，回应序号32，TCP头长度4，reserved6,控制代码6，窗口大小16，偏移量16，校验和16，选项32，总计20字节
  - 源端口16，目标端口16，长度16，校验和16，总计8字节
- 区别三：TCP具有一定的可靠连接传输，UDP不保证可靠交付
- 区别四：UDP面向报文
- 区别五：TCP需要建立连接消耗资源更多，UDP较少
- 区别六：TCP可以保证数据顺序，UDP不可以

## 3 HTTP

### 3.1 HTTP HTTPS 区别

- HTTPS是HTTP协议的安全封装，HTTP协议是明文传输，HTTPS使用 SSL/TLS进行加密处理
- HTTP默认端口80，HTTPS默认端口443

## 4 MQTT

## 5 WebSocket

基于TCP的全双工通信协议，允许服务端主动向客户端推送数据

客户端和服务端完成一次握手就可以建立长连接

- 建立在TCP协议之上，实现较为容易
- 数据格式比较轻量，性能开销小，通信高效
- 无同源限制
- 协议标识符WS，WSS
