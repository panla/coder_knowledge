# Docker

[TOC]

## 1 Docker 网络通信

### 1.1 bridge

为容器创建独立的网络命名空间，容器具有独立的网卡等所有的单独的网络栈

完成容器间，容器与宿主机间的网络隔离

通过docker0网桥与Iptables NAT表完成容器与宿主机通信

### 1.2 host

直接使用容器宿主机的网络命名空间

容器端口即为宿主机占用的端口
