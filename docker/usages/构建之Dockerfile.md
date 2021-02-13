# Dockerfile 命令

镜像构建文件

Dockerfile -- build --> image

## 上下文

上下文目录

## Dockerfile 参数

- FROM 构建的镜像是基于什么镜像
- WORKDIR 进入到容器后的工作目录
- ADD copy 文件并解压，并删除原文件
- COPY 复制文件
- ENV 定义环境变量
- EXPOSE 容器对外暴露的端口，加了以后才能 -p
- ENTRYPOINT 启动容器时执行的命令
- MAINTAINER
- RUN 运行 shell
- CMD 启动容器时执行的命令

## usage
