# Dockerfile 命令

[TOC]

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
- RUN 运行 shell
- CMD 启动容器时执行的命令
- ENTRYPOINT 启动容器时执行的命令
- MAINTAINER
- ARG 设置构建环境的环境变量
- USER 指定执行 RUN CMD ENTRYPOINT等的身份

## usage

### ENV

```text
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Asia/Shanghai
```

```text
-e TZ="Asia/Shanghai" -e "LC_ALL=C.UTF-8" -e "LANG=C.UTF-8"
```

## CMD

```text
CMD ["echo", "$HOME"]
```
