# buildx

[TOC]

docker buildx

跨指令集，构建镜像

## 安装

<https://github.com/docker/buildx>

### linux 中安装

linux: `~/.docker/cli-plugins/docker-buildx`

daemon.json 增加 `"experimental": true`

`~/.docker/config.json` 增加 `"experimental": "enabled"`

## 新建一个 build

```bash
docker buildx create --use --name=mybuilder-cn --driver docker-container
```

```bash
docker buildx ls
```

## 安装模拟器

```bash
docker run --rm --privileged tonistiigi/binfmt:latest --install all

docker run --rm --privileged moby/buildkit:latest --install all
```

## build and push

```bash
docker buildx build --platform linux/arm64,linux/amd64 --push -t pankla/nginx-live-server .
```
