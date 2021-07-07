# Docker 下的使用

## 安装

- [docker-compose.yml 参考](./docker-compose.yml)

## 访问

默认访问地址，`BASE_URL:8080`

- 初始密码,`secrets/initialAdminPassword`

## 部署项目等操作

### 在容器中的 jenkins 中使用宿主机的 docker 命令(单机)

为了能够使 docker 容器中的 jenkins 执行 docker 命令，
需要挂载数据卷。docker,docker.sock 如此，就可以

```text
-v /var/run/docker.sock:/var/run/docker.sock
-v /usr/bin/docker:/usr/bin/docker
-v /usr/bin/docker-compose:/usr/bin/docker-compose
```

### jenkins 与 项目所在不同服务器

通过 ssh 等来远程操作服务器

## 配置

### 换源安装插件

服务启动后，在访问前修改，修改完再重启

1. 修改 `hudson.model.UpdateCenter.xml`：

    把原先的 `<url>https://updates.jenkins.io/update-center.json</url>`

    修改为 `<url>https://mirrors.tuna.tsinghua.edu.cn/jenkins/updates/update-center.json</url>`

2. 修改 `updates/default.json`：
    原因: 镜像上的 `update-center.json` 指向境外.

    镜像源上和本地各有一个映射文件, 修改本地的,来达到效果

    ```bash
    sed -i 's/http:\/\/updates.jenkins-ci.org\/download/https:\/\/mirrors.tuna.tsinghua.edu.cn\/jenkins/g' updates/default.json

    sed -i 's/http:\/\/www.google.com/https:\/\/www.baidu.com/g' updates/default.json
    ```

### 秘钥，密码

`Dashboard/系统管理/安全/Manage Credentials/`

设置访问 github/gitlab/服务器 等账户密码，公私钥
