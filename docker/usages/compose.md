# Docker Compose

负责对 Docker 容器,集群的快速编排，定义和运行多个容器的应用

通过 docker-compose.yml 模板文件来定义一组相关联的应用容器为一个项目

- 服务，一个应用的容器，可以包括若干运行相同镜像的容器实例
- 项目，一组关联的应用容器组成的一个完整的业务单元

## 下载

```bash
sudo curl -L "https://github.com/docker/compose/releases/download/1.28.2/docker-compose-$(uname -s)-$(uname -m)" -o /opt/docker/bin/docker-compose
cd /opt/docker/bin
chmod 777 docker-compose
cd /usr/bin
sudo ln -s /opt/docker/bin/docker-compose ./
```
