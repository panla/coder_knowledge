# DBeaver

## 下载

[github](https://github.com/dbeaver/dbeaver/releases)
[代下-1](https://www.offcloud.com/)
[代下-2](https://shrill-pond-3e81.hunsh.workers.dev/)

## 安装-配置

窗口->首选项->连接->驱动->maven->添加

```text
name
http://maven.aliyun.com/nexus/content/repositories/central/
```

## build

```bash
git clone https://github.com/dbeaver/dbeaver.git dbeaver
git clone git@gitee.com:mirrors/dbeaver.git dbeaver
cd dbeaver
# 需要已经配置 java maven
mvn package
```
