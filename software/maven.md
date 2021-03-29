# maven

## 下载

[maven.apache.org](http://maven.apache.org/download.cgi)

解压到 `/home/opt/apache-maven`

## setting.xml 配置

`conf/settings.xml`

一 阿里云镜像

```text
<mirrors>
  <mirror>
    <id>aliyunmaven</id>
    <mirrorOf>*</mirrorOf>
    <name>Aliyun Mirror.</name>
    <url>https://maven.aliyun.com/repository/public</url>
  </mirror>
</mirrors>
```

二 本地缓存目录

```text
<localRepository>/home/opt/apache-maven-3.6.3/repository</localRepository>
```
