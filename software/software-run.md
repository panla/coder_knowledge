# software

## 1 git

### 1.1 git 部分配置

- 生成公钥私钥

```bash
ssh-keygen -t rsa -C "email"
```

- 配置用户名,邮箱

```bash
git config --global user.email "email"
git config --global user.name "name"
git config --global core.quotepath false
git config --global pull.rebase false
```

- vim

```bash
git config --global core.editor "vim"
```

- 保存账户密码

```bash
git config --global credential.helper store
```

### 1.2 别名

```bash
git config --global alias.co checkout
git config --global alias.ci commit
git config --global alias.st status
```

### 1.3 加速？

```text
https://www.ipaddress.com/

github.com

github.global.ssl.fastly.net

assets-cdn.github.com

avatars1.githubusercontent.com

====================================================
199.232.69.194 github.global.ssl.fastly.net

140.82.112.4 github.com

185.199.108.153 assert-cdn.github.com

185.199.108.133 avatars1.githubusercontent.com

====================================================

修改 /etc/resolv.conf 新增
nameserver 8.8.8.8
nameserver 8.8.4.4
```

## 2 ssh config

```text
Host test
    HostName ip or server_name
    User user
    Port 22
    IdentityFile ~/.ssh/id_rsa
```

## 3 dbeaver

### 3.1 下载

[github](https://github.com/dbeaver/dbeaver/releases)
[代下-1](https://www.offcloud.com/)
[代下-2](https://shrill-pond-3e81.hunsh.workers.dev/)

### 3.2 安装-配置

窗口->首选项->连接->驱动->maven->添加

```text
name
https://maven.aliyun.com/repository/public
```

### 3.3 build

```bash
git clone https://github.com/dbeaver/dbeaver.git dbeaver
git clone xxx.xxx.xxx.com/dbeaver.git dbeaver
cd dbeaver
# 需要已经配置 java maven
mvn package
```

### 3.4 desktop

```text
[Desktop Entry]
Version=1.0
Type=Application
Name=DBeaver Community
GenericName=UniversaL Database Manager
Comment=Universal Database Manager and SQL Client.
Path=/home/opt/dbeaver/
Exec=/home/opt/dbeaver/dbeaver
Icon=/home/opt/dbeaver/dbeaver.png
Categories=IDE;Development
WM_CLASS=DBeaver
StartupWMClass=DBeaver
StartupNotify=true
Keywords=Database;SQL;IDE;JDBC;ODBC;MySQL;PostgreSQL;Oracle;DB2;MariaDB
MimeType=application/sql
Terminal=false
```

## 4 jdk jre

```text
export JAVA_HOME=/home/opt/jdk-11.0.9
export CLASSPATH=.:${JAVA_HOME}/lib
export PATH=${JAVA_HOME}/bin:$PATH
```

## 5 maven

### 5.1 下载

[maven.apache.org](http://maven.apache.org/download.cgi)

解压到 `/home/opt/apache-maven`

### 5.2 setting.xml 配置

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

### 5.3 profile

```text
export PATH="/opt/apache-maven/bin:$PATH"
```
