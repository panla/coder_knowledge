# MySQL

## 下载解压

[清华大学镜像站](https://mirrors.tuna.tsinghua.edu.cn/mysql/downloads/MySQL-8.0/)

## 准备

创建文件夹

```bash
cd /opt/mysql
sudo mkdir data
sudo mkdir logs
```

创建用户,用户组

```bash
sudo groupadd mysql
sudo useradd -M -s /sbin/nologin mysql -g mysql
```

授权

```bash
cd /opt
sudo chown -R mysql:mysql mysql
```

## 配置文件

### my.cnf

`/etc/my.cnf`

```text
[client]
port = 3306
socket = /opt/mysql/logs/mysql.sock

[mysqld]
user = mysql
server-id = 1
port = 3306
mysqlx_port = 33060
socket = /opt/mysql/logs/mysql.sock
mysqlx_socket = /opt/mysql/logs/mysqlx.sock
basedir = /opt/mysql
datadir = /opt/mysql/data
log-error = /opt/mysql/logs/mysql.err
pid-file = /opt/mysql/logs/mysql.pid
default-authentication-plugin=mysql_native_password
character-set-server = utf8mb4
collation-server = utf8mb4_general_ci
init_connect='SET NAMES utf8mb4'
```

### service

`/usr/lib/systemd/system/mysql.service`

```text
[Unit]
Description=MySQL Server
Documentation=man:mysqld(8)
Documentation=http://dev.mysql.com/doc/refman/en/using-systemd.html
After=network.target
After=syslog.target

[Install]
WantedBy=multi-user.target

[Service]
User=mysql
Group=mysql
Type=notify
# Disable service start and stop timeout logic of systemd for mysqld service.
TimeoutSec=0
# Execute pre and post scripts as root
PermissionsStartOnly=true
# Start main service
ExecStart=/opt/mysql/bin/mysqld --defaults-file=/etc/my.cnf $MYSQLD_OPTS
# Use this to switch malloc implementation
EnvironmentFile=-/etc/sysconfig/mysql
# Sets open_files_limit
LimitNOFILE = 10000
Restart=on-failure
RestartPreventExitStatus=1
# Set enviroment variable MYSQLD_PARENT_PID. This is required for restart.
Environment=MYSQLD_PARENT_PID=1
PrivateTmp=false
```

## 初始化

环境变量

```text
export PATH="/opt/mysql/bin:$PATH"
```

```bash
mysqld --initialize --console
```

## so 文件

```bash
sudo ln -s /usr/lib64/libtinfo.so.6.1 /usr/lib64/libtinfo.so.5
```

## 启动

```bash
sudo systemctl daemon-reload
sudo systemctl start mysql
sudo systemctl enable mysql
```

## 修改密码

```text
ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY 'new password';

flush privileges;
exit
```
