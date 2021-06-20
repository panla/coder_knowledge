# MySQL

## 下载解压

[清华大学镜像站](https://mirrors.tuna.tsinghua.edu.cn/mysql/downloads/MySQL-8.0/)

## 准备

创建文件夹

```bash
cd /opt/mysql
sudo mkdir data
sudo mkdir logs
sudo mkdir logs/binlogs
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

`/opt/mysql/my.cnf`

```text
[client]
port = 3306
socket = /home/opt/mysql/tmp/mysql.sock

[mysqld]
user = mysql
server-id = 1
port = 3306
mysqlx_port = 33060
socket = /home/opt/mysql/tmp/mysql.sock
mysqlx_socket = /home/opt/mysql/tmp/mysqlx.sock

# mysql 基本路径
basedir = /home/opt/mysql

# 数据路径
datadir = /home/opt/mysql/data

# bin-log
log-bin = /home/opt/mysql/logs/binlogs/binlog

# log-error
log-error = /home/opt/mysql/logs/mysql.err

# 进程号文件
pid-file = /home/opt/mysql/tmp/mysql.pid

# 密码校验
default-authentication-plugin=mysql_native_password

# 数据库默认字符集
character-set-server = utf8mb4

# 数据库字符集对应一些排序等规则
collation-server = utf8mb4_general_ci

# 服务器为每个连接的客户端执行的字符串，设置 client 连接 mysql 时的字符集
init_connect='SET NAMES utf8mb4'

# 服务器关闭非交互连接之前等待活动的秒数
wait_timeout = 120

# 设置时区
default-time-zone=+08:00

# 最大连接数
max_connections=2000
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
TimeoutSec=0
PermissionsStartOnly=true
ExecStart=/home/opt/mysql/bin/mysqld --defaults-file=/home/opt/mysql/my.cnf $MYSQLD_OPTS
# Use this to switch malloc implementation
EnvironmentFile=-/etc/sysconfig/mysql
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
export PATH="/home/opt/mysql/bin:$PATH"
```

```bash
mysqld --defaults-file=/home/opt/mysql/my.cnf --initialize --console
```

## so 文件

```bash
# bin/mysqld: error while loading shared libraries: libaio.so.1
dnf install -y libaio

# bin/mysqld: error while loading shared libraries: libnuma.so.1
dnf install -y numactl

# bin/mysql: error while loading shared libraries: libtinfo.so.5
sudo ln -s /usr/lib64/libtinfo.so.6.1 /usr/lib64/libtinfo.so.5
```

## 启动

```bash
# 注意 selinux 配置
sudo systemctl daemon-reload
sudo systemctl start mysql
sudo systemctl enable mysql
```

## 修改密码

mysql.sock 应该是在编译时确定的位置，不然就需要在 /tmp/mysql.sock 方软连接或真实文件

刷新密码和允许远程连接

```sql
use mysql;
ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY 'new password';

use mysql;
UPDATE user SET `Host` = '%' WHERE `User` = 'root' LIMIT 1;

flush privileges;
exit
```

开放防火墙

```bash
sudo firewall-cmd --add-port=3306/tcp --zone=public --permanent
sudo systemctl restart firewalld.service
```
