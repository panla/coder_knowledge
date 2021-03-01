# PostgreSQL 安装

## 下载源码

[清华大学镜像](https://mirrors.tuna.tsinghua.edu.cn/postgresql/source/v13.1/)

## 编译安装

安装所需工具

```bash
# error: no acceptable C compiler found in $PATH
dnf install gcc gcc-c++ make

# configure: error: readline library not found
dnf install readline-devel ncurses-c++-libs ncurses-devel

# configure: error: zlib library not found
dnf install zlib-devel

# configure: error: library 'crypto' is required for OpenSSL
dnf install openssl-devel

# configure: error: library 'uuid' is required for E2FS UUID
dnf install libuuid-devel

# configure: error: header file <systemd/sd-daemon.h> is required for systemd support
# docker 中不行
```

```bash
mkdir build
cd build

../configure --prefix=/opt/pgsql --exec-prefix=/opt/pgsql --with-pgport=5432 --with-openssl --with-uuid=e2fs

make world
make install-world
make distclean
```

--with-systemd

## 环境变量

```text
export PATH="/opt/pgsql/bin:$PATH"
export MANPATH="/opt/pgsql/share/man:$MANPATH"
export LD_LIBRARY_PATH="/opt/pgsql/lib:$LD_LIBRARY_PATH"
```

## systemd

```text
[Unit]
Description=PostgreSQL database server
Documentation=man:postgres(1)

[Service]
Type=forking
User=postgres
ExecStart=/opt/pgsql/bin/pg_ctl -D /opt/pgsql/data -l /opt/pgsql/logs/pgsql.log start
ExecStop=/opt/pgsql/bin/pg_ctl stop -D /opt/pgsql/data -l /opt/pgsql/logs/pgsql.log -m fast
ExecReload=/opt/pgsql/bin/pg_ctl reload -D /opt/pgsql/data -l /opt/pgsql/logs/pgsql.log -s
TimeoutSec=60

[Install]
WantedBy=multi-user.target
```

## 初始化

```bash
initdb -D /opt/pgsql/data -E UTF8 --locale=zh_CN.utf8
```

## 配置

`opt/pgsql/data/postgresql.conf`

```text
unix_socket_directories = '/opt/pgsql/tmp'
unix_socket_group = 'postgres'
unix_socket_permissions = 0777
```

```bash
ln -s /opt/pgsql/tmp/.s.PGSQL.5432 /tmp/.s.PGSQL.5432
ln -s /opt/pgsql/tmp/.s.PGSQL.5432.lock /tmp/.s.PGSQL.5432.lock
```

## 启动

```bash
pg_ctl -D /opt/pgsql/data -l /opt/pgsql/logs/pgsql.log start
sudo systemctl daemon-reload
sudo systemctl start pgsql
sudo systemctl status pgsql
```
