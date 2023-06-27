# PostgreSQL 安装

[TOC]

## 下载源码

[清华大学镜像](https://mirrors.tuna.tsinghua.edu.cn/postgresql/source/v13.1/)

## 编译安装

### 安装所需工具

推荐在docker里编译，这样不干扰外部环境

```bash
dnf install gcc gcc-c++ make readline-devel ncurses-c++-libs ncurses-devel zlib-devel openssl-devel libuuid-devel llvm-toolset llvm-devel cmake-filesystem llvm llvm-libs pam-devel  glibc-common langpacks-zh_CN passwd -y

# configure: error: no acceptable C compiler found in $PATH
dnf install gcc gcc-c++ make

# configure: error: readline library not found
dnf install readline-devel ncurses-c++-libs ncurses-devel

# configure: error: zlib library not found
dnf install zlib-devel

# configure: error: library 'crypto' is required for OpenSSL
dnf install openssl-devel

# configure: error: library 'uuid' is required for E2FS UUID
dnf install libuuid-devel

# configure: error: llvm-config not found
dnf install llvm-toolset llvm-devel cmake-filesystem llvm llvm-libs

# configure: error: library 'pam' is required for PAM
dnf install pam-devel

# configure: error: header file <systemd/sd-daemon.h> is required for systemd support
# docker 中不行
```

### configure make

```bash
mkdir build
cd build

../configure --prefix=/opt/pgsql --exec-prefix=/opt/pgsql --with-pgport=5432 --with-openssl --with-uuid=e2fs --with-llvm --with-pam

make world
make install-world
make distclean
```

--with-systemd

### configure 部分参数

```text
--prefix=dir            把所有文件装在目录PREFIX下的目录中
--exec-prefix=dir       默认 等于PREFIX并且体系相关和体系无关的文件都会安装到同一棵目录树下
--with-pgport=5432      服务端和客户端默认端口，默认5432
--with-openssl          编译SSL模块连接支持
--with-uuid=e2f3        使用指定的UUID库编译uuid-issp模块
--with-llvm             支持基于LLVM的JIT编译
--with-pam              编译PAM（可插拔认证模块）支持

--bindir=dir            可执行程序指定目录，默认 prefix/bin
--sysconfdir=dir        各种各样配置文件的目录，默认 prefix/etc
--libdir=dir            安装库和动态加载模块的目录，默认 prefix/lib
--inludedir=dir         C/C++ 头文件的目录，默认 prefix/include
--datrootdir=dir        只读数据文件的根目录，默认 prefix/share
--datadir=dir           程序使用的只读数据文件的目录，默认 datarootdir
--localedir=dir         安装区域的目录，默认 datarootdir/locale
--mandir=dir            手册页安装目录，默认 datarootdir/man
--docdir=dir            文档文件根目录，默认 datarootdir
--htmldir=dir           HTML格式文档目录，默认 datarootdir

--with-segsize=1        段尺寸，以G字节计算，默认1G
--with-blocksize=8      块尺寸，以K字节计算，2的幂并且在1--32之间，默认8K
--with-wal-blocksize=8  WAL块尺寸，以 K 字节计，2的幂并且在1--64之间。这是 WAL 日志存储和I/O的单位，默认8K
```

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
ExecStart=/opt/pgsql/bin/pg_ctl -D /opt/pgsql/data start
ExecStop=/opt/pgsql/bin/pg_ctl stop -D /opt/pgsql/data -m fast
ExecReload=/opt/pgsql/bin/pg_ctl reload -D /opt/pgsql/data -s
TimeoutSec=60

[Install]
WantedBy=multi-user.target
```

## 初始化

安装中文支持

```bash
dnf install glibc-common
dnf install -y langpacks-zh_CN
```

```bash
sudo groupadd postgres
sudo useradd -g postgres postgres
sudo passwd postgres

su - postgres
initdb -D /opt/pgsql/data -E UTF8 --locale=zh_CN.utf8
```

initdb 部分参数

```text
-D --pgdata             location for this db cluster
-E --encoding=UTF       设置默认编码
--locale=zh_CN.utf8     设置默认本地语言
-U --username           db superuser
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
pg_ctl -D /opt/pgsql/data start
sudo systemctl daemon-reload
sudo systemctl start pgsql
sudo systemctl status pgsql
```
