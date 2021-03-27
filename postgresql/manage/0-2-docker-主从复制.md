# 实验

创建网络

```bash
docker network create --driver bridge --subnet 172.18.0.0/16 --gateway 172.18.0.1 pgsql
```

创建docker容器

```bash
docker run -it --name pgmaster --net pgsql --ip 172.18.0.3 -p 15433:5433 -v /mnt/T1000/downs/used/postgresql-13.1:/srv/postgresql registry.cn-hangzhou.aliyuncs.com/panla/centos8_base:v1 bash

docker run -it --name pgslave --net pgsql --ip 172.18.0.4 -p 25433:5433 -v /mnt/T1000/downs/used/postgresql-13.1:/srv/postgresql registry.cn-hangzhou.aliyuncs.com/panla/centos8_base:v1 bash
```

docker 中安装依赖

```bash
dnf install gcc gcc-c++ make readline-devel ncurses-c++-libs ncurses-devel zlib-devel openssl-devel libuuid-devel llvm-toolset llvm-devel cmake-filesystem llvm llvm-libs pam-devel glibc-common langpacks-zh_CN passwd -y
```

docker 中编译postgrsql

```bash
../configure --prefix=/opt/pgsql --exec-prefix=/opt/pgsql --with-pgport=5433  --with-openssl --with-uuid=e2fs --with-llvm --with-pam

../configure --prefix=/opt/pgsql --exec-prefix=/opt/pgsql --with-pgport=5433 --with-openssl --with-uuid=e2fs --with-llvm --with-pam

make world
make install-world
```

docker 中创建用户，用户组

```bash
groupadd postgres
useradd -g postgres postgres

dnf install passwd -y
passwd postgres
```

env

```text
export PATH="/opt/pgsql/bin:$PATH"
export MANPATH="/opt/pgsql/share/man:$MANPATH"
export LD_LIBRARY_PATH="/opt/pgsql/lib:$LD_LIBRARY_PATH"
```

init

```bash
initdb -D /opt/pgsql/data -E UTF8 --locale=zh_CN.utf8
```

主库配置

```text
isten_address = '*'
tcp_keepalives_idle = 120
tcp_keepalives_interval = 120
password_encryption = scram-sha-256
max_files_per_process = 1000
wal_level = replica
archive_mode = on
archive_command = 'cp %p /data/postgresql/archive/%f '
recovery_target_timeline = 'latest'
max_wal_senders= 10
hot_standby = on
```

start

```bash
pg_ctl -D /opt/pgsql/data start
```

从库同步数据

```bash
pg_basebackup -h 172.18.0.3 -U repuser -p 5433 -F p -X s -v -P -R -D /opt/pgsql/data
```
