# Centos 安装 PostgreSQL

## 下载源码

[清华大学镜像](https://mirrors.tuna.tsinghua.edu.cn/postgresql/latest/)

解压

## configure

```bash
sudo ./configure --prefix=/opt/postgresql
```

可能需要安装 readline

```bash
sudo yum install readline-devel
```

## 安装

```bash
sudo make -j4 && sudo make install

# 安装contrib目录下的一些工具，是第三方组织的一些工具代码，建议安装
cd contrib
sudo make -j4 && sudo make install
```

初始化

```bash
initdb -E UTF-8 -D /opt/postgresql/data
```

## systemed

`/usr/lib/systemd/system/pgsql.service`

```text
[Unit]
Description=PostgreSQL database server
After=network.target

[Service]
Type=forking
User=postgres
Group=postgres
OOMScoreAdjust=-1000
ExecStart=/opt/postgresql/bin/pg_ctl start -D /opt/postgresql/data -l /opt/postgresql/log/pgsql.log -s -w -t 300
ExecStop=/opt/postgresql/bin/pg_ctl stop -D /opt/postgresql/data -l /opt/postgresql/log/pgsql.log -s -m fast
ExecReload=/opt/pgsql/bin/pg_ctl reload -D /opt/postgresql/data -l /opt/postgresql/log/pgsql.log -s
TimeoutSec=300

[Install]
WantedBy=multi-user.target
```

## 启动

```bash
sudo systemctl start postgresql
sudo systemctl enable postgresql
psql
```

## 配置

### postgrsql

`data/postgrsql.conf`

### pg_hba

`data/pg_hba.conf`

### pg_ident

`data/pg_ident.conf`
