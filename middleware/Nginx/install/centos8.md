# nginx

[TOC]

## 附加软件源码

[pcre源码](https://ftp.pcre.org/pub/pcre)
[zlib](https://github.com/madler/zlib)
[openssl](https://github.com/openssl/openssl)

## make install nginx on linux

[下载地址](http://nginx.org/en/download.html)

解压后运行

```bash
cd nginx
sudo ./configure 配置参数

sudo make -j4
sudo make install
```

配置参数

```bash
./configure --user=root --group=root --prefix=/opt/nginx --sbin-path=/opt/nginx/sbin/nginx --with-file-aio --with-http_v2_module --with-http_realip_module --with-http_addition_module --with-http_xslt_module --with-http_xslt_module=dynamic --with-http_image_filter_module --with-http_image_filter_module=dynamic --with-http_geoip_module --with-http_geoip_module=dynamic --with-http_sub_module --with-http_dav_module --with-http_flv_module --with-http_mp4_module --with-http_gunzip_module --with-http_gzip_static_module --with-http_auth_request_module --with-http_random_index_module --with-http_secure_link_module --with-http_degradation_module --with-http_slice_module --with-http_stub_status_module --with-http_perl_module --with-http_perl_module=dynamic --with-perl=path --with-mail --with-mail=dynamic --with-mail_ssl_module --with-stream --with-stream=dynamic --with-stream_realip_module --with-stream_geoip_module --with-stream_geoip_module=dynamic --with-stream_ssl_preread_module --with-cpp_test_module  --with-perl=/usr/bin/perl5.26.3 --with-pcre-jit --with-pcre=/srv/temp/pcre-8.44 --with-zlib=/srv/temp/zlib-1.2.11 --with-openssl=/srv/temp/openssl-1.1.1j
```

```bash
./configure --user=root --group=root --prefix=/opt/nginx --sbin-path=/opt/nginx/sbin/nginx --with-file-aio --with-http_v2_module --with-http_realip_module --with-http_addition_module --with-http_xslt_module --with-http_xslt_module=dynamic --with-http_image_filter_module --with-http_image_filter_module=dynamic --with-http_sub_module --with-http_dav_module --with-http_flv_module --with-http_mp4_module --with-http_gunzip_module --with-http_gzip_static_module --with-http_auth_request_module --with-http_random_index_module --with-http_secure_link_module --with-http_degradation_module --with-http_slice_module --with-http_stub_status_module --with-http_perl_module --with-http_perl_module=dynamic --with-perl=path --with-mail --with-mail=dynamic --with-mail_ssl_module --with-stream --with-stream=dynamic --with-stream_realip_module --with-stream_ssl_preread_module --with-perl=/usr/bin/perl5.26.3 --with-pcre-jit --with-pcre=/srv/temp/pcre-8.44 --with-zlib=/srv/temp/zlib-1.2.11 --with-openssl=/srv/temp/openssl-1.1.1j
```

其他参数

```text
http://nginx.org/en/docs/

--with-pcre=path    源码
--with-pcre-jit
--with-zlib=path    源码
--with-openssl=path 源码
--with-perl=path    可执行文件
```

nginx -V (debian:bullseye-slim)

```text
nginx version: nginx/1.22.0
built by gcc 10.2.1 20210110 (Debian 10.2.1-6)
built with OpenSSL 1.1.1n  15 Mar 2022
TLS SNI support enabled
configure arguments: --sbin-path=/usr/local/sbin/nginx --conf-path=/etc/nginx/nginx.conf --error-log-path=/var/log/nginx/error.log --http-log-path=/var/log/nginx/access.log --pid-path=/var/run/nginx/nginx.pid --lock-path=/var/lock/nginx.lock --http-client-body-temp-path=/tmp/nginx-client-body --with-http_realip_module --with-stream_realip_module --with-http_ssl_module --with-stream_ssl_preread_module --with-http_v2_module --with-http_addition_module --with-http_flv_module --with-http_mp4_module --with-http_gunzip_module --with-http_gzip_static_module --with-stream --with-stream=dynamic --with-threads --with-file-aio --with-http_sub_module --with-http_dav_module --with-http_auth_request_module --with-http_slice_module --with-http_stub_status_module --with-http_random_index_module --with-http_secure_link_module --with-http_degradation_module --add-module=/tmp/build/nginx-http-flv-module
```

## systemctl管理

`/usr/lib/systemd/system/nginx.service`

```text
[Unit]
Description=The nginx HTTP and reverse proxy server
After=network-online.target remote-fs.target nss-lookup.target
Wants=network-online.target

[Service]
PIDFile=/opt/nginx/logs/nginx.pid
ExecStartPre=/opt/nginx/sbin/nginx -t
ExecStart=/opt/nginx/sbin/nginx -g "daemon off;" -c /opt/nginx/conf/nginx.conf
ExecReload=/bin/kill -s HUP $MAINPID
KillSignal=SIGQUIT
TimeoutStopSec=5
KillMode=mixed
PrivateTmp=true

[Install]
WantedBy=multi-user.target
```

## 错误以及应对方法

```bash
dnf install gcc gcc-c++ make libxslt-devel libxml2-devel gd-devel perl geoip geoip-devel
```

缺少 c 编译器

```bash
dnf install gcc gcc-c++ make
```

缺少 libxml2/libxslt

```bash
dnf install libxslt-devel libxml2-devel
```

the HTTP image filter module requires the GD library

```bash
# epel
dnf install gd-devel
```

perl 5.8.6 or higher is required

```bash
dnf install perl
```

error: the GeoIP module requires the GeoIP library.

```text
安装 epel 后 dnf install geoip geoip-devel
```

注意 **nginx, openssl, zlib, pcre 里的 config configure 执行权限**
