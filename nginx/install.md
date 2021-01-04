# nginx

## user group

```bash
sudo groupadd nginx
sudo useradd nginx -g nginx -M -s /sbin/nologin
```

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

sudo make
sudo make install
```

配置参数

```text
--prefix=/opt/nginx --user=nginx --group=nginx --with-threads --with-file-aio --with-http_ssl_module --with-http_v2_module --with-http_realip_module --with-http_addition_module --with-http_xslt_module --with-http_xslt_module=dynamic --with-http_image_filter_module --with-http_image_filter_module=dynamic --with-http_sub_module --with-http_dav_module --with-http_flv_module --with-http_mp4_module --with-http_gunzip_module --with-http_gzip_static_module --with-http_auth_request_module --with-http_random_index_module --with-http_secure_link_module --with-http_degradation_module --with-http_slice_module --with-http_stub_status_module --with-http_perl_module --with-http_perl_module=dynamic --with-mail --with-mail=dynamic --with-mail_ssl_module --with-stream --with-stream=dynamic --with-stream_ssl_module --with-stream_realip_module --with-stream_ssl_preread_module
```

其他参数

```text
http://nginx.org/en/docs/

--with-pcre=path    源码
--with-pcre-jit
--with-zlib=path    源码
--with-openssl=path 源码
--with-perl=path    可执行文件

--with-stream_geoip_module
--with-stream_geoip_module=dynamic
--with-http_geoip_module --with-http_geoip_module=dynamic
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

## 注意

注: 提示缺少什么就安装什么，比如 `gd-devel` `gcc`

```text
the HTTP XSLT module requires the libxml2/libxslt

the HTTP image filter module requires the GD library

error: perl module ExtUtils::Embed is required
```

```bash
sudo dnf install gcc gcc-c++ make libxslt-devel libxml2-devel gd-devel perl-devel perl-ExtUtils-Embed
```
