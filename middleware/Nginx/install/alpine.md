# Alpine

## depend

```bash
sed -i 's/dl-cdn.alpinelinux.org/mirrors.aliyun.com/g' /etc/apk/repositories \
&& apk update --no-cache \
&& apk add --no-cache \
    libc-dev gcc make \
    linux-headers \
    ca-certificates \
    tzdata \
    yasm freetype-dev \
    gd-dev geoip-dev \
    libmaxminddb-dev \
```

## Compilation

```bash
./configure --with-cc-opt='-static -static-libgcc' --with-ld-opt=-static \
--prefix=/usr/local/nginx \
--user=root \
--group=root \
--sbin-path=/usr/local/sbin/nginx \
--conf-path=/etc/nginx/nginx.conf \
--error-log-path=/var/log/nginx/error.log \
--http-log-path=/var/log/nginx/access.log \
--pid-path=/var/run/nginx/nginx.pid \
--lock-path=/var/lock/nginx.lock \
--http-client-body-temp-path=/opt/nginx-client-body \
--with-file-aio \
--with-threads \
--with-http_ssl_module \
--with-http_realip_module \
--with-http_addition_module \
--with-http_v2_module \
--with-http_sub_module \
--with-http_dav_module \
--with-http_flv_module \
--with-http_mp4_module \
--with-http_gunzip_module \
--with-http_gzip_static_module \
--with-http_random_index_module \
--with-http_secure_link_module \
--with-http_slice_module \
--with-http_degradation_module \
--with-http_auth_request_module  \
--with-http_stub_status_module \
--with-http_geoip_module=dynamic \
--with-stream \
--with-stream=dynamic \
--with-stream_realip_module \
--with-stream_ssl_preread_module \
--with-stream_geoip_module=dynamic \
--with-mail \
--with-mail_ssl_module \
--with-compat \
--with-pcre-jit \
--with-pcre=/opt/build/pcre2 \
--with-openssl=/opt/build/openssl \
--with-zlib=/opt/build/zlib \
--add-module=/opt/build/nginx-http-flv-module \
--add-module=/opt/build/ngx_http_consistent_hash \

```
