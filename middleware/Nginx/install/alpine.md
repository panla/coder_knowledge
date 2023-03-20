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
--sbin-path=/usr/local/sbin/nginx \
--conf-path=/etc/nginx/nginx.conf \
--error-log-path=/usr/local/nginx/logs/error.log \
--http-log-path=/usr/local/nginx/logs/access.log \
--pid-path=/usr/local/nginx/run/nginx.pid \
--lock-path=/usr/local/nginx/run/nginx.lock \
--user=root \
--group=root \
--with-select_module \
--with-poll_module \
--with-threads \
--with-file-aio \
--with-http_ssl_module \
--with-http_v2_module \
--with-http_realip_module \
--with-http_addition_module \
--with-http_image_filter_module=dynamic \
--with-http_geoip_module=dynamic \
--with-http_sub_module \
--with-http_dav_module \
--with-http_flv_module \
--with-http_mp4_module \
--with-http_gunzip_module \
--with-http_gzip_static_module \
--with-http_auth_request_module  \
--with-http_random_index_module \
--with-http_secure_link_module \
--with-http_degradation_module \
--with-http_slice_module \
--with-http_stub_status_module \
--with-stream \
--with-stream=dynamic \
--with-stream_realip_module \
--with-stream_geoip_module=dynamic \
--with-stream_ssl_preread_module \
--with-google_perftools_module \
--with-mail \
--with-mail_ssl_module \
--with-compat \

--with-pcre
--with-pcre-jit \
--with-pcre=/opt/build/pcre2 \
--with-openssl=/opt/build/openssl \
--with-zlib=/opt/build/zlib \
--add-module=/opt/build/nginx-http-flv-module \
--add-module=/opt/build/ngx_http_consistent_hash \

```

```text
--with-cc-opt="-I /usr/local/include -D FD_SETSIZE=2048 -static -static-libgcc"
--with-ld-opt="-L /usr/local/lib"
```
