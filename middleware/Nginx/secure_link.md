# secure

安全链接, 防盗链

## require

```text
依赖模块

--with-http_secure_link_module
```

## settings

```text
生成md5
    echo -n "时间戳,URL,密钥" | openssl md5 -binary | openssl base64 | tr +/ - | tr -d =
组合成请求
    /test/test.txt?md5=md5生成值&expires=过期时间戳

nginx 配置
    secure_link $arg_md5,$arg_expires;
    secure_link_md5 "$secure_link_expires,$uri,$secret";

    if ($secure_link = "") {
        return 403;
    }
    if ($secure_link = "0") {
        return 410 "please re-request"
    }

校验结果
    空: 验证不通过
    0:  过期
    1:  通过

```
