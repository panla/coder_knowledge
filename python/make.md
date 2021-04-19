# 编译 Python

> Python:3.8.9

[华为镜像](https://mirrors.huaweicloud.com/python/)

## error

```bash
./configure --prefix=/opt/python --enable-optimizations
```

```bash
# dnf groupinstall "Development tools"
dnf install kernel-devel kernel-headers gcc gcc-c++ make zlib zlib-devel bzip2 bzip2-devel libffi-devel openssl-devel ncurses-devel sqlite-devel readline-devel tk-devel gdbm-devel xz-devel
```
