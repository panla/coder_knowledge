# EPEL 仓库

> EPEL 仓库提供了额外的软件包，这些软件在 CentOS 8 和 RHEL 8 的默认软件包仓库中不可用。

## RHEL 8 中安装启用

```bash
dnf install https://mirrors.tuna.tsinghua.edu.cn/epel/epel-release-latest-8.noarch.rpm -y

dnf install https://dl.fedoraproject.org/pub/epel/epel-release-latest-8.noarch.rpm -y

dnf repolist epel
```

## CentOS 8 中安装启用

```bash
dnf install epel-release -y
```

## 换源

```bash
sed -e 's!^metalink=!#metalink=!g' -e 's!^#baseurl=!baseurl=!g' -e 's!//download\.fedoraproject\.org/pub!//mirrors.tuna.tsinghua.edu.cn!g' -e 's!http://mirrors\.tuna!https://mirrors.tuna!g' -i /etc/yum.repos.d/epel.repo /etc/yum.repos.d/epel-testing.repo
```

```text
[epel]
name=Extra Packages for Enterprise Linux $releasever - $basearch
baseurl=https://mirrors.tuna.tsinghua.edu.cn/epel/$releasever/Everything/$basearch
#metalink=https://mirrors.fedoraproject.org/metalink?repo=epel-$releasever&arch=$basearch&infra=$infra&content=$contentdir
enabled=1
gpgcheck=1
countme=1
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-EPEL-8

[epel-modular]
name=Extra Packages for Enterprise Linux Modular $releasever - $basearch
baseurl=https://mirrors.tuna.tsinghua.edu.cn/epel/$releasever/Modular/$basearch
#metalink=https://mirrors.fedoraproject.org/metalink?repo=epel-modular-$releasever&arch=$basearch&infra=$infra&content=$contentdir
enabled=1
gpgcheck=1
countme=1
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-EPEL-8

[epel-playground]
name=Extra Packages for Enterprise Linux $releasever - Playground - $basearch
baseurl=https://mirrors.tuna.tsinghua.edu.cn/epel/playground/$releasever/Everything/$basearch/os
#metalink=https://mirrors.fedoraproject.org/metalink?repo=playground-epel$releasever&arch=$basearch&infra=$infra&content=$contentdir
enabled=1
gpgcheck=1
countme=1
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-EPEL-8
```
