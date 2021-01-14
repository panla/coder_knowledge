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
