# 给 CentOS 升级内核

[TOC]

## 参考

[CSDN 文章](https://blog.csdn.net/dengshulei/article/details/103704417)
[清华大学镜像](https://mirrors.tuna.tsinghua.edu.cn/help/elrepo/)

## 步骤

### 查看当前Linux内核版本

```bash
uname -sr
```

### 查看有无内核升级包，并处理

```bash
rpm -qa | grep elrepo.noarch
sudo dnf -y remove elrepo-release-8.0-2.el8.elrepo.noarch.rpm
```

### 配置elrepo

```bash
sudo rpm --import https://www.elrepo.org/RPM-GPG-KEY-elrepo.org
sudo yum install https://www.elrepo.org/elrepo-release-8.el8.elrepo.noarch.rpm
sudo vim /etc/yum.repos.d/elrepo.repo
# 注释 mirrorlist
# 并将 elrepo.org/linux 替换为 mirrors.tuna.tsinghua.edu.cn/elrepo
sudo dnf makecache
```

```text
[elrepo]
name=ELRepo.org Community Enterprise Linux Repository - el8
baseurl=https://mirrors.tuna.tsinghua.edu.cn/elrepo/elrepo/el8/$basearch/
    http://mirrors.coreix.net/elrepo/elrepo/el8/$basearch/
    http://mirror.rackspace.com/elrepo/elrepo/el8/$basearch/
    http://linux-mirrors.fnal.gov/linux/elrepo/elrepo/el8/$basearch/
mirrorlist=http://mirrors.elrepo.org/mirrors-elrepo.el8
enabled=1
gpgcheck=1
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-elrepo.org

[elrepo-kernel]
name=ELRepo.org Community Enterprise Linux Kernel Repository - el8
baseurl=https://mirrors.tuna.tsinghua.edu.cn/elrepo/kernel/el8/$basearch/
    http://mirrors.coreix.net/elrepo/kernel/el8/$basearch/
    http://mirror.rackspace.com/elrepo/kernel/el8/$basearch/
    http://linux-mirrors.fnal.gov/linux/elrepo/kernel/el8/$basearch/
mirrorlist=http://mirrors.elrepo.org/mirrors-elrepo-kernel.el8
enabled=0
gpgcheck=1
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-elrepo.org

[elrepo-extras]
name=ELRepo.org Community Enterprise Linux Extras Repository - el8
baseurl=https://mirrors.tuna.tsinghua.edu.cn/elrepo/extras/el8/$basearch/
    http://mirrors.coreix.net/elrepo/extras/el8/$basearch/
    http://mirror.rackspace.com/elrepo/extras/el8/$basearch/
    http://linux-mirrors.fnal.gov/linux/elrepo/extras/el8/$basearch/
mirrorlist=http://mirrors.elrepo.org/mirrors-elrepo-extras.el8
enabled=1
gpgcheck=1
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-elrepo.org
```

### 查看有无最新版本内核

```bash
sudo dnf --disablerepo="*" --enablerepo="elrepo-kernel" list available
```

### 安装内核包 1

```bash
sudo dnf install -y --enablerepo=elrepo-kernel kernel-ml kernel-ml-core kernel-ml-devel kernel-ml-modules
```

### 检查默认启动项并重启

```bash
sudo grubby --default-kernel
reboot
uname -sr
```

### 删除旧内核

```bash
sudo dnf -y remove kernel-xxxx
```

### 安装内核包 2

```bash
sudo dnf install -y --enablerepo=elrepo-kernel kernel-ml-doc kernel-ml-headers kernel-ml-modules-extra kernel-ml-tools kernel-ml-tools-libs kernel-ml-tools-libs-devel --allowerasing
```

### 恢复 gcc 等组件

```bash
sudo dnf install annobin gcc-c++ make
```
