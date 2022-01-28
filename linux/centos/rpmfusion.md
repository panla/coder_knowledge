# rpmfusion

[toc]

## 安装

```bash
sudo dnf install https://mirrors.tuna.tsinghua.edu.cn/rpmfusion/free/el/rpmfusion-free-release-8.noarch.rpm
```

## 修改

```text
[rpmfusion-free-updates]
name=RPM Fusion for EL 8 - Free - Updates
baseurl=https://mirrors.tuna.tsinghua.edu.cn/rpmfusion/free/el/updates/8/$basearch/
#mirrorlist=http://mirrors.rpmfusion.org/mirrorlist?repo=free-el-updates-released-8&arch=$basearch
enabled=1
gpgcheck=1
gpgkey=file:///etc/pki/rpm-gpg/RPM-GPG-KEY-rpmfusion-free-el-8
```

## 其他

```bash
sudo rm rpmfusion-free-updates-testing.repo
sudo dnf makecache
```
