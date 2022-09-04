# 安装软件过程中的奇怪问题

[toc]

Apple M1

## 0 软件

- brew
- git tree fzf
- ohmyzsh
- AnotherRedisManagerDesktop
- miniconda
- PyCharm
- VSCode
- iTerm2
- dbeaver
- jdk
- 百度网盘
- GoogleChrome
- docker desktop
- 向日葵
- 微信
- Postman

## 1 Pycharm

下载后，-> Applications

```bash
sudo xattr -r -d com.apple.quarantine /Applications/PyCharm.app
```

## 2 brew

引发了 git, xcode, brew, fzf, tree

### 2.1 设置 brew 清华源

```text
export HOMEBREW_BREW_GIT_REMOTE="https://mirrors.tuna.tsinghua.edu.cn/git/homebrew/brew.git"
export HOMEBREW_CORE_GIT_REMOTE="https://mirrors.tuna.tsinghua.edu.cn/git/homebrew/homebrew-core.git"
export HOMEBREW_BOTTLE_DOMAIN="https://mirrors.tuna.tsinghua.edu.cn/homebrew-bottles"
export HOMEBREW_CASK_GIT_REMOTE="https://mirrors.tuna.tsinghua.edu.cn/git/homebrew/homebrew-cask.git"
```

### 2.2 安装 brew

```bash
/bin/bash -c "$(curl -fsSL https://gitee.com/ineo6/homebrew-install/raw/master/install.sh)"
```

### 2.3 git fzf tree

安装完后，安装 ohmyzsh 把 brew 源变量设置到 .zshrc

```bash
brew install git fzf tree
```

## 3 docker

### 3.1 docker.sock

```text
/private/var/run/docker.sock -> /Users/user/.docker/run/docker.sock

/private/var/dirs_cleaner/jP/docker.sock -> /Users/user/.docker/run/docker.sock

/var/run/docker.sock -> /Users/user/.docker/run/docker.sock
```

### 3.2 镜像，容器数据

<https://blog.csdn.net/m0_38112165/article/details/120116336>

```text
1 启动一个容器
2 进入容器
3 cd /var/lib/docker/containers
4 查看各个容器数据，可以删除过多的日志
```

### 3.3 镜像不同指令集

arm64 aarch64
