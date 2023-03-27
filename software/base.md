# BASE

[toc]

## 0

开发软件，**git, ssh, vim, ohmyzsh, tmux, JDK, maven**

## 1 git

### 1.1 git 部分配置

- 生成公钥私钥

```bash
ssh-keygen -t rsa -C "email"
```

- 配置用户名,邮箱

```bash
git config --global user.email "email"
git config --global user.name "name"
git config --global core.quotepath false
git config --global pull.rebase false
```

- vim

```bash
git config --global core.editor "vim"
```

- 保存账户密码

```bash
git config --global credential.helper store
```

### 1.2 别名

```bash
git config --global alias.co checkout
git config --global alias.ci commit
git config --global alias.st status
```

### 1.3 加速？

```text
https://www.ipaddress.com/

github.com
```

### 1.4 代理

```bash
git config --global http.proxy http://ip:port
```

## 2 ssh 公私钥连接

### 2.2 cmd

```bash
ssh user@地址
ssh 别名
```

### 2.1客户端

`~/.ssh/config`

```text
Host 别名
    HostName ip地址
    Port 端口
    User 远程用户名
    IdentityFile ~/.ssh/id_rsa 本机私钥
```

```text
Host test
    HostName ip or server_name
    User user
    Port 22
    IdentityFile ~/.ssh/id_rsa
```

### 2.2 服务器

`/etc/ssh/sshd_config`

```text
Port 22
ListenAddress 0.0.0.0
ListenAddress ::

HostKey /etc/ssh/ssh_host_rsa_key
HostKey /etc/ssh/ssh_host_ecdsa_key
HostKey /etc/ssh/ssh_host_ed25519_key

# 是否允许root登录
PermitRootLogin no

# 是否允许公钥登录
PubkeyAuthentication yes
AuthorizedKeysFile %h/.ssh/authorized_keys

# 是否允许密码登录
PasswordAuthentication no
```

服务器 `~/.ssh/authorized_keys` 上加入公钥 并且 `chmod 400`

## 3 Vim

`~/.vimrc`

```text
set fileencodings=utf-8,gb18030
set fileencoding=utf-8
set termencoding=utf-8
set encoding=utf-8

set nu

set cursorline

set mouse=a
set selection=exclusive
set selectmode=mouse,key

set t_Co=256

set showmatch
set tabstop=4
set expandtab
set shiftwidth=4
set autoindent
set backspace=indent,eol,start

set paste
set listchars=tab:>-,trail:-

if has( 'mouse' )
    set mouse-=a
endif
```

`/etc/vim/vimrc`

```text
syntax on

set showcmd
set showmatch
set smartcase
set incsearch
set mouse=a
```

## 4 ohmyzsh

### 4.1 install

```bash
git clone https://gitee.com/pankla/ohmyzsh.git oh-my-zsh
cd oh-my-zsh && sh ./tools/upgrade

install zsh

# 切换 shell

# 重启

sudo apt install fzf

cd .oh-my-zsh/custom/plugins
git clone git@gitee.com:pankla/zsh-autosuggestions.git
git clone git@gitee.com:pankla/zsh-syntax-highlighting.git
git clone https://github.com/zsh-users/zsh-history-substring-search
```

### 4.2 插件

```text
plugins=(
         git compleat sudo docker fzf z 
         zsh-interactive-cd 
         zsh-autosuggestions 
         zsh-syntax-highlighting 
         zsh-history-substring-search
)
```

## 5 tmux

```bash
git clone git@github.com:gpakosz/.tmux.git
```

```text
# ~/.tmux.conf

set -g prefix C-a

unbind C-b

set -g default-terminal "screen-256color"
```

## 6 JDK

```text
export JAVA_HOME=/opt/jdk-11.0.9
export CLASSPATH=.:${JAVA_HOME}/lib
export PATH=${JAVA_HOME}/bin:$PATH
```

## 7 maven

### 7.1 下载

- [maven.apache.org](http://maven.apache.org/download.cgi)

解压到 `/opt/apache-maven`

### 7.2 setting.xml 配置

`conf/settings.xml`

一 阿里云镜像

```text
<mirrors>
  <mirror>
    <id>aliyunmaven</id>
    <mirrorOf>*</mirrorOf>
    <name>Aliyun Mirror.</name>
    <url>https://maven.aliyun.com/repository/public</url>
  </mirror>
</mirrors>
```

二 本地缓存目录

```text
<localRepository>/opt/apache-maven-3.6.3/repository</localRepository>
```

### 7.3 profile

```text
export PATH="/opt/apache-maven/bin:$PATH"
export M2_HOME="/extend/opt/apache-maven"
```
