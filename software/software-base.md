# software

[toc]

环境，**Git, SSH, JDK, Maven, vim, tmux, ohmyzsh**

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

github.global.ssl.fastly.net

assets-cdn.github.com

avatars1.githubusercontent.com

====================================================
199.232.69.194 github.global.ssl.fastly.net

140.82.112.4 github.com

185.199.108.153 assert-cdn.github.com

185.199.108.133 avatars1.githubusercontent.com

====================================================

修改 /etc/resolv.conf 新增
nameserver 8.8.8.8
nameserver 8.8.4.4
```

## 2 ssh config

```text
Host test
    HostName ip or server_name
    User user
    Port 22
    IdentityFile ~/.ssh/id_rsa
```

## 3 jdk jre

```text
export JAVA_HOME=/home/opt/jdk-11.0.9
export CLASSPATH=.:${JAVA_HOME}/lib
export PATH=${JAVA_HOME}/bin:$PATH
```

## 4 maven

### 4.1 下载

[maven.apache.org](http://maven.apache.org/download.cgi)

解压到 `/home/opt/apache-maven`

### 4.2 setting.xml 配置

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
<localRepository>/home/opt/apache-maven-3.6.3/repository</localRepository>
```

### 4.3 profile

```text
export PATH="/opt/apache-maven/bin:$PATH"
```

## 5 vim 部分配置

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

## 6 tmux

```bash
git clone git@github.com:gpakosz/.tmux.git
```

```text
# ~/.tmux.conf

set -g prefix C-a

unbind C-b

set -g default-terminal "screen-256color"
```

## 7 ohmyzsh

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

插件

```text
plugins=(
         git compleat sudo docker fzf z 
         zsh-interactive-cd 
         zsh-autosuggestions 
         zsh-syntax-highlighting 
         zsh-history-substring-search
)
```
