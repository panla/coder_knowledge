# Screen

[TOC]

## 1

用户可以通过该软件同时连接多个本地或远程的命令行会话，并在其间自由切换。
GNU Screen可以看作是窗口管理器的命令行界面版本。
它提供了统一的管理多个会话的界面和相应的功能。

screen tmux byobu

## 2 install

```bash
apt install screen

dnf install screen
```

## 3 command

- 显示所有，screen -ls
- 创建新的，screen -S name
- 进入已有的，screen -r name
- 远程退出目前，screen -d name

## 4 bind

- C-a d 退出
- C-a c 创建新 shell 窗口

## 5 开机自启动

```text
1
    先编写一个程序运行脚本 【不能后台运行】

2
    界面自启动，ubuntu 桌面启动后才启动
    Startup Application

    screen -dmS $Name [-L -Logfile screen] 日志路径 第一步中的脚本绝对路径

3
    screen -r $Name
    Ctrl A 前缀指令
    Ctrl A + D
```
