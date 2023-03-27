# Jenkins 记录

## 下载安装

### 下载地址

[清华大学镜像站 war 文件](https://mirrors.tuna.tsinghua.edu.cn/jenkins/war-stable/)

### 安装

```bash
JENKINS_HOME=$HOME/softer/jenkins java -jar jenkins.war --help
JENKINS_HOME=/opt/jenkins java -jar jenkins.war
```

### 换插件源

终止后

1. 修改 `hudson.model.UpdateCenter.xml`：

    把原先的 `<url>https://updates.jenkins.io/update-center.json</url>`

    修改为 `<url>https://mirrors.tuna.tsinghua.edu.cn/jenkins/updates/update-center.json</url>`

2. 修改 `updates/default.json`：
    原因: 镜像上的 `update-center.json` 指向境外.

    镜像上和本地各有一个映射文件, 修改本地的,来达到效果

    ```bash
    sed -i 's/http:\/\/updates.jenkins-ci.org\/download/https:\/\/mirrors.tuna.tsinghua.edu.cn\/jenkins/g' updates/default.json

    sed -i 's/http:\/\/www.google.com/https:\/\/www.baidu.com/g' updates/default.json
    ```

### 安装推荐插件

如果下载速度慢或下载失败,则在换插件源结束后,重启 jenkins

### 密码

- 初始密码,`secrets/initialAdminPassword`

- 在用户管理里更改

## 使用与管理Jenkins

### 更新

### 工作空间

### 密钥等
