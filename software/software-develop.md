# software

[toc]

开发软件，**PyCharm, VSCode, Chrome, Postman, dbeaver, Oss-Browser, Idea, ApiFox**

## 1 pycharm

下载解压

### 1.1 权限

```bash
ch /opt/PyCharm/bin
chmod 777 ./*
```

### 1.2 idea.properties

```text
idea.config.path=/home/opt/PyCharm/PyCharmCE/config
idea.system.path=/home/opt/PyCharm/PyCharmCE/system
idea.plugins.path=/home/opt/PyCharm/PyCharmCE/config/plugins
idea.log.path=/home/opt/PyCharm/PyCharmCE/system/log
```

### 1.3 pycharm64.vmoptions

```text
-Xms150m
-Xmx500m
-XX:ReservedCodeCacheSize=350m
-XX:+UseConcMarkSweepGC
-XX:SoftRefLRUPolicyMSPerMB=100
```

### 1.4 桌面图标

```text
[Desktop Entry]
Type=Application
Name=PyCharm
GenericName=IDE
Comment=Python IDE
Exec=sh /home/opt/PyCharm-202112/bin/pycharm.sh
Icon=/home/opt/PyCharm-202112/bin/pycharm.png
Categories=Development;IDE;
Actions=new-empty-window;
Terminal=PyCharm

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=sh /home/opt/PyCharm-202112/bin/pycharm.sh
Icon=/home/opt/PyCharm-202112/bin/pycharm.png
Terminal=PyCharm
```

### 1.5 插件

```text
.ignore  ATOM  Rainbow Brackets
```

可以先移除的

```text
cwm-plugin  cwm-plugin-projector  editorconfig  github  markdown  space  svn4idea  yaml
```

## 2 chrome

### 2.1 下载链接

[下载链接chrome64bit.com](https://www.chrome64bit.com/index.php/google-chrome-64-bit-for-linux)

### 2.2 install Error

```bash
sudo dnf provides */xxxx
sudo dnf install libappindicator-gtk3
sudo dnf install liberation-fonts
sudo dnf install vulkan-loader
```

## 3 vscode

### 3.1 下载安装

[下载地址](https://code.visualstudio.com/Download)

把下载链接的 `https://az764295.vo.msecnd.net` 替换为 `http://vscode.cdn.azure.cn`

[最新下载 linux cdn](http://vscode.cdn.azure.cn/stable/784b0177c56c607789f9638da7b6bf3230d47a8c/code-stable-x64-1662018655.tar.gz)

[最新下载 mac m1 cdn](http://vscode.cdn.azure.cn/stable/784b0177c56c607789f9638da7b6bf3230d47a8c/VSCode-darwin-arm64.zip)

解压

Desktop 文件

`/usr/share/applications/code.desktop`

```text
[Desktop Entry]
Name=Visual Studio Code
Comment=Code Editing. Redefined.
GenericName=Text Editor
Exec=/home/opt/VSCode/bin/code --no-sandbox --unity-launch %F
Icon=/home/opt/VSCode/resources/app/resources/linux/code.png
Type=Application
StartupNotify=false
StartupWMClass=Code
Categories=Utility;TextEditor;Development;IDE;
MimeType=text/plain;inode/directory;
Actions=new-empty-window;
Keywords=vscode;

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=/home/opt/VSCode/bin/code --no-sandbox --unity-launch %F
Icon=/home/opt/VSCode/resources/app/resources/linux/code.png
```

```json
{
    "files.autoSave": "afterDelay",
    "workbench.iconTheme": "vscode-icons",
    "workbench.colorTheme": "Monokai",
    "workbench.editor.scrollToSwitchTabs": true,
    "security.workspace.trust.untrustedFiles": "open",
    "workbench.colorCustomizations": {
        "editor.lineHighlightBackground": "#dd1b1b23",
        "editor.lineHighlightBorder": "#f030304d",
    },
    "editor.fontFamily": "Consolas, 'Courier New','monospace'",
    "editor.tabCompletion": "on",
    "editor.detectIndentation": false,
    "editor.tabSize": 4,
    "editor.fontSize": 14,
    "editor.formatOnSave": true,
    "editor.formatOnPaste": true,
    "editor.rulers": [
        121
    ],
    "editor.bracketPairColorization.enabled": true,
    "editor.renderLineHighlight": "all",
    "cSpell.blockCheckingWhenAverageChunkSizeGreaterThan": 121,
    "cSpell.showAutocompleteSuggestions": true,
    "todo-tree.general.showIconsInsteadOfTagsInStatusBar": true,
    "[json]": {
        "editor.defaultFormatter": "vscode.json-language-features"
    },
    "[jsonc]": {
        "editor.defaultFormatter": "vscode.json-language-features"
    },
    "go.formatTool": "goformat"
}
```

### 3.2 插件

```text
Chinese Language
Better Comments             高亮注释
Todo Tree                   高亮快速找到 TODO FIXME
vscode-icons                图标
Remnote-Containers          连接进入容器

Better TOML                 toml
MySQL Syntax                mysql语法辅助-remove
Markdown Preview Enhanced   md 展示
markdownlint                md 语法
Prettify JSON               json
YAML                        yaml-remove

Git Graph
gitignore
GitLens

Bracket Pair Colorizer 2    括号匹配，不再维护
Rainbow Brackets            括号匹配 remove vscode 自带了相似功能
indent-rainbow              缩进深度
Trailing Spaces             多余空格

Pylance
Python
Python Docstring Generator  生成文档
Python Indent
Python Type Hint            类型标记

C/C++
C++ Intellisense
Better C++ Synatx

Cython                      remove
Language-Cython             remove
vscode-cython-annotate      remove

Go
Go Doc
Go Nightly                  stop
Go Outliner                 remove
```

### 3.3 字体与空格宽度

```text
Consolas, 'Courier New','monospace'
'Droid Sans Mono', 'monospace', monospace, 'Droid Sans Fallback'
```

### 3.4 错误

```text
error while loading shared libraries: libXss.so.1
```

```bash
dnf install libXScrnSaver
```

## 4 postman

## 4.1 下载

[下载地址](https://www.postman.com/downloads/)

### 4.2 desktop

```text
[Desktop Entry]
Type=Application
Name=Postman
Encoding=UTF-8
Categories=Development;
Exec=/home/opt/Postman/app/Postman
Icon=/home/opt/Postman/app/resources/app/assets/icon.png
Terminal=false
```

### 4.3 error

```bash
# error while loading shared libraries: libXss.so.1
dnf install libXScrnSaver
```

### 4.4 变量

```javascript
//把json字符串转化为对象
var data=JSON.parse(responseBody);

//获取data对象的 token 值。
var token=data.token;

//设置成全局变量
pm.globals.set("token_local", token);


var csrf_token = postman.getResponseCookie("csrftoken").value;
// postman.clearGlobalVariable("local_csrf");
postman.setGlobalVariable("local_csrf", csrf_token);
```

```javascript
//把json字符串转化为对象
var data=JSON.parse(responseBody);
// 获取data对象的utoken值。
var token=data.data.access_token;
111
// 环境变量
pm.environment.set("dji_token_local", token);
222
//设置成全局变量
pm.globals.set("token_local", token);
```

## 5 dbeaver

### 5.1 下载

[github](https://github.com/dbeaver/dbeaver/releases)
[代下-1](https://www.offcloud.com/)
[代下-2](https://shrill-pond-3e81.hunsh.workers.dev/)

### 5.2 安装-配置

窗口->首选项->连接->驱动->maven->添加

```text
name
https://maven.aliyun.com/repository/public
```

### 5.3 build

```bash
git clone https://github.com/dbeaver/dbeaver.git dbeaver
git clone xxx.xxx.xxx.com/dbeaver.git dbeaver
cd dbeaver
# 需要已经配置 java maven
mvn clean package
```

### 5.4 desktop

```text
[Desktop Entry]
Version=1.0
Type=Application
Name=DBeaver Community
GenericName=UniversaL Database Manager
Comment=Universal Database Manager and SQL Client.
Path=/home/opt/dbeaver/
Exec=/home/opt/dbeaver/dbeaver
Icon=/home/opt/dbeaver/dbeaver.png
Categories=IDE;Development
WM_CLASS=DBeaver
StartupWMClass=DBeaver
StartupNotify=true
Keywords=Database;SQL;IDE;JDBC;ODBC;MySQL;PostgreSQL;Oracle;DB2;MariaDB
MimeType=application/sql
Terminal=false
```

## 6 Oss-Browser

OSS 浏览器

```text
[Desktop Entry]
Type=Application
Name=Oss-Browser
Encoding=UTF-8
Categories=Development;
Exec=/extend/opt/oss-browser/oss-browser
Icon=/extend/opt/oss-browser/resources/custom/icon.png
Terminal=false
```

## 7 Idea

```text
[Desktop Entry]
Type=Application
Name=Idea
GenericName=Java IDE
Comment=Idea
Exec=sh /extend/opt/idea/bin/idea.sh
Icon=/extend/opt/idea/bin/idea.png
Categories=Development;IDE;
Actions=new-empty-window;
Terminal=Idea

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=sh /extend/opt/idea/bin/idea.sh
Icon=/extend/opt/idea/bin/idea.png
Terminal=Idea
```

## 8 ApiFox

```text
[Desktop Entry]
Name=ApiFox
Comment=ApiFox
GenericName=ApiFox
Exec=/extend/opt/apifox/Apifox.AppImage
Icon=/extend/opt/apifox/icon.png
Type=Application
StartupNotify=false
StartupWMClass=ApiFox
Categories=Development;
Actions=new-empty-window;
Keywords=api;

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=/extend/opt/apifox/Apifox.AppImage
Icon=/extend/opt/apifox/icon.png
```

## 9 Another-Redis-Desktop-Manager

```text
[Desktop Entry]
Name=RedisManager
Comment=Redis Desktop Manager
GenericName=Redis Desktop Manager
# Exec=/extend/opt/ARDM/another-redis-desktop-manager --no-sandbox --unity-launch %F
Exec=/extend/opt/ARDM/Another-Redis-Desktop-Manager.1.5.1.AppImage
Icon=/extend/opt/ARDM/icon.png
# Icon 可以在解压后的文件里找到
Type=Application
StartupNotify=false
StartupWMClass=RedisManager
Categories=Development;
Actions=new-empty-window;
Keywords=redis;

[Desktop Action new-empty-window]
Name=New Empty Window
# Exec=/extend/opt/ARDM/another-redis-desktop-manager --no-sandbox --unity-launch %F
Exec=/extend/opt/ARDM/Another-Redis-Desktop-Manager.1.5.1.AppImage
Icon=/extend/opt/ARDM/icon.png
```
