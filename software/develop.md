# software

[toc]

## 0

开发软件，**PyCharm, VSCode, Idea, Postman, ApiFox, Another-Redis-Desktop-Manager, DBeaver, Studio3T, Oss-Browser, Chrome**

## 1 pycharm

下载解压

### 1.1 权限

```bash
ch /opt/PyCharm/bin
chmod 777 ./*
```

### 1.2 idea.properties

```text
idea.config.path=/opt/PyCharm/PyCharmCE/config
idea.system.path=/opt/PyCharm/PyCharmCE/system
idea.plugins.path=/opt/PyCharm/PyCharmCE/config/plugins
idea.log.path=/opt/PyCharm/PyCharmCE/system/log
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
Exec=sh /opt/PyCharm-202112/bin/pycharm.sh
Icon=/opt/PyCharm-202112/bin/pycharm.png
Categories=Development;IDE;
Actions=new-empty-window;
Terminal=PyCharm

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=sh /opt/PyCharm-202112/bin/pycharm.sh
Icon=/opt/PyCharm-202112/bin/pycharm.png
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

## 2 vscode

### 2.1 下载安装

[下载地址](https://code.visualstudio.com/Download)

把下载链接的 `https://az764295.vo.msecnd.net` 替换为 `http://vscode.cdn.azure.cn`

V1.79.2

[默认](https://az764295.vo.msecnd.net/stable/695af097c7bd098fbf017ce3ac85e09bbc5dda06/code-stable-x64-1686734446.tar.gz)

[最新下载 linux-x86-64](https://vscode.cdn.azure.cn/stable/695af097c7bd098fbf017ce3ac85e09bbc5dda06/code-stable-x64-1686734446.tar.gz)

[最新下载 apple m1](https://vscode.cdn.azure.cn/stable/695af097c7bd098fbf017ce3ac85e09bbc5dda06/VSCode-darwin-arm64.zip)

[最新下载 windows](https://vscode.cdn.azure.cn/stable/695af097c7bd098fbf017ce3ac85e09bbc5dda06/VSCode-win32-x64-1.79.2.zip)

解压

Desktop 文件

`/usr/share/applications/code.desktop`

```text
[Desktop Entry]
Name=Visual Studio Code
Comment=Code Editing. Redefined.
GenericName=Text Editor
Exec=/opt/VSCode/bin/code --no-sandbox --unity-launch %F
Icon=/opt/VSCode/resources/app/resources/linux/code.png
Type=Application
StartupNotify=false
StartupWMClass=Code
Categories=Utility;TextEditor;Development;IDE;
MimeType=text/plain;inode/directory;
Actions=new-empty-window;
Keywords=vscode;

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=/opt/VSCode/bin/code --no-sandbox --unity-launch %F
Icon=/opt/VSCode/resources/app/resources/linux/code.png
```

### 2.2 user.json

- macos `/Users/user/Library/Application Support/Code/User/settings.json`
- linux `~/.config/Code/User/settings.json`

```json
{
    "files.autoSave": "afterDelay",
    "workbench.iconTheme": "vscode-icons",
    "workbench.colorTheme": "Monokai",
    "workbench.editor.scrollToSwitchTabs": true,
    "workbench.colorCustomizations": {
        "editor.lineHighlightBackground": "#dd1b1b23",
        "editor.lineHighlightBorder": "#f030304d",
    },
    "security.workspace.trust.untrustedFiles": "open",
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
    "go.formatTool": "goformat",
    "python.analysis.inlayHints.functionReturnTypes": true,
    "[python]": {
        "editor.formatOnType": true
    },
    "debug.console.fontSize": 14,
    "terminal.integrated.fontSize": 14,
    "scm.inputFontSize": 14,
    "interactiveSession.editor.fontSize": 13
}
```

### 2.3 插件

| 插件 | 作用 | 其他 |
| :-: | :-: | :-: |
| Chinese Language | 中文 | 通用 |
| vscode-icons | 文件图标 | 通用 |
| Better Comments | 注释 | 通用 |
| Todo Tree | TODO 注释 | 通用 |
| Trailing Spaces | 空格检查 | 通用 |
| Code Spell Checker | 拼写检查 | 通用 |
| indent-rainbow | 缩进展示 | 通用 |
| Better TOML | 语言语法支持 | 文件 |
| Prettify JSON | 格式化 | 文件 |
| XML Tools | XML | 文件 |
| Markdown Preview Enhanced | 展示 | 文件 |
| markdownlint | 语言语法支持 | 文件 |
| CMake | 语言语法支持 | CMake |
| Better C++ Syntax | C++ 语法支持 | C/C++ |
| C/C++ | IntelliSense/Debugging/Code Browsing | C/C++ |
| Git Graph | View Git Graph | Git |
| gitignore |  | Git |
| GitLens |  | Git |
| Go | Go 语言语法支持 | Go |
| Python | 语言语法支持 | Python |
| Pylance | 语言语法支持 | Python |
| Python Indent | Python 缩进 | Python |
| Python Type Hint | 类型标记 | Python |
| autoDocstring - Python Docstring Generator |  | Python |
| Dev Containers | 进入容器 | Container |
| Go Doc |  | Go 可不用 |
| CMake Tools |  | CMake 可不用 |
| C/C++ Snippets | Code Snippets | C/C++ 可不用 |

### 2.4 字体与空格宽度

```text
Consolas, 'Courier New','monospace'
'Droid Sans Mono', 'monospace', monospace, 'Droid Sans Fallback'
```

### 2.5 错误

```text
error while loading shared libraries: libXss.so.1
```

```bash
dnf install libXScrnSaver
```

## 3 Idea

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
Exec=/opt/Postman/app/Postman
Icon=/opt/Postman/app/resources/app/assets/icon.png
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

## 5 ApiFox

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

## 5 ApiPost

```text
[Desktop Entry]
Actions=new-empty-window;
Categories=Development;
Comment=ApiPost
GenericName=ApiPost
Keywords=api;
Name=ApiPost
StartupNotify=false
StartupWMClass=ApiFox
Type=Application
X-Deepin-Vendor=user-custom
Exec=/extend/opt/apipost/apipost7
Icon=/extend/opt/apipost/icon.ico

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=/extend/opt/apipost/apipost7
Icon=/extend/opt/apipost/icon.ico
```

## 7 Another-Redis-Desktop-Manager

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

## 8 dbeaver

### 8.1 下载

[github](https://github.com/dbeaver/dbeaver/releases)
[代下-1](https://www.offcloud.com/)
[代下-2](https://shrill-pond-3e81.hunsh.workers.dev/)

### 8.2 安装-配置

窗口->首选项->连接->驱动->maven->添加

```text
name
https://maven.aliyun.com/repository/public
```

### 8.3 build

```bash
git clone https://github.com/dbeaver/dbeaver.git dbeaver
git clone xxx.xxx.xxx.com/dbeaver.git dbeaver
cd dbeaver
# 需要已经配置 java maven
mvn clean package
```

### 8.4 desktop

```text
[Desktop Entry]
Version=1.0
Type=Application
Name=DBeaver Community
GenericName=UniversaL Database Manager
Comment=Universal Database Manager and SQL Client.
Path=/opt/dbeaver/
Exec=/opt/dbeaver/dbeaver
Icon=/opt/dbeaver/dbeaver.png
Categories=IDE;Development
WM_CLASS=DBeaver
StartupWMClass=DBeaver
StartupNotify=true
Keywords=Database;SQL;IDE;JDBC;ODBC;MySQL;PostgreSQL;Oracle;DB2;MariaDB
MimeType=application/sql
Terminal=false
```

## 9 Studio3T

```text
#!/usr/bin/env xdg-open
[Desktop Entry]
Type=Application
Name=Studio 3T
Exec="/extend/opt/studio3t/Studio-3T"  %U
Icon=/extend/opt/studio3t/.install4j/i4j_extf_2_kzs8aq_703o14.png
StartupWMClass=install4j-t3-dataman-mongodb-app-Studio3TAp
```

## 10 Oss-Browser

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

## 11 chrome

### 11.1 下载链接

[下载链接chrome64bit.com](https://www.chrome64bit.com/index.php/google-chrome-64-bit-for-linux)

### 11.2 install Error

```bash
sudo dnf provides */xxxx
sudo dnf install libappindicator-gtk3
sudo dnf install liberation-fonts
sudo dnf install vulkan-loader
```
