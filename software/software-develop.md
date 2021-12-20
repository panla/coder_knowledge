# software

[toc]

开发软件，PyCharm, VSCode, Chrome, Postman

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

[最新下载cdn](http://vscode.cdn.azure.cn/stable/899d46d82c4c95423fb7e10e68eba52050e30ba3/code-stable-x64-1639562789.tar.gz)

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
Rainbow Brackets            括号匹配
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
