# install vscode on linux

## 下载安装

[下载地址](https://code.visualstudio.com/Download)

解压

Desktop 文件

`/usr/share/applications/code.desktop`

```text
[Desktop Entry]
Name=Visual Studio Code
Comment=Code Editing. Redefined.
GenericName=Text Editor
Exec=执行文件绝对路径 --no-sandbox --unity-launch %F
Icon=com.visualstudio.code
# Icon 可以在解压后的文件里找到
Type=Application
StartupNotify=false
StartupWMClass=Code
Categories=Utility;TextEditor;Development;IDE;
MimeType=text/plain;inode/directory;
Actions=new-empty-window;
Keywords=vscode;

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=执行文件绝对路径 --no-sandbox --unity-launch %F
Icon=com.visualstudio.code
# Icon 可以在解压后的文件里找到
```

## 插件

```text
Anaconda Extension Pack
Chinese Language Pack
MagicPython
Markdown PreView Enhanced
markdownlint
Python
Python Extension Pack
Python Indent
Rainbow CSV
YAML
Visual Studio IntelliCode
Prettify JSON
```

## 错误

```text
error while loading shared libraries: libXss.so.1
```

```bash
dnf install libXScrnSaver
```
