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
C/C++
C/C++ Snippets
C++ Intellisense
Chinese Language Pack
EditorConfig for VS Code
flask-snippets
Git Graph
Git History
Jupyter
Markdown PreView Enhanced
markdownlint
MySQL Syntax
Prettify JSON
Pylance
Python
Python Extension Pack
Python Indent
Python snippets
Rainbow CSV
Visual Studio IntelliCode
YAML
```

## 字体与空格宽度

```text
Consolas, 'Courier New','monospace'
```

## 错误

```text
error while loading shared libraries: libXss.so.1
```

```bash
dnf install libXScrnSaver
```
