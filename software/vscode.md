# install vscode on linux

## 下载安装

[下载地址](https://code.visualstudio.com/Download)

把下载链接的 `https://az764295.vo.msecnd.net` 替换为 `http://vscode.cdn.azure.cn`

[最新下载cdn](http://vscode.cdn.azure.cn/stable/83bd43bc519d15e50c4272c6cf5c1479df196a4d/code-stable-x64-1631295096.tar.gz)

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

## 插件

```text
Bracket Pair Colorizer 2
Chinese Language Pack
EditorConfig for VS Code
Git Graph
gitignore
GitLens
indent-rainbow
Markdown Preview Enhanced
markdownlint
MySQL Syntax
Prettify JSON
Pylance
Python
Python Indent
Python Type Hint
Remote-Containers
Todo Tree
vscode-icons
YAML

Docker
Redis

Better C++ Syntax
C/C++
C/C++ Snippets
C++ Intellisense

Go
Go Doc
Go Ngghtly
Go Outliner
Go To Method
```

## 字体与空格宽度

```text
Consolas, 'Courier New','monospace'
'Droid Sans Mono', 'monospace', monospace, 'Droid Sans Fallback'
```

## 错误

```text
error while loading shared libraries: libXss.so.1
```

```bash
dnf install libXScrnSaver
```
